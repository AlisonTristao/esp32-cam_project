#include <cstdio>
#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <btp/node.hpp>

#include "CamRobot.h"
#include "CamShell.h"
#include "Camera.h"
#include "CameraConfig.h"

#ifdef DEV_EMBEDDED_KEY_L
#include "dev_key_l_generated.h"
#endif

namespace {

const char* const kTag = "cam_main";

CamRobot g_robot;
QueueHandle_t g_shell_queue = nullptr;
std::atomic<std::uint64_t> g_frames_rx{0U};

// Accumulates whatever the shell's active command writes through
// TinyShell::output_callback (help dumps, MODULE_NOT_FOUND's embedded help,
// run_command_line()'s own generic status line) so shell_task can hand it to
// TerminalResponder::deliver_command_output() -- mirrors bally_OS's
// Logger::begin_capture/end_capture, minus the multi-task bookkeeping: this
// firmware has exactly one shell task and it calls run_command_line()
// synchronously, so a plain buffer cleared before each command is enough.
std::string g_shell_output;

// Single sink for anything a command wants to mirror back over TERMINAL_OUT:
// TinyShell::set_output_callback() feeds it directly (help dumps, generic
// pass/fail line); register_cam_shell_commands() gets the exact same
// function so cam_status()/cam_threshold()/etc.'s own diagnostic text rides
// along in g_shell_output too, instead of only reaching the local UART via
// printf().
void append_shell_output(const std::string& text) { g_shell_output += text; }

#ifdef MAC_ADDR
constexpr std::uint8_t kDonglePeerMac[6] = {MAC_ADDR};
#endif

std::uint64_t now_us() noexcept {
    return static_cast<std::uint64_t>(esp_timer_get_time());
}

std::uint64_t now_ms() noexcept { return now_us() / 1000ULL; }

// Installed as the TxScheduler's radio callback (main.cpp's own composition
// root, same as bally_OS's setup_system_callbacks()) -- this is the ONLY
// place esp_now_send() is called from, so TxScheduler's "one outstanding
// send at a time" invariant holds.
bool radio_send(void* /*context*/, const std::uint8_t* data,
                std::size_t size) noexcept {
#ifdef MAC_ADDR
    return esp_now_send(kDonglePeerMac, data, size) == ESP_OK;
#else
    (void)data;
    (void)size;
    return false;
#endif
}

void on_espnow_send(const esp_now_send_info_t* /*tx_info*/,
                    esp_now_send_status_t status) {
    g_robot.tx_scheduler.on_delivery(status == ESP_NOW_SEND_SUCCESS);
}

void on_espnow_recv(const esp_now_recv_info_t* info, const uint8_t* data,
                    int len) {
    // Cheap radio prefilter, NOT authentication -- see BtpTransport.h's
    // btp_command::authorized_source comment. The real authorization is the
    // AEAD tag CamLink::open() checks after reassembly.
#ifdef MAC_ADDR
    if (info == nullptr || info->src_addr == nullptr ||
        std::memcmp(info->src_addr, kDonglePeerMac, 6) != 0) {
        return;
    }
#else
    (void)info;
#endif
    if (data == nullptr || len <= 0) return;

    g_frames_rx.fetch_add(1U, std::memory_order_relaxed);

    btp::Node* node = g_robot.node();
    if (node == nullptr) return;

    btp::ReceivedMessage msg{};
    const btp::NodeRx outcome = node->receive(
        data, static_cast<std::size_t>(len), now_ms(), &msg);
    if (outcome == btp::NodeRx::Complete) {
        g_robot.handleCompleteMessage(msg.header, msg.payload, now_us());
    }
}

// Mirrors ROBOT::configureCommunication(): Wi-Fi STA with no AP connection
// (ESP-NOW only), the same channel default bally_dongle's own EspNowManager
// relies on (it never calls esp_wifi_set_channel() either -- see
// EspNowManager::begin()'s own comment: "canal atual do radio").
bool configure_communication() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&wifi_cfg) != ESP_OK) return false;
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) return false;
    if (esp_wifi_start() != ESP_OK) return false;
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(on_espnow_recv);
    esp_now_register_send_cb(on_espnow_send);

#ifdef MAC_ADDR
    esp_now_peer_info_t peer_info{};
    std::memcpy(peer_info.peer_addr, kDonglePeerMac, 6U);
    peer_info.channel = 0;  // current channel, set above
    peer_info.encrypt = false;  // ESP-NOW link itself is clear; BTP/RadioSeal
                                // do the real AEAD (bally_channels.h)
    peer_info.ifidx = WIFI_IF_STA;
    if (esp_now_add_peer(&peer_info) != ESP_OK) return false;
#else
#warning "MAC_ADDR not defined -- this firmware will never authenticate a peer"
#endif
    return true;
}

void comms_task(void* /*arg*/) {
    while (true) {
        const std::uint64_t us = now_us();
        const std::uint64_t ms = us / 1000ULL;

        g_robot.terminal_responder.pump(us);
        g_robot.tx_scheduler.pump(ms);
        if (btp::Node* node = g_robot.node()) node->tick(ms);
        g_robot.telemetry.flush(4U);

        static std::uint64_t next_status_us = 0U;
        if (us >= next_status_us) {
            next_status_us = us + 500000ULL;  // 500 ms heartbeat, same
                                              // period bally_OS's STATUS
                                              // uses to feed hub.peers.online
            const TxScheduler::Stats tx_stats = g_robot.tx_scheduler.stats();
            const CommandProcessor::Stats cmd_stats =
                g_robot.command_processor.stats();
            const TelemetryPublisher::Stats tel_stats =
                g_robot.telemetry.stats();
            btp::Receiver::Stats rx_stats{};
            if (btp::Node* node = g_robot.node()) rx_stats = node->receiver().stats();

            StatusReporter::Counters counters{};
            counters.frames_rx = g_frames_rx.load(std::memory_order_relaxed);
            counters.frames_tx = tx_stats.delivered;
            counters.frames_dropped = tx_stats.dropped;
            counters.crc_errors = rx_stats.dropped_crc;
            counters.decode_errors = rx_stats.dropped_decode;
            counters.reassembly_completed = rx_stats.completed;
            counters.reassembly_timeouts = rx_stats.reassembly_timeouts;
            counters.reassembly_rejected = rx_stats.dropped_reassembly;
            counters.command_duplicates = cmd_stats.duplicates;
            counters.telemetry_dropped = tel_stats.dropped_full +
                                         tel_stats.dropped_invalid +
                                         tel_stats.send_failed;

            g_robot.status_reporter.publish(0U, us, counters, us);
        }

        // Run the communication pump at the FreeRTOS tick rate (1 ms).
        vTaskDelay(1);
    }
}

void shell_task(void* /*arg*/) {
    QueuedCommand cmd{};
    while (true) {
        if (xQueueReceive(g_shell_queue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        g_shell_output.clear();
        const std::uint8_t status = g_robot.shell.run_command_line(cmd.text);
        const std::uint64_t us = now_us();

        if (cmd.from_terminal) {
            g_robot.terminal_responder.deliver_command_output(
                cmd.terminal_source_id, cmd.terminal_boot_id,
                g_shell_output.c_str(), status);
        } else {
            CommandProcessor::ResultView result{};
            if (g_robot.command_processor.complete(cmd.cache_slot, status, us,
                                                   &result)) {
                g_robot.command_processor.send_result(result);
            }
        }
    }
}

}  // namespace

extern "C" void app_main(void) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    puts("BOOT: aplicacao ESP32-CAM (shell + BTP) iniciada");

    if (!g_robot.configureIdentity()) {
        ESP_LOGE(kTag, "Falha ao configurar identidade BTP");
    }

    bool key_loaded = false;
    if (g_robot.sd_card.begin()) {
        key_loaded = g_robot.key_store.load_from_card(g_robot.sd_card);
        if (!key_loaded) {
            ESP_LOGW(kTag,
                     "bally.key invalido/ausente (%s)",
                     KeyStore::error_field(g_robot.key_store.last_error()));
        }

        // Best-effort: an absent/invalid camera.cfg just leaves the atomics
        // at their DEFAULT_* value (Camera.cpp), same as today's behavior
        // before this file existed -- unlike bally.key, nothing is
        // fail-closed on it.
        const cam::ConfigError config_error =
            cam::load_config_from_card(g_robot.sd_card);
        if (config_error == cam::ConfigError::Ok) {
            ESP_LOGI(kTag, "camera.cfg carregado do cartao SD");
        } else if (config_error != cam::ConfigError::File) {
            ESP_LOGW(kTag, "camera.cfg invalido (%s), usando padroes",
                     cam::config_error_field(config_error));
        }
    } else {
        ESP_LOGW(kTag, "Cartao SD nao montado");
    }

#ifdef DEV_EMBEDDED_KEY_L
    // TEMPORARY, bench-only: no SD card provisioned yet (see platformio.ini
    // and scripts/gen_dev_key.py's own comments). Remove this whole block,
    // the DEV_EMBEDDED_KEY_L build flag and the pre:scripts/gen_dev_key.py
    // line once a real card is in use.
    if (!key_loaded) {
        ESP_LOGW(kTag,
                 "DEV_EMBEDDED_KEY_L ativo: usando chave de bancada embutida "
                 "(senha de teste fixa) -- NAO USAR EM PRODUCAO");
        key_loaded = g_robot.key_store.load(kDevKeyFile, sizeof(kDevKeyFile));
        if (!key_loaded) {
            ESP_LOGE(kTag, "Falha ao carregar a chave de bancada embutida (%s)",
                     KeyStore::error_field(g_robot.key_store.last_error()));
        }
    }
#endif

    if (!key_loaded) {
        ESP_LOGW(kTag,
                 "Nenhuma chave L carregada -- canal C ficara fail-closed "
                 "ate bally.key estar disponivel");
    }

    g_robot.shell.begin();
    register_cam_shell_commands(g_robot.shell, append_shell_output,
                               g_robot.sd_card);
    g_robot.shell.set_output_callback(append_shell_output);

    if (!configure_communication()) {
        ESP_LOGE(kTag, "Falha ao inicializar Wi-Fi/ESP-NOW");
    }

    g_robot.tx_scheduler.configure(radio_send, nullptr);
    g_robot.protocol.set_send_callback(TxScheduler::enqueue_callback,
                                       &g_robot.tx_scheduler);
    g_robot.bindProtocolTransport();

    g_shell_queue = xQueueCreate(4, sizeof(QueuedCommand));
    g_robot.set_shell_queue(g_shell_queue);

    // comms needs real headroom: StatusReporter::publish() and
    // TerminalResponder's emit_terminal_out() both go through
    // BtpEndpoint::send_logical(), which stack-allocates a 1936-byte
    // scratch[kMaxLogicalPayloadSize + kAeadTagSize] buffer to seal into
    // before fragmenting -- plus whatever the AEAD call underneath (mbedtls/
    // PSA GCM) needs on top of that. 4096 overflowed on the very first
    // STATUS publish (next_status_us starts at 0).
    //
    // Pinned to core 1 (with "camera"), NOT core 0: core 0 is where
    // ESP-IDF's own Wi-Fi/ESP-NOW driver task runs, at a priority well above
    // this task's 4. Pinning comms there let that driver task starve it for
    // seconds at a time, during which camera.matrix's single replace-in-
    // place stage (TelemetryPublisher::matrix_stage_) sat pending and every
    // fresh capture got dropped -- the image only advancing every few
    // seconds. comms_task's own priority (4) already beats "camera"'s (3),
    // so co-locating them still lets comms preempt camera on demand.
    xTaskCreatePinnedToCore(comms_task, "comms", 12288, nullptr, 4, nullptr,
                            1);
    xTaskCreate(shell_task, "shell", 8192, nullptr, 2, nullptr);
    xTaskCreatePinnedToCore(cam::task, "camera", 8192, &g_robot, 3, nullptr, 1);

    puts("BOOT: pronto");
}
