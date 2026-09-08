#include "CamRobot.h"

#include <cstring>
#include <vector>

#include <RadioSeal.h>

#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"

bool CamRobot::configureIdentity() noexcept {
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_result = nvs_flash_init();
    }
    if (nvs_result != ESP_OK) return false;

    std::uint8_t mac[6] = {};
    esp_efuse_mac_get_default(mac);
    source_id_ = btp_command::source_id_from_mac(mac);

    // boot_id increments every boot, persisted in NVS -- 0 is reserved by
    // BTP, so starting the counter at 1 keeps it non-zero forever.
    nvs_handle_t handle = 0U;
    if (nvs_open("btp", NVS_READWRITE, &handle) == ESP_OK) {
        std::uint32_t last_boot = 0U;
        nvs_get_u32(handle, "last_boot", &last_boot);
        boot_id_ = last_boot + 1U;
        nvs_set_u32(handle, "last_boot", boot_id_);
        nvs_commit(handle);
        nvs_close(handle);
    } else {
        boot_id_ = 1U;
    }

    // Same MAC->source_id conversion as this device's own identity above,
    // applied to the dongle's known MAC (the same build flag the ESP-NOW
    // peer registration in main.cpp reads). Left 0U -- an invalid source_id
    // CommandProcessor::intake() already rejects -- when this build has no
    // MAC_ADDR at all.
#ifdef MAC_ADDR
    {
        static constexpr std::uint8_t kDongleMac[6] = {MAC_ADDR};
        dongle_source_id_ = btp_command::source_id_from_mac(kDongleMac);
    }
#endif

    // Binds RadioSeal to key_store regardless of load order: seal()/open()
    // read the KeyStore's state at call time, and both already fail closed
    // on !loaded() -- see RadioSeal.h. Safe to do before
    // key_store.load_from_card() runs later in main.cpp.
    RadioSeal::configure(key_store);

    // COMMAND_RESULT is sealed with whichever channel's key opened the
    // request it answers: seal_link for channel C (dongle, key L),
    // seal_endpoint for channel B (TraceView, key E).
    command_processor.configure(protocol, RadioSeal::seal, nullptr,
                                RadioSeal::seal_e, nullptr);
    // TELEMETRY is channel B (bally_channels.h): TraceView holds key E, the
    // dongle relays the samples and never reads them.
    telemetry.configure(protocol, RadioSeal::seal_e, nullptr);
    // STATUS (heartbeat) is channel C by definition.
    status_reporter.configure(protocol, telemetry, RadioSeal::seal, nullptr);
    // TERMINAL_OUT is channel B: TraceView's terminal widget holds key E,
    // the dongle relays the stream verbatim. Tab completion runs against
    // this device's real TinyShell catalog.
    terminal_responder.configure(
        protocol, RadioSeal::seal_e, nullptr,
        [this](const std::string& input, std::string* out,
               std::size_t max_out) -> std::size_t {
            if (out == nullptr || max_out == 0U) return 0U;
            const std::vector<std::string> matches =
                shell.complete_line(input, max_out);
            std::size_t written = 0U;
            for (const std::string& match : matches) {
                if (written >= max_out) break;
                out[written++] = match;
            }
            return written;
        },
        &CamRobot::submitTerminalCommandStatic, this, "cam> ");
    return true;
}

void CamRobot::bindProtocolTransport() noexcept {
    protocol_link_.source_id = source_id_;
    protocol_link_.boot_id = boot_id_;
    protocol_link_.transport = btp::kEspNowTransport;

    node_.emplace(protocol_link_, kNodeReassemblyTimeoutMs);
    node_->topic(TelemetryPublisher::kCameraStatusTopicId,
                TelemetryPublisher::kSchemaVersion, "camera.status")
        .u8("state")
        .u32("frame_count")
        .u16("fps", 0.1, "fps")
        .end();
    // OpaqueBytes body (adaptive RAW/RLE grid, see CameraMatrix.h),
    // no wire fields -- same catalog shape bally_OS
    // uses for its own replace-in-place text topic (system.monitor), just a
    // different encoding tag. Rates mirror TelemetryPublisher's own
    // camera.matrix TopicSchema entry (100 Hz is a permissive ceiling and
    // default; the sustainable rate still depends on frame delivery).
    node_->topic(TelemetryPublisher::kCameraMatrixTopicId,
                TelemetryPublisher::kCameraMatrixSchemaVersion, "camera.matrix",
                /*fill=*/nullptr, /*fill_ctx=*/nullptr,
                btp::TelemetryEncoding::OpaqueBytes, /*subscribable=*/true,
                /*max_rate_millihz=*/100000U)
        .min_rate(100U)
            .default_rate(100000U)
        .end();
    node_->begin();
    node_->serve_catalog();
    telemetry.bind_subscriptions(*node_->subscriptions());
    protocol.bind(node_->endpoint());
}

bool CamRobot::handleCompleteMessage(const btp::Header& header,
                                     btp::ByteView payload,
                                     std::uint64_t now_us) noexcept {
    if (header.type != btp::MessageType::Command ||
        header.object_id != btp_command::kCommandRequestObjectId) {
        return false;
    }

    const bally::Channel channel =
        CamLink::classify_channel(header, dongle_source_id_);
    const CommandProcessor::Intake intake =
        command_processor.intake(header, payload, now_us, channel);

    switch (intake.kind) {
        case CommandProcessor::IntakeKind::Ready: {
            QueuedCommand cmd{};
            cmd.from_terminal = false;
            cmd.cache_slot = intake.work.cache_slot;
            std::strncpy(cmd.text, intake.work.command, sizeof(cmd.text) - 1U);
            if (shell_queue_ == nullptr ||
                xQueueSend(shell_queue_, &cmd, 0) != pdTRUE) {
                CommandProcessor::ResultView result{};
                if (command_processor.reject_busy(cmd.cache_slot, now_us,
                                                  &result)) {
                    command_processor.send_result(result);
                }
            }
            return true;
        }
        case CommandProcessor::IntakeKind::ResultReady:
            command_processor.send_result(intake.result);
            return true;
        case CommandProcessor::IntakeKind::DuplicateInProgress:
        case CommandProcessor::IntakeKind::Ignored:
            return true;
    }
    return true;
}

bool CamRobot::submitTerminalCommandStatic(void* context,
                                           std::uint32_t source_id,
                                           std::uint32_t boot_id,
                                           const char* command_line) noexcept {
    auto* self = static_cast<CamRobot*>(context);
    if (self == nullptr || self->shell_queue_ == nullptr ||
        command_line == nullptr) {
        return false;
    }

    QueuedCommand cmd{};
    cmd.from_terminal = true;
    cmd.terminal_source_id = source_id;
    cmd.terminal_boot_id = boot_id;
    std::strncpy(cmd.text, command_line, sizeof(cmd.text) - 1U);
    return xQueueSend(self->shell_queue_, &cmd, 0) == pdTRUE;
}
