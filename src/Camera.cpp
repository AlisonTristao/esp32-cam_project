#include "Camera.h"

#define OUTPUT_ROWS 30
// Grade quadrada de verdade (linhas x colunas) -- o dobro de colunas de
// antes so compensava a fonte do monitor serial, que nao se aplica mais
// agora que a matriz vai pelo camera.matrix telemetry, nao pelo console.
#define OUTPUT_COLS OUTPUT_ROWS
#define DEFAULT_BW_THRESHOLD 180

#include <cstdio>
#include <string>

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "CamRobot.h"

static_assert(OUTPUT_ROWS > 0, "OUTPUT_ROWS deve ser positivo");
static_assert(OUTPUT_COLS > 0, "OUTPUT_COLS deve ser positivo");

namespace cam {

std::atomic<std::uint8_t> g_threshold{DEFAULT_BW_THRESHOLD};
std::atomic<std::uint32_t> g_frame_count{0U};
std::atomic<std::uint16_t> g_fps_x10{0U};
std::atomic<std::uint8_t> g_state{0U};

namespace {

const char* const kTag = "binary_camera";

esp_err_t camera_init() {
    // Pinagem da ESP32-CAM AI Thinker com OV2640. Campo a campo (em vez de
    // inicializador designado) porque g++ -- ao contrario do compilador C do
    // main.c original -- exige a ordem exata de declaracao e reclama de todo
    // campo omitido; atribuicao evita as duas armadilhas e sobrevive a
    // qualquer campo novo que uma versao futura do driver adicione.
    camera_config_t config = {};
    config.pin_pwdn = 32;
    config.pin_reset = -1;
    config.pin_xclk = 0;
    config.pin_sccb_sda = 26;
    config.pin_sccb_scl = 27;
    config.pin_d0 = 5;
    config.pin_d1 = 18;
    config.pin_d2 = 19;
    config.pin_d3 = 21;
    config.pin_d4 = 36;
    config.pin_d5 = 39;
    config.pin_d6 = 34;
    config.pin_d7 = 35;
    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_pclk = 22;
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QQVGA;  // 160 x 120, um byte por pixel.
    config.fb_count = 2;  // Permite capturar o proximo frame enquanto o atual e enviado.
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    return esp_camera_init(&config);
}

// Downsamples fb's centered square crop to OUTPUT_COLS x OUTPUT_ROWS and
// appends the ASCII black/white grid (each pixel '0'/'1' against
// g_threshold) to `out` -- the same framing this file's serial dump used to
// print directly, now built into a buffer so task() can publish it as the
// camera.matrix telemetry document instead.
bool build_binary_matrix(camera_fb_t* fb, std::string& out) {
    // Valida o formato e o tamanho antes de acessar o buffer.
    if (fb == nullptr || fb->buf == nullptr ||
        fb->format != PIXFORMAT_GRAYSCALE || fb->width == 0 ||
        fb->height == 0 || fb->height > fb->len / fb->width) {
        ESP_LOGE(kTag, "Framebuffer grayscale invalido ou incompleto");
        return false;
    }

    // Recorta o maior quadrado possivel bem no centro do frame.
    const size_t square = fb->width < fb->height ? fb->width : fb->height;
    const size_t x_offset = (fb->width - square) / 2;
    const size_t y_offset = (fb->height - square) / 2;
    const size_t step_x = square / OUTPUT_COLS;
    const size_t step_y = square / OUTPUT_ROWS;
    if (step_x == 0 || step_y == 0) {
        ESP_LOGE(kTag, "Frame (%ux%u) menor que a saida desejada (%dx%d)",
                 static_cast<unsigned int>(fb->width),
                 static_cast<unsigned int>(fb->height), OUTPUT_COLS,
                 OUTPUT_ROWS);
        return false;
    }

    const std::uint8_t threshold = g_threshold.load(std::memory_order_relaxed);

    out.clear();
    out.reserve((OUTPUT_COLS + 1U) * OUTPUT_ROWS + 32U);
    char header[32];
    const int header_len =
        std::snprintf(header, sizeof(header), "FRAME\n%d x %d\n", OUTPUT_COLS, OUTPUT_ROWS);
    if (header_len > 0) {
        out.append(header, static_cast<size_t>(header_len));
    }

    for (size_t row = 0; row < OUTPUT_ROWS; row++) {
        const size_t y = y_offset + row * step_y;
        for (size_t col = 0; col < OUTPUT_COLS; col++) {
            const size_t x = x_offset + col * step_x;
            const std::uint8_t pixel = fb->buf[y * fb->width + x];
            out.push_back(pixel >= threshold ? '1' : '0');
        }
        out.push_back('\n');
    }
    out += "---\n";
    return true;
}

}  // namespace

void task(void* arg) {
    auto* robot = static_cast<CamRobot*>(arg);

    puts("CAMERA: inicializando OV2640...");
    const esp_err_t err = camera_init();
    if (err != ESP_OK) {
        g_state.store(1U, std::memory_order_relaxed);
        // Mantem o diagnostico visivel se o monitor for aberto depois do boot.
        while (true) {
            ESP_LOGE(kTag,
                     "Falha ao inicializar a camera: %s (0x%x). Confira "
                     "camera/alimentacao e reinicie a placa.",
                     esp_err_to_name(err), static_cast<unsigned int>(err));
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    puts("CAMERA: pronta; iniciando captura");

    std::uint32_t frames_this_window = 0U;
    std::int64_t window_start_us = esp_timer_get_time();
    std::int64_t last_telemetry_us = window_start_us;
    std::int64_t next_matrix_us = window_start_us;
    std::string matrix;

    // Sem atraso artificial: atualiza na taxa maxima que camera e serial
    // permitirem -- comportamento inalterado em relacao ao main.c original.
    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb == nullptr) {
            ESP_LOGE(kTag, "Falha ao capturar frame");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        const std::int64_t now_us = esp_timer_get_time();

        // camera.matrix is a replace-in-place document (TelemetryPublisher's
        // own staging slot, like bally_OS's system.monitor), so it is only
        // worth downsampling+publishing when someone actually subscribed and
        // only at that subscription's own effective rate -- same gating
        // bally_OS's sampleTelemetry() applies to its own UTF8 topic.
        if (robot != nullptr) {
            const std::uint64_t period_us = robot->telemetry.topic_period_us(
                TelemetryPublisher::kCameraMatrixTopicId);
            if (period_us != 0U && now_us >= next_matrix_us) {
                next_matrix_us = now_us + static_cast<std::int64_t>(period_us);
                if (build_binary_matrix(fb, matrix)) {
                    robot->telemetry.publish_camera_matrix(
                        matrix.data(), matrix.size(),
                        static_cast<std::uint64_t>(now_us));
                }
            }
        }

        esp_camera_fb_return(fb);

        g_frame_count.fetch_add(1U, std::memory_order_relaxed);
        ++frames_this_window;

        if (now_us - window_start_us >= 1000000) {
            const std::uint16_t fps_x10 = static_cast<std::uint16_t>(
                frames_this_window * 10000000ULL /
                static_cast<std::uint64_t>(now_us - window_start_us));
            g_fps_x10.store(fps_x10, std::memory_order_relaxed);
            frames_this_window = 0U;
            window_start_us = now_us;
        }

        // camera.status is a low-rate status summary (see TelemetryPublisher's
        // schema comment), not a per-frame stream -- publish at most once a
        // second regardless of the capture rate above.
        if (robot != nullptr && now_us - last_telemetry_us >= 1000000) {
            last_telemetry_us = now_us;
            robot->telemetry.publish_camera_status(
                g_state.load(std::memory_order_relaxed),
                g_frame_count.load(std::memory_order_relaxed),
                g_fps_x10.load(std::memory_order_relaxed),
                static_cast<std::uint64_t>(now_us));
        }
    }
}

}  // namespace cam
