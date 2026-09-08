#include "Camera.h"
#include "CameraMatrix.h"

// Default grid side (pixels); runtime-adjustable up to kMaxMatrixSize via
// the "cam set_size" shell command (CamShell.cpp) -- unchanged from the grid
// this file always sent before it became configurable.
#define DEFAULT_MATRIX_SIZE 30
#define DEFAULT_BW_THRESHOLD 180
#define DEFAULT_BITS_PER_PIXEL 1

#include <cstdio>

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "CamRobot.h"
#include "TelemetryPublisher.h"

static_assert(DEFAULT_MATRIX_SIZE >= cam::kMinMatrixSize &&
                  DEFAULT_MATRIX_SIZE <= cam::kMaxMatrixSize,
              "DEFAULT_MATRIX_SIZE fora dos limites de cam::g_matrix_size");

namespace cam {

std::atomic<std::uint8_t> g_threshold{DEFAULT_BW_THRESHOLD};
std::atomic<std::uint8_t> g_matrix_size{DEFAULT_MATRIX_SIZE};
std::atomic<std::uint8_t> g_crop_size{kMaxMatrixSize};
std::atomic<std::uint8_t> g_bits_per_pixel{DEFAULT_BITS_PER_PIXEL};
std::atomic<std::uint8_t> g_delta{kDefaultDelta};
std::atomic<std::uint16_t> g_matrix_raw_bytes{0}, g_matrix_encoded_bytes{0};
std::atomic<std::uint8_t> g_matrix_encoding{0};
std::atomic<std::uint32_t> g_frame_count{0U};
std::atomic<std::uint16_t> g_fps_x10{0U};
std::atomic<std::uint8_t> g_state{0U};

namespace {

const char* const kTag = "binary_camera";

// Buffer capacity for the largest document build_matrix can ever legally
// produce -- fits_matrix_budget rejects any (grid_size, bits_per_pixel)
// combination whose total_size would exceed
// TelemetryPublisher::kMaxCameraMatrixBodySize before it ever reaches
// g_matrix_size/g_bits_per_pixel, so that constant is the true ceiling
// regardless of how the two trade off. task() allocates exactly one of
// these, reused every capture.
constexpr std::size_t kMatrixBufCapacity =
    TelemetryPublisher::kMaxCameraMatrixBodySize;

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

// Sample/quantize the centered crop, then filter and adaptively encode schema v2.
bool build_matrix(camera_fb_t* fb, std::uint8_t grid_size,
                   std::uint8_t bits_per_pixel, std::uint8_t* out,
                   std::size_t out_capacity, std::size_t* out_len) {
    // Valida o formato e o tamanho antes de acessar o buffer.
    if (fb == nullptr || fb->buf == nullptr ||
        fb->format != PIXFORMAT_GRAYSCALE || fb->width == 0 ||
        fb->height == 0 || fb->height > fb->len / fb->width) {
        ESP_LOGE(kTag, "Framebuffer grayscale invalido ou incompleto");
        return false;
    }
    if (out == nullptr || out_len == nullptr || grid_size < kMinMatrixSize ||
        grid_size > kMaxMatrixSize || !is_valid_bit_depth(bits_per_pixel)) {
        ESP_LOGE(kTag, "Parametros invalidos para build_matrix");
        return false;
    }

    // Recorta um quadrado de g_crop_size bem no centro do frame -- clamped
    // to the frame's own native square in case a future sensor mode makes
    // that smaller than kMaxMatrixSize (the shell already keeps g_crop_size
    // within [kMinMatrixSize, kMaxMatrixSize], so this only ever bites then).
    const size_t max_square = fb->width < fb->height ? fb->width : fb->height;
    size_t crop = g_crop_size.load(std::memory_order_relaxed);
    if (crop > max_square) {
        crop = max_square;
    }
    const size_t x_offset = (fb->width - crop) / 2;
    const size_t y_offset = (fb->height - crop) / 2;

    // Dedicated capture-task scratch in BSS; avoid ~29 KB on its small stack.
    static std::uint8_t samples[kMaxMatrixSize * kMaxMatrixSize];
    static std::uint8_t filtered[kMaxMatrixSize * kMaxMatrixSize];
    const auto threshold = g_threshold.load(std::memory_order_relaxed);
    const auto delta = g_delta.load(std::memory_order_relaxed);
    for (size_t row = 0; row < grid_size; row++) {
        const size_t y = y_offset + (row * crop) / grid_size;
        for (size_t col = 0; col < grid_size; col++) {
            const size_t x = x_offset + (col * crop) / grid_size;
            const std::uint8_t pixel = fb->buf[y * fb->width + x];
            const std::uint8_t sample =
                (bits_per_pixel == 1U)
                    ? (pixel >= threshold ? 1U : 0U)
                    : static_cast<std::uint8_t>(pixel >>
                                                 (8U - bits_per_pixel));
            samples[row * grid_size + col] = sample;
        }
    }

    if (!encode_matrix(samples, grid_size, grid_size, bits_per_pixel, delta,
                       filtered, sizeof(filtered), out, out_capacity, out_len)) return false;
    g_matrix_raw_bytes.store(kMatrixHeaderSize +
        ((std::size_t(grid_size) * bits_per_pixel + 7U) / 8U) * grid_size,
        std::memory_order_relaxed);
    g_matrix_encoded_bytes.store(static_cast<std::uint16_t>(*out_len), std::memory_order_relaxed);
    g_matrix_encoding.store(out[3], std::memory_order_relaxed);
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
    std::uint8_t matrix_buf[kMatrixBufCapacity];
    std::size_t matrix_len = 0U;

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
        // bally_OS's sampleTelemetry() applies to its own topic.
        if (robot != nullptr) {
            const std::uint64_t period_us = robot->telemetry.topic_period_us(
                TelemetryPublisher::kCameraMatrixTopicId);
            if (period_us != 0U && now_us >= next_matrix_us) {
                next_matrix_us = now_us + static_cast<std::int64_t>(period_us);
                const std::uint8_t grid_size =
                    g_matrix_size.load(std::memory_order_relaxed);
                const std::uint8_t bits_per_pixel =
                    g_bits_per_pixel.load(std::memory_order_relaxed);
                if (build_matrix(fb, grid_size, bits_per_pixel, matrix_buf,
                                 sizeof(matrix_buf), &matrix_len)) {
                    robot->telemetry.publish_camera_matrix(
                        reinterpret_cast<const char*>(matrix_buf), matrix_len,
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
