#include "Camera.h"

// Default grid side (pixels); runtime-adjustable up to kMaxMatrixSize via
// the "cam set_size" shell command (CamShell.cpp) -- unchanged from the grid
// this file always sent before it became configurable.
#define DEFAULT_MATRIX_SIZE 30
#define DEFAULT_BW_THRESHOLD 180
#define DEFAULT_BITS_PER_PIXEL 1

#include <cstdio>
#include <cstring>

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
std::atomic<std::uint32_t> g_frame_count{0U};
std::atomic<std::uint16_t> g_fps_x10{0U};
std::atomic<std::uint8_t> g_state{0U};

bool is_valid_bit_depth(std::uint8_t bits_per_pixel) noexcept {
    return bits_per_pixel == 1U || bits_per_pixel == 2U ||
           bits_per_pixel == 4U || bits_per_pixel == 8U;
}

// Header octets in front of the packed grid (see build_matrix): grid rows,
// grid cols, then bits_per_pixel. Kept separate from assuming square/fixed
// depth so a decoder never has to guess either.
constexpr std::size_t kMatrixHeaderSize = 3U;

bool fits_matrix_budget(std::uint8_t grid_size,
                         std::uint8_t bits_per_pixel) noexcept {
    if (grid_size < kMinMatrixSize || grid_size > kMaxMatrixSize ||
        !is_valid_bit_depth(bits_per_pixel)) {
        return false;
    }
    const std::size_t row_bits =
        static_cast<std::size_t>(grid_size) * bits_per_pixel;
    const std::size_t row_bytes = (row_bits + 7U) / 8U;
    const std::size_t total_size =
        kMatrixHeaderSize + row_bytes * static_cast<std::size_t>(grid_size);
    return total_size <= TelemetryPublisher::kMaxCameraMatrixBodySize;
}

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

// Samples a g_crop_size x g_crop_size square centered in fb (the "zoom" --
// smaller than the frame's own native square means digital zoom) down (or up,
// if grid_size > crop) to grid_size x grid_size, nearest-neighbor, and packs
// each cell to bits_per_pixel bits into `out` -- the wire body
// btp::TelemetryEncoding::OpaqueBytes carries as camera.matrix. Layout:
// out[0]=rows, out[1]=cols (both grid_size; kept separate so a decoder never
// has to assume square), out[2]=bits_per_pixel, then rows*row_bytes bytes of
// packed samples where row_bytes=ceil(cols*bits_per_pixel/8), one row after
// another, each row starting its own byte (so a row never spans a byte
// boundary). Within a row, samples pack MSB-first: the first (leftmost)
// column occupies the top bits_per_pixel bits of row_bytes[0], the next
// sample the next bits_per_pixel bits, and so on -- e.g. at
// bits_per_pixel=4, byte 0 holds {col 0 in bits 7-4, col 1 in bits 3-0}.
// bits_per_pixel==1 keeps today's threshold behavior (g_threshold decides
// each bit); bits_per_pixel in {2,4,8} instead keeps the top bits_per_pixel
// bits of the sensor's 8-bit sample verbatim (no threshold), i.e. grayscale.
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

    const size_t row_bits =
        static_cast<size_t>(grid_size) * bits_per_pixel;
    const size_t row_bytes = (row_bits + 7U) / 8U;
    const size_t body_size = row_bytes * grid_size;
    const size_t total_size = kMatrixHeaderSize + body_size;
    if (out_capacity < total_size) {
        ESP_LOGE(kTag, "Buffer da matriz pequeno demais (%u < %u)",
                 static_cast<unsigned int>(out_capacity),
                 static_cast<unsigned int>(total_size));
        return false;
    }

    const std::uint8_t threshold = g_threshold.load(std::memory_order_relaxed);
    // Every sample's bits fit within one byte (bits_per_pixel evenly divides
    // 8), so no sample-to-byte alignment math is needed beyond a shift.
    const unsigned samples_per_byte = 8U / bits_per_pixel;
    const std::uint8_t sample_mask =
        static_cast<std::uint8_t>((1U << bits_per_pixel) - 1U);

    out[0] = grid_size;
    out[1] = grid_size;
    out[2] = bits_per_pixel;
    std::uint8_t* body = out + kMatrixHeaderSize;
    std::memset(body, 0, body_size);

    for (size_t row = 0; row < grid_size; row++) {
        const size_t y = y_offset + (row * crop) / grid_size;
        std::uint8_t* row_bits = body + row * row_bytes;
        for (size_t col = 0; col < grid_size; col++) {
            const size_t x = x_offset + (col * crop) / grid_size;
            const std::uint8_t pixel = fb->buf[y * fb->width + x];
            const std::uint8_t sample =
                (bits_per_pixel == 1U)
                    ? (pixel >= threshold ? 1U : 0U)
                    : static_cast<std::uint8_t>(pixel >>
                                                 (8U - bits_per_pixel));
            const unsigned shift =
                (samples_per_byte - 1U - (col % samples_per_byte)) *
                bits_per_pixel;
            row_bits[col / samples_per_byte] |= static_cast<std::uint8_t>(
                (sample & sample_mask) << shift);
        }
    }

    *out_len = total_size;
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
