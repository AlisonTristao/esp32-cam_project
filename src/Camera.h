#ifndef CAMERA_H
#define CAMERA_H

#include <atomic>
#include <cstdint>

class CamRobot;

namespace cam {

// Runtime-adjustable via the "cam" shell module (CamShell.cpp) -- same
// BW_THRESHOLD the original main.c hardcoded, now a live atomic so it can be
// read/set over the shell (both BTP channels, or the local serial console).
// Ignored by build_matrix when g_adaptive_threshold is enabled.
extern std::atomic<std::uint8_t> g_threshold;

// When non-zero, build_matrix derives its black/white cut (depth=1) or its
// grayscale contrast (depth=2/4/8) from the current frame's own sampled
// min/max instead of a fixed calibration, so exposure/lighting swings do
// not need a manual re-tune each time:
//   - depth=1: ignores g_threshold and uses the midpoint of min/max instead.
//   - depth=2/4/8: linearly stretches [min, max] to [0, (1<<depth)-1]
//     instead of just truncating the sensor's top depth bits, so a
//     low-contrast frame still spans the full range of gray levels.
// Runtime-adjustable via "cam" (CamShell.cpp's cam_set_adaptive). Trades
// frame-to-frame brightness comparability for per-frame contrast -- the same
// physical brightness can map to a different level once the scene's own
// range shifts.
extern std::atomic<std::uint8_t> g_adaptive_threshold;

// Shared bounds (pixels) for the two square sizes build_matrix deals with:
// g_matrix_size (the grid it sends) and g_crop_size (the centered source
// region it samples that grid from, i.e. the "zoom" -- see its own comment
// below). kMinMatrixSize just keeps both above a degenerate sliver;
// kMaxMatrixSize is the side of the centered square Camera.cpp can crop out
// of a FRAMESIZE_QQVGA frame (160x120 -> 120x120) -- raise it only alongside
// a bigger camera_init() frame_size. For g_matrix_size it doubles as "one
// grid cell per native pixel, no downsampling" at the full crop, and (at
// g_bits_per_pixel's default of 1) keeps the bit-packed body (build_matrix's
// header+bitmap) comfortably under TelemetryPublisher::kMaxCameraMatrixBodySize:
// 120 rows x ceil(120/8) bytes/row = 1800 bytes. This is a hard ceiling on
// the grid regardless of depth; the achievable grid at higher
// g_bits_per_pixel is smaller still and enforced separately by
// fits_matrix_budget (e.g. 8 bits/pixel tops out around 120x120 -> wildly
// over budget, so cam_set_depth/cam_set_size clamp well below kMaxMatrixSize
// once depth grows).
inline constexpr std::uint8_t kMinMatrixSize = 4U;
inline constexpr std::uint8_t kMaxMatrixSize = 120U;

// Side (in pixels) of the square grid camera.matrix sends, runtime-
// adjustable via "cam" (CamShell.cpp's cam_set_size).
extern std::atomic<std::uint8_t> g_matrix_size;

// Side (in pixels) of the centered square build_matrix samples
// g_matrix_size's grid from -- shrinking it below kMaxMatrixSize is a
// digital zoom (the same grid resolution now covers less of the sensor);
// growing g_matrix_size past this instead upsamples (repeats source
// pixels). Defaults to kMaxMatrixSize, i.e. today's fixed behavior: the
// full centered square, no zoom. Runtime-adjustable via "cam"
// (CamShell.cpp's cam_set_crop).
extern std::atomic<std::uint8_t> g_crop_size;

// Sample depth build_matrix packs each grid cell to: 1 (today's default --
// g_threshold black/white bit), 2, 4 or 8 (grayscale: the sensor's top N
// bits, or a min/max contrast stretch when g_adaptive_threshold is on).
// Runtime-adjustable via "cam" (CamShell.cpp's cam_set_depth). Widening this
// grows the wire body roughly linearly (8x at bpp=8 vs bpp=1 for the same
// grid), so cam_set_depth/cam_set_size both reject a combination
// fits_matrix_budget rejects -- see that function.
extern std::atomic<std::uint8_t> g_bits_per_pixel;

// Last encoded frame's body sizes (including the 4-byte matrix header).
extern std::atomic<std::uint16_t> g_matrix_raw_bytes, g_matrix_encoded_bytes;
extern std::atomic<std::uint8_t> g_matrix_encoding;

// Whether `bits_per_pixel` is one of the depths build_matrix supports
// (1, 2, 4 or 8 -- each evenly divides a byte, so no sample ever straddles
// a byte boundary).
bool is_valid_bit_depth(std::uint8_t bits_per_pixel) noexcept;

// Whether a (grid_size, bits_per_pixel) combination's wire body --
// TelemetryPublisher's 4-byte {rows, cols, bits_per_pixel, encoding} header plus the
// packed grid -- fits TelemetryPublisher::kMaxCameraMatrixBodySize. Shared
// by cam_set_size and cam_set_depth (CamShell.cpp) so neither setter alone
// can put the other's already-stored value over budget.
bool fits_matrix_budget(std::uint8_t grid_size,
                         std::uint8_t bits_per_pixel) noexcept;

// Updated once per captured frame; read by "cam -status" and published as
// the camera.status telemetry sample.
extern std::atomic<std::uint32_t> g_frame_count;
extern std::atomic<std::uint16_t> g_fps_x10;  // frames/sec * 10
extern std::atomic<std::uint8_t> g_state;     // 0 = capturing, 1 = camera error

// FreeRTOS task entry point. `arg` is a CamRobot* -- used only to publish
// camera.status telemetry roughly once a second; the ASCII frame dump this
// firmware already produced keeps going to the serial port unchanged.
void task(void* arg);

}  // namespace cam

#endif  // CAMERA_H
