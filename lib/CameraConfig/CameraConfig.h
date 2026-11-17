#ifndef CAMERA_CONFIG_H
#define CAMERA_CONFIG_H

#include <cstddef>
#include <cstdint>

#include "Camera.h"
#include "CameraMatrix.h"

// Only a reference is needed, so the SD driver stays out of this header --
// same reasoning as KeyStore.h (lib/KeyStore/KeyStore.h).
class SDCard;

// SD card root file holding the five "cam" shell parameters
// (threshold/matrix_size/crop_size/bits_per_pixel/adaptive_threshold), so
// they survive a reboot instead of always coming back at their DEFAULT_*
// value (see Camera.cpp). Same convention as KEY_STORE_FILE: relative to the
// SDCard mount point.
#define CAMERA_CONFIG_FILE "camera.cfg"

namespace cam {

/**
 * @brief Wire layout of CAMERA_CONFIG_FILE.
 *
 * Deliberately mirrors KeyStore's file (magic + format + fields + trailing
 * CRC32) rather than inventing a second convention for the one other file
 * this firmware puts on the card. Every integer is little-endian.
 *
 *   offset size field
 *   0      8    magic               "CAMCFG\0\0"
 *   8      1    format              0x03 (v1/v2 read too, see below)
 *   9      1    threshold
 *   10     1    matrix_size
 *   11     1    crop_size
 *   12     1    bits_per_pixel
 *   13     1    adaptive_threshold  0=off, 1=on (v3 only; v1/v2 files predate
 *                                   this field and always read as off)
 *   14     10   reserved            zeros
 *   24     4    crc32               CRC-32/ISO-HDLC over octets 0..23, LE
 */
inline constexpr std::size_t kConfigFileSize = 28U;

/**
 * @brief A snapshot of the five live "cam" atomics (Camera.h), or a file
 * image not yet applied to them.
 */
struct Config {
    std::uint8_t threshold = 0U;
    std::uint8_t matrix_size = 0U;
    std::uint8_t crop_size = 0U;
    std::uint8_t bits_per_pixel = 0U;
    std::uint8_t adaptive_threshold = 0U;
};

/**
 * @brief Which field rejected the file, or which live-value check failed.
 *
 * One enumerator per field, same reasoning as KeyStore::Error: a robot that
 * fails to load its config should say why, not silently keep whatever the
 * atomics already held.
 */
enum class ConfigError : std::uint8_t {
    Ok = 0,
    Size,    // wrong length: not exactly kConfigFileSize octets
    Magic,   // not "CAMCFG\0\0"
    Format,  // format octet is neither 0x01 nor 0x02
    Crc32,   // stored CRC does not cover the first 24 octets
    Depth,   // bits_per_pixel not one of is_valid_bit_depth's values
    Range,   // matrix_size or crop_size outside [kMinMatrixSize, kMaxMatrixSize]
    Budget,  // (matrix_size, bits_per_pixel) fails fits_matrix_budget
    File,    // card not mounted, or CAMERA_CONFIG_FILE unreadable/unwritable
};

/** @brief Name of the field a ConfigError refers to, for logging. */
const char* config_error_field(ConfigError error) noexcept;

/**
 * @brief Validate a complete file image and, optionally, extract it.
 *
 * Pure: no SD card, no ESP-IDF, no allocation. Fields are checked in the
 * order of the layout above, then the same range/depth/budget rules
 * cam_set_size/cam_set_depth (CamShell.cpp) already enforce, so a file this
 * function accepts can never push the live atomics somewhere the shell
 * itself would have refused to.
 */
ConfigError parse_config(const std::uint8_t* data, std::size_t size,
                         Config* out = nullptr) noexcept;

/**
 * @brief Serialize `in` to exactly kConfigFileSize octets, CRC included.
 *
 * The caller is responsible for `in` already passing the same checks
 * parse_config enforces on the way in -- snapshot_config() only ever reads
 * values the shell already validated, so this never re-checks them.
 */
void serialize_config(const Config& in, std::uint8_t* out) noexcept;

/** @brief Snapshot of the five live atomics. */
Config snapshot_config() noexcept;

/** @brief Same range/depth/budget rules parse_config enforces on a file. */
bool is_valid_config(const Config& config) noexcept;

/** @brief Applies an already-validated config to the five live atomics. */
void apply_config(const Config& config) noexcept;

/**
 * @brief Read CAMERA_CONFIG_FILE from the card, parse it and, on success,
 * apply it to the live atomics.
 *
 * The atomics are left untouched on any failure. Only available on the
 * ESP-IDF build; the host stub always returns ConfigError::File.
 */
ConfigError load_config_from_card(SDCard& card) noexcept;

/**
 * @brief Snapshot the live atomics and write CAMERA_CONFIG_FILE to the card.
 *
 * Only available on the ESP-IDF build; the host stub always returns
 * ConfigError::File.
 */
ConfigError save_config_to_card(SDCard& card) noexcept;

}  // namespace cam

#endif  // CAMERA_CONFIG_H
