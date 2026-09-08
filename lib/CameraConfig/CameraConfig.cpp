#include "CameraConfig.h"

#include <cstring>

// Same CRC routine and convention KeyStore.cpp already uses for
// KEY_STORE_FILE -- reused instead of adding a second CRC to the firmware.
#include <btp/codec.hpp>

// SDCard is an ESP-IDF driver, so it cannot be pulled into a host build.
// Only the card-touching functions at the bottom need it; parse_config/
// serialize_config stay pure. Same split as KeyStore.cpp.
#if defined(ESP_PLATFORM)
#include <SDCard.h>
#endif

namespace cam {

namespace {

constexpr std::size_t kOffMagic = 0U;
constexpr std::size_t kOffFormat = 8U;
constexpr std::size_t kOffThreshold = 9U;
constexpr std::size_t kOffMatrixSize = 10U;
constexpr std::size_t kOffCropSize = 11U;
constexpr std::size_t kOffBitsPerPixel = 12U;
constexpr std::size_t kOffDelta = 13U;
constexpr std::size_t kOffCrc = 24U;

constexpr std::uint8_t kMagic[8] = {'C', 'A', 'M', 'C', 'F', 'G', 0x00, 0x00};
constexpr std::uint8_t kFormatV1 = 0x01U;
constexpr std::uint8_t kFormatV2 = 0x02U;

std::uint32_t read_u32_le(const std::uint8_t* data) noexcept {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

void write_u32_le(std::uint8_t* data, std::uint32_t value) noexcept {
    data[0] = static_cast<std::uint8_t>(value);
    data[1] = static_cast<std::uint8_t>(value >> 8U);
    data[2] = static_cast<std::uint8_t>(value >> 16U);
    data[3] = static_cast<std::uint8_t>(value >> 24U);
}

}  // namespace

const char* config_error_field(ConfigError error) noexcept {
    switch (error) {
        case ConfigError::Ok:     return "ok";
        case ConfigError::Size:   return "size";
        case ConfigError::Magic:  return "magic";
        case ConfigError::Format: return "format";
        case ConfigError::Crc32:  return "crc32";
        case ConfigError::Depth:  return "depth";
        case ConfigError::Range:  return "range";
        case ConfigError::Budget: return "budget";
        case ConfigError::File:   return "file";
    }
    return "unknown";
}

bool is_valid_config(const Config& config) noexcept {
    if (!is_valid_bit_depth(config.bits_per_pixel)) return false;
    if (config.matrix_size < kMinMatrixSize || config.matrix_size > kMaxMatrixSize ||
        config.crop_size < kMinMatrixSize || config.crop_size > kMaxMatrixSize) {
        return false;
    }
    return fits_matrix_budget(config.matrix_size, config.bits_per_pixel);
}

ConfigError parse_config(const std::uint8_t* data, std::size_t size,
                         Config* out) noexcept {
    if (data == nullptr || size != kConfigFileSize) return ConfigError::Size;

    for (std::size_t i = 0U; i < sizeof(kMagic); ++i) {
        if (data[kOffMagic + i] != kMagic[i]) return ConfigError::Magic;
    }
    if (data[kOffFormat] != kFormatV1 && data[kOffFormat] != kFormatV2) return ConfigError::Format;

    // CRC before the fields: a corrupted file must be reported as corrupted,
    // not as an out-of-range value.
    if (btp::crc32(data, kOffCrc) != read_u32_le(data + kOffCrc)) {
        return ConfigError::Crc32;
    }

    Config parsed{};
    parsed.threshold = data[kOffThreshold];
    parsed.matrix_size = data[kOffMatrixSize];
    parsed.crop_size = data[kOffCropSize];
    parsed.bits_per_pixel = data[kOffBitsPerPixel];
    parsed.delta = data[kOffFormat] == kFormatV1 ? kDefaultDelta : data[kOffDelta];

    if (!is_valid_bit_depth(parsed.bits_per_pixel)) return ConfigError::Depth;
    if (parsed.matrix_size < kMinMatrixSize || parsed.matrix_size > kMaxMatrixSize ||
        parsed.crop_size < kMinMatrixSize || parsed.crop_size > kMaxMatrixSize) {
        return ConfigError::Range;
    }
    if (!fits_matrix_budget(parsed.matrix_size, parsed.bits_per_pixel)) {
        return ConfigError::Budget;
    }

    if (out != nullptr) *out = parsed;
    return ConfigError::Ok;
}

void serialize_config(const Config& in, std::uint8_t* out) noexcept {
    std::memset(out, 0, kConfigFileSize);
    std::memcpy(out + kOffMagic, kMagic, sizeof(kMagic));
    out[kOffFormat] = kFormatV2;
    out[kOffThreshold] = in.threshold;
    out[kOffMatrixSize] = in.matrix_size;
    out[kOffCropSize] = in.crop_size;
    out[kOffBitsPerPixel] = in.bits_per_pixel;
    out[kOffDelta] = in.delta;
    write_u32_le(out + kOffCrc, btp::crc32(out, kOffCrc));
}

Config snapshot_config() noexcept {
    Config config{};
    config.threshold = g_threshold.load(std::memory_order_relaxed);
    config.matrix_size = g_matrix_size.load(std::memory_order_relaxed);
    config.crop_size = g_crop_size.load(std::memory_order_relaxed);
    config.bits_per_pixel = g_bits_per_pixel.load(std::memory_order_relaxed);
    config.delta = g_delta.load(std::memory_order_relaxed);
    return config;
}

void apply_config(const Config& config) noexcept {
    g_threshold.store(config.threshold, std::memory_order_relaxed);
    g_matrix_size.store(config.matrix_size, std::memory_order_relaxed);
    g_crop_size.store(config.crop_size, std::memory_order_relaxed);
    g_bits_per_pixel.store(config.bits_per_pixel, std::memory_order_relaxed);
    g_delta.store(config.delta, std::memory_order_relaxed);
}

#if defined(ESP_PLATFORM)

ConfigError load_config_from_card(SDCard& card) noexcept {
    if (!card.is_mounted()) return ConfigError::File;

    // One octet more than the format: a longer file then reads as
    // kConfigFileSize+1 and is rejected by name ("size") instead of being
    // silently truncated into something that looks valid -- same trick
    // KeyStore::load_from_card uses.
    std::uint8_t buffer[kConfigFileSize + 1U] = {};
    std::size_t bytes_read = 0U;
    if (!card.read_file(CAMERA_CONFIG_FILE, buffer, sizeof(buffer),
                        &bytes_read)) {
        return ConfigError::File;
    }

    Config parsed{};
    const ConfigError error = parse_config(buffer, bytes_read, &parsed);
    if (error != ConfigError::Ok) return error;

    apply_config(parsed);
    return ConfigError::Ok;
}

ConfigError save_config_to_card(SDCard& card) noexcept {
    if (!card.is_mounted()) return ConfigError::File;

    const Config config = snapshot_config();
    // The live atomics can only ever hold values the shell's own setters
    // already validated, so this should never trip -- guarded anyway so a
    // future setter bug fails loudly here instead of writing a file
    // load_config_from_card would then refuse to apply.
    if (!is_valid_config(config)) return ConfigError::Budget;

    std::uint8_t buffer[kConfigFileSize] = {};
    serialize_config(config, buffer);
    return card.write_file(CAMERA_CONFIG_FILE, buffer, sizeof(buffer))
               ? ConfigError::Ok
               : ConfigError::File;
}

#else

// Host builds have no SD driver. Kept as a definition rather than omitted so
// that using it off-target is a clean runtime failure instead of a link
// error -- same reasoning as KeyStore::load_from_card's host stub.
ConfigError load_config_from_card(SDCard&) noexcept { return ConfigError::File; }
ConfigError save_config_to_card(SDCard&) noexcept { return ConfigError::File; }

#endif  // ESP_PLATFORM

}  // namespace cam
