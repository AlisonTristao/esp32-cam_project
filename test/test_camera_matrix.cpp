#include "Camera.h"
#include "CameraMatrix.h"
#include "CameraConfig.h"
#include <btp/codec.hpp>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <random>
#include <vector>

// Host definitions: Camera.cpp itself depends on the physical sensor/RTOS.
namespace cam {
std::atomic<std::uint8_t> g_threshold{180}, g_matrix_size{30}, g_crop_size{120},
    g_bits_per_pixel{1}, g_adaptive_threshold{0};
}

std::vector<std::uint8_t> encode(const std::vector<std::uint8_t>& input, unsigned rows,
                                 unsigned cols, unsigned depth) {
    // Guard byte detects writes past the caller-owned buffer.
    std::vector<std::uint8_t> body(cam::kMatrixBodyCapacity + 1, 0xA5);
    std::size_t length = 0;
    assert(cam::encode_matrix(input.data(), rows, cols, depth, body.data(),
                             cam::kMatrixBodyCapacity, &length));
    assert(body.back() == 0xA5);
    body.resize(length);
    return body;
}

// Independent wire reader verifies packing, run lengths and row padding.
std::vector<std::uint8_t> decode(const std::vector<std::uint8_t>& body) {
    std::vector<std::uint8_t> pixels;
    const unsigned rows = body[0], cols = body[1], depth = body[2];
    if (body[3] == 0) {
        const unsigned stride = (cols * depth + 7) / 8;
        for (unsigned r = 0; r < rows; ++r)
            for (unsigned c = 0; c < cols; ++c)
                pixels.push_back((body[4 + r * stride + c * depth / 8] >>
                                  (8 - depth - c * depth % 8)) & ((1U << depth) - 1));
    } else {
        const unsigned bits = depth == 8 ? 8 : 8 - depth;
        for (std::size_t i = 4; i < body.size();) {
            const unsigned value = depth == 8 ? body[i] : body[i] >> bits;
            const unsigned run = depth == 8 ? body.at(i + 1) + 1U
                                            : (body[i] & ((1U << bits) - 1)) + 1U;
            pixels.insert(pixels.end(), run, value);
            i += depth == 8 ? 2 : 1;
        }
    }
    assert(pixels.size() == rows * cols);
    return pixels;
}

void codec_cases() {
    auto uniform = encode(std::vector<std::uint8_t>(3600, 7), 60, 60, 4);
    assert(uniform.size() == 229 && uniform[3] == 1);
    assert(std::all_of(uniform.begin() + 4, uniform.end(),
                       [](auto value) { return value == 0x7F; }));
    assert(decode(uniform) == std::vector<std::uint8_t>(3600, 7));

    for (unsigned depth : {1U, 2U, 4U, 8U}) {
        const unsigned side = depth == 1 ? 120 : depth == 2 ? 80 : depth == 4 ? 60 : 40;
        std::vector<std::uint8_t> image(side * side, (1U << depth) - 1);
        auto data = encode(image, side, side, depth);
        assert(data[3] == 1 && decode(data) == image);
    }

    // Alternating stable columns force RAW; odd widths must pad each row.
    std::vector<std::uint8_t> image(45);
    for (unsigned i = 0; i < 45; ++i) image[i] = (i % 9) % 2 ? 15 : 0;
    auto raw = encode(image, 5, 9, 4);
    assert(raw[3] == 0 && raw.size() == 29);
    assert(decode(raw) == image);
    for (unsigned row = 0; row < 5; ++row) assert((raw[4 + row * 5 + 4] & 15) == 0);

    // Tie (four 4-bit runs for eight stable pixels) selects RAW.
    image = {0, 0, 15, 15, 0, 0, 15, 15};
    auto tie = encode(image, 4, 2, 4);
    assert(tie[3] == 0);

    std::mt19937 random(1234);
    for (unsigned depth : {1U, 2U, 4U, 8U}) {
        for (unsigned side = 4; side <= 120; ++side) {
            if (!cam::fits_matrix_budget(side, depth)) continue;
            image.resize(side * side);
            for (auto& value : image) value = random() & ((1U << depth) - 1);
            auto result = encode(image, side, side, depth);
            assert(decode(result) == image);
            assert(result.size() <= 4 + ((side * depth + 7) / 8) * side);
        }
    }
    std::uint8_t output[64]{};
    std::size_t length = 100;
    assert(!cam::encode_matrix(nullptr, 4, 4, 4, output, 64, &length));
    assert(length == 0);
    assert(!cam::encode_matrix(image.data(), 4, 4, 3, output, 64, &length));
    assert(!cam::encode_matrix(image.data(), 4, 4, 4, output, 1, &length));
    assert(cam::fits_matrix_budget(60, 4) && !cam::fits_matrix_budget(62, 4));
}

void config_cases() {
    cam::Config config{180, 60, 120, 4, 1}, restored{};
    std::uint8_t bytes[cam::kConfigFileSize]{};
    cam::serialize_config(config, bytes);
    assert(bytes[8] == 3);
    assert(cam::parse_config(bytes, sizeof(bytes), &restored) == cam::ConfigError::Ok);
    assert(restored.adaptive_threshold == 1);
    cam::apply_config(restored);
    assert(cam::snapshot_config().adaptive_threshold == 1);

    // v1/v2 files predate adaptive_threshold: whatever byte 13 used to mean
    // (nothing, then the old quantization "delta") is ignored, always off.
    bytes[8] = 1; bytes[13] = 0;
    auto crc = btp::crc32(bytes, 24);
    for (unsigned i = 0; i < 4; ++i) bytes[24 + i] = crc >> (8 * i);
    assert(cam::parse_config(bytes, sizeof(bytes), &restored) == cam::ConfigError::Ok);
    assert(restored.adaptive_threshold == 0 && restored.matrix_size == 60 &&
           restored.bits_per_pixel == 4);

    bytes[8] = 2; bytes[13] = 7;
    crc = btp::crc32(bytes, 24);
    for (unsigned i = 0; i < 4; ++i) bytes[24 + i] = crc >> (8 * i);
    assert(cam::parse_config(bytes, sizeof(bytes), &restored) == cam::ConfigError::Ok);
    assert(restored.adaptive_threshold == 0);

    bytes[13] ^= 1;
    assert(cam::parse_config(bytes, sizeof(bytes), &restored) == cam::ConfigError::Crc32);
}

int main() {
    codec_cases(); config_cases();
    std::puts("camera matrix/config: all tests passed");
}
