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
    g_bits_per_pixel{1}, g_delta{kDefaultDelta};
}

struct Encoded {
    std::vector<std::uint8_t> body, filtered;
};

Encoded encode(const std::vector<std::uint8_t>& input, unsigned rows,
               unsigned cols, unsigned depth, unsigned delta) {
    Encoded result{{}, std::vector<std::uint8_t>(input.size())};
    // Guard bytes detect writes past either caller-owned buffer.
    result.body.resize(cam::kMatrixBodyCapacity + 1, 0xA5);
    result.filtered.push_back(0xA5);
    std::size_t length = 0;
    assert(cam::encode_matrix(input.data(), rows, cols, depth, delta,
                             result.filtered.data(), input.size(), result.body.data(),
                             cam::kMatrixBodyCapacity, &length));
    assert(result.body.back() == 0xA5 && result.filtered.back() == 0xA5);
    result.body.resize(length);
    result.filtered.pop_back();
    return result;
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
    auto uniform = encode(std::vector<std::uint8_t>(3600, 7), 60, 60, 4, 2);
    assert(uniform.body.size() == 229 && uniform.body[3] == 1);
    assert(std::all_of(uniform.body.begin() + 4, uniform.body.end(),
                       [](auto value) { return value == 0x7F; }));
    assert(decode(uniform.body) == uniform.filtered);

    for (unsigned depth : {1U, 2U, 4U, 8U}) {
        const unsigned side = depth == 1 ? 120 : depth == 2 ? 80 : depth == 4 ? 60 : 40;
        auto data = encode(std::vector<std::uint8_t>(side * side, (1U << depth) - 1),
                           side, side, depth, 0);
        assert(data.body[3] == 1 && decode(data.body) == data.filtered);
    }
    std::vector<std::uint8_t> image(81, 0);
    image[40] = 15;
    auto noise = encode(image, 9, 9, 4, 2);
    assert(std::all_of(noise.filtered.begin(), noise.filtered.end(), [](auto x) { return x == 0; }));

    std::fill(image.begin(), image.end(), 0);
    for (unsigned i = 0; i < 9; ++i) image[i * 9 + i] = 15;
    auto diagonal = encode(image, 9, 9, 4, 2);
    // The two endpoints lose support, but edits do not cascade down the line.
    for (unsigned i = 1; i < 8; ++i) assert(diagonal.filtered[i * 9 + i] == 15);
    assert(diagonal.filtered.front() == 0 && diagonal.filtered.back() == 0);

    // Wide vertical stripes: neighborhood support preserves original levels.
    image.resize(64);
    for (unsigned i = 0; i < 64; ++i) image[i] = 7 + (i % 8) / 2;
    auto gradient = encode(image, 8, 8, 4, 2);
    assert(gradient.filtered[3 * 8] == 7);
    assert(gradient.filtered[3 * 8 + 5] == 7);
    assert(gradient.filtered[3 * 8 + 6] == 10); // Fixed anchor, no drift to 7.
    auto exact = encode(image, 8, 8, 4, 0);
    assert(exact.filtered == image);

    // Alternating stable columns force RAW; odd widths must pad each row.
    image.resize(45);
    for (unsigned i = 0; i < 45; ++i) image[i] = (i % 9) % 2 ? 15 : 0;
    auto raw = encode(image, 5, 9, 4, 0);
    assert(raw.body[3] == 0 && raw.body.size() == 29);
    assert(decode(raw.body) == raw.filtered);
    for (unsigned row = 0; row < 5; ++row) assert((raw.body[4 + row * 5 + 4] & 15) == 0);

    // Same stripe pattern at depth=1: even delta=255 cannot merge black/white.
    for (auto& value : image) value = value ? 1 : 0;
    assert(encode(image, 5, 9, 1, 255).filtered == encode(image, 5, 9, 1, 0).filtered);

    // Tie (four 4-bit runs for eight stable pixels) selects RAW.
    image = {0, 0, 15, 15, 0, 0, 15, 15};
    auto tie = encode(image, 4, 2, 4, 0);
    assert(tie.body[3] == 0);

    std::mt19937 random(1234);
    for (unsigned depth : {1U, 2U, 4U, 8U}) {
        for (unsigned side = 4; side <= 120; ++side) {
            if (!cam::fits_matrix_budget(side, depth)) continue;
            image.resize(side * side);
            for (auto& value : image) value = random() & ((1U << depth) - 1);
            auto result = encode(image, side, side, depth, side % 4);
            assert(decode(result.body) == result.filtered);
            assert(result.body.size() <= 4 + ((side * depth + 7) / 8) * side);
        }
    }
    std::uint8_t scratch[64]{}, output[64]{};
    std::size_t length = 100;
    assert(!cam::encode_matrix(nullptr, 4, 4, 4, 2, scratch, 64, output, 64, &length));
    assert(length == 0);
    assert(!cam::encode_matrix(image.data(), 4, 4, 3, 2, scratch, 64, output, 64, &length));
    assert(!cam::encode_matrix(image.data(), 4, 4, 4, 2, scratch, 1, output, 64, &length));
    assert(!cam::encode_matrix(image.data(), 4, 4, 4, 2, scratch, 64, output, 1, &length));
    assert(cam::fits_matrix_budget(60, 4) && !cam::fits_matrix_budget(62, 4));
}

void config_cases() {
    cam::Config config{180, 60, 120, 4, 5}, restored{};
    std::uint8_t bytes[cam::kConfigFileSize]{};
    cam::serialize_config(config, bytes);
    assert(bytes[8] == 2);
    assert(cam::parse_config(bytes, sizeof(bytes), &restored) == cam::ConfigError::Ok);
    assert(restored.delta == 5);
    cam::apply_config(restored);
    assert(cam::snapshot_config().delta == 5);
    bytes[8] = 1; bytes[13] = 0; // Existing v1 files default to delta=2.
    const auto crc = btp::crc32(bytes, 24);
    for (unsigned i = 0; i < 4; ++i) bytes[24 + i] = crc >> (8 * i);
    assert(cam::parse_config(bytes, sizeof(bytes), &restored) == cam::ConfigError::Ok);
    assert(restored.delta == 2 && restored.matrix_size == 60 && restored.bits_per_pixel == 4);
    bytes[13] ^= 1;
    assert(cam::parse_config(bytes, sizeof(bytes), &restored) == cam::ConfigError::Crc32);
}

int main() {
    codec_cases(); config_cases();
    std::puts("camera matrix/config: all tests passed");
}
