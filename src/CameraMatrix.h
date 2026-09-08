#pragma once

#include <cstddef>
#include <cstdint>

namespace cam {
inline constexpr std::size_t kMatrixHeaderSize = 4U;
inline constexpr std::size_t kMatrixBodyCapacity = 1900U;
inline constexpr std::uint8_t kDefaultDelta = 2U;
inline constexpr unsigned kMinPixels = 3U; // Includes the center pixel.

// Quantized row-major input, distinct scratch (rows*cols bytes), and output.
// v2 body: rows, cols, depth, encoding (0=RAW, 1=RLE), data.
// RAW rows are byte-aligned/MSB-first. RLE crosses row boundaries;
// depth<8: high depth bits=value, remaining bits=count-1;
// depth=8: value byte followed by count-1 byte.
// Both encodings reconstruct the same filtered/tolerance-grouped image.
bool encode_matrix(const std::uint8_t* samples, std::uint8_t rows,
                   std::uint8_t cols, std::uint8_t depth, std::uint8_t delta,
                   std::uint8_t* scratch, std::size_t scratch_capacity,
                   std::uint8_t* out, std::size_t capacity,
                   std::size_t* length) noexcept;
}
