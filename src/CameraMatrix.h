#pragma once

#include <cstddef>
#include <cstdint>

namespace cam {
inline constexpr std::size_t kMatrixHeaderSize = 4U;
inline constexpr std::size_t kMatrixBodyCapacity = 1900U;

// Quantized row-major input, straight to output -- no filtering, so RAW/RLE
// both reconstruct the input exactly.
// v2 body: rows, cols, depth, encoding (0=RAW, 1=RLE), data.
// RAW rows are byte-aligned/MSB-first. RLE crosses row boundaries;
// depth<8: high depth bits=value, remaining bits=count-1;
// depth=8: value byte followed by count-1 byte.
bool encode_matrix(const std::uint8_t* samples, std::uint8_t rows,
                   std::uint8_t cols, std::uint8_t depth,
                   std::uint8_t* out, std::size_t capacity,
                   std::size_t* length) noexcept;
}
