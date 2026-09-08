#include "CameraMatrix.h"
#include "Camera.h"

#include <algorithm>
#include <cstring>

namespace cam {
bool is_valid_bit_depth(std::uint8_t depth) noexcept {
    return depth == 1U || depth == 2U || depth == 4U || depth == 8U;
}

bool fits_matrix_budget(std::uint8_t size, std::uint8_t depth) noexcept {
    return size >= kMinMatrixSize && size <= kMaxMatrixSize &&
           is_valid_bit_depth(depth) &&
           kMatrixHeaderSize + ((std::size_t(size) * depth + 7U) / 8U) * size
               <= kMatrixBodyCapacity;
}

namespace {
bool compatible(unsigned a, unsigned b, unsigned delta) noexcept {
    return (a > b ? a - b : b - a) <= delta;
}
}

bool encode_matrix(const std::uint8_t* samples, std::uint8_t rows,
                   std::uint8_t cols, std::uint8_t depth, std::uint8_t delta,
                   std::uint8_t* scratch, std::size_t scratch_capacity,
                   std::uint8_t* out, std::size_t capacity,
                   std::size_t* length) noexcept {
    if (length) *length = 0;
    if (!samples || !scratch || !out || !length || samples == scratch ||
        rows == 0 || cols == 0 || rows > kMaxMatrixSize || cols > kMaxMatrixSize ||
        !is_valid_bit_depth(depth)) return false;
    const std::size_t count = std::size_t(rows) * cols;
    const std::size_t row_bytes = (std::size_t(cols) * depth + 7U) / 8U;
    const std::size_t raw_size = row_bytes * rows;
    if (scratch_capacity < count || capacity < kMatrixHeaderSize + raw_size ||
        kMatrixHeaderSize + raw_size > kMatrixBodyCapacity) return false;
    const unsigned max_sample = (1U << depth) - 1U;
    for (std::size_t i = 0; i < count; ++i)
        if (samples[i] > max_sample) return false;
    const unsigned tolerance = depth == 1U ? 0U : std::min<unsigned>(delta, max_sample);

    // Read only the original grid: no scan-order-dependent cascading edits.
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const std::size_t index = std::size_t(row) * cols + col;
            std::uint8_t neighbors[8];
            unsigned n = 0, support = 1;
            for (int y = std::max(0, row - 1); y <= std::min<int>(rows - 1, row + 1); ++y) {
                for (int x = std::max(0, col - 1); x <= std::min<int>(cols - 1, col + 1); ++x) {
                    if (y == row && x == col) continue;
                    const auto value = samples[std::size_t(y) * cols + x];
                    neighbors[n++] = value;
                    if (compatible(value, samples[index], tolerance)) ++support;
                }
            }
            scratch[index] = samples[index];
            if (support < kMinPixels && n > 0) {
                // At most eight values: bounded insertion sort for the median.
                for (unsigned i = 1; i < n; ++i) {
                    const auto value = neighbors[i];
                    unsigned j = i;
                    while (j > 0 && neighbors[j - 1] > value) {
                        neighbors[j] = neighbors[j - 1];
                        --j;
                    }
                    neighbors[j] = value;
                }
                scratch[index] = neighbors[n / 2]; // Upper median, including at borders.
            }
        }
    }

    // Fix the reference for the entire region, even when its wire run splits.
    auto reference = scratch[0];
    for (std::size_t i = 1; i < count; ++i) {
        if (!compatible(scratch[i], reference, tolerance)) reference = scratch[i];
        scratch[i] = reference;
    }
    const unsigned count_bits = depth == 8U ? 8U : 8U - depth;
    const std::size_t max_run = 1U << count_bits;
    const std::size_t record_size = depth == 8U ? 2U : 1U;
    std::size_t rle_size = 0;
    for (std::size_t i = 0; i < count;) {
        std::size_t run = 1;
        while (i + run < count && run < max_run && scratch[i + run] == scratch[i]) ++run;
        rle_size += record_size;
        i += run;
    }

    out[0] = rows; out[1] = cols; out[2] = depth;
    out[3] = rle_size < raw_size ? 1U : 0U;
    auto* body = out + kMatrixHeaderSize;
    if (out[3] == 1U) {
        std::size_t written = 0;
        for (std::size_t i = 0; i < count;) {
            std::size_t run = 1;
            while (i + run < count && run < max_run && scratch[i + run] == scratch[i]) ++run;
            if (depth == 8U) {
                body[written++] = scratch[i];
                body[written++] = static_cast<std::uint8_t>(run - 1U);
            } else {
                body[written++] = static_cast<std::uint8_t>((scratch[i] << count_bits) | (run - 1U));
            }
            i += run;
        }
        *length = kMatrixHeaderSize + written;
    } else {
        std::memset(body, 0, raw_size);
        const unsigned per_byte = 8U / depth;
        for (std::size_t row = 0; row < rows; ++row)
            for (std::size_t col = 0; col < cols; ++col)
                body[row * row_bytes + col / per_byte] |=
                    scratch[row * cols + col] << ((per_byte - 1U - col % per_byte) * depth);
        *length = kMatrixHeaderSize + raw_size;
    }
    return true;
}
}
