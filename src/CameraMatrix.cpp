#include "CameraMatrix.h"
#include "Camera.h"

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

bool encode_matrix(const std::uint8_t* samples, std::uint8_t rows,
                   std::uint8_t cols, std::uint8_t depth,
                   std::uint8_t* out, std::size_t capacity,
                   std::size_t* length) noexcept {
    if (length) *length = 0;
    if (!samples || !out || !length ||
        rows == 0 || cols == 0 || rows > kMaxMatrixSize || cols > kMaxMatrixSize ||
        !is_valid_bit_depth(depth)) return false;
    const std::size_t count = std::size_t(rows) * cols;
    const std::size_t row_bytes = (std::size_t(cols) * depth + 7U) / 8U;
    const std::size_t raw_size = row_bytes * rows;
    if (capacity < kMatrixHeaderSize + raw_size ||
        kMatrixHeaderSize + raw_size > kMatrixBodyCapacity) return false;
    const unsigned max_sample = (1U << depth) - 1U;
    for (std::size_t i = 0; i < count; ++i)
        if (samples[i] > max_sample) return false;

    const unsigned count_bits = depth == 8U ? 8U : 8U - depth;
    const std::size_t max_run = 1U << count_bits;
    const std::size_t record_size = depth == 8U ? 2U : 1U;
    std::size_t rle_size = 0;
    for (std::size_t i = 0; i < count;) {
        std::size_t run = 1;
        while (i + run < count && run < max_run && samples[i + run] == samples[i]) ++run;
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
            while (i + run < count && run < max_run && samples[i + run] == samples[i]) ++run;
            if (depth == 8U) {
                body[written++] = samples[i];
                body[written++] = static_cast<std::uint8_t>(run - 1U);
            } else {
                body[written++] = static_cast<std::uint8_t>((samples[i] << count_bits) | (run - 1U));
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
                    samples[row * cols + col] << ((per_byte - 1U - col % per_byte) * depth);
        *length = kMatrixHeaderSize + raw_size;
    }
    return true;
}
}
