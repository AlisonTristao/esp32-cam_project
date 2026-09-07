#include "CamShell.h"

#include <cstdarg>
#include <cstdio>
#include <functional>
#include <string>
#include <utility>

#include <TinyShell.h>

#include "Camera.h"

namespace {

// Set once by register_cam_shell_commands() to the exact sink main.cpp also
// hands to TinyShell::set_output_callback() (appends to g_shell_output).
// TinyShell only auto-generates a generic "executado com sucesso"/status
// message through run_command_line() (see TinyShell.cpp) -- there is no
// per-command hook to inject custom text into that message, so a plain
// printf() here (the original behavior) only ever reaches the local UART
// console and never TerminalResponder::deliver_command_output(), which
// mirrors g_shell_output back as TERMINAL_OUT. Routing through this sink
// too is what makes e.g. "cam -threshold" show its value in TraceView's
// Terminal instead of only "executado com sucesso".
std::function<void(const std::string&)> g_emit = [](const std::string&) {};

// Formats once: local console (printf's original destination, unchanged)
// and the remote-terminal sink above both get the same text.
void cam_print(const char* fmt, ...) {
    char buf[160];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    std::fputs(buf, stdout);
    g_emit(buf);
}

std::uint8_t cam_status() {
    cam_print("cam: state=%u frame_count=%lu fps=%.1f threshold=%u\n",
             static_cast<unsigned>(cam::g_state.load()),
             static_cast<unsigned long>(cam::g_frame_count.load()),
             cam::g_fps_x10.load() / 10.0, static_cast<unsigned>(cam::g_threshold.load()));
    return RESULT_OK;
}

std::uint8_t cam_threshold() {
    cam_print("cam: threshold=%u\n", static_cast<unsigned>(cam::g_threshold.load()));
    return RESULT_OK;
}

std::uint8_t cam_set_threshold(std::uint8_t value) {
    cam::g_threshold.store(value);
    cam_print("cam: threshold set to %u\n", static_cast<unsigned>(value));
    return RESULT_OK;
}

std::uint8_t cam_size() {
    const std::uint8_t size = cam::g_matrix_size.load();
    cam_print("cam: matrix size=%ux%u (min=%u max=%u)\n",
             static_cast<unsigned>(size), static_cast<unsigned>(size),
             static_cast<unsigned>(cam::kMinMatrixSize),
             static_cast<unsigned>(cam::kMaxMatrixSize));
    return RESULT_OK;
}

std::uint8_t cam_set_size(std::uint8_t value) {
    if (value < cam::kMinMatrixSize || value > cam::kMaxMatrixSize) {
        return RESULT_ERROR;
    }
    const std::uint8_t bits_per_pixel = cam::g_bits_per_pixel.load();
    if (!cam::fits_matrix_budget(value, bits_per_pixel)) {
        cam_print(
            "cam: %ux%u a %u bits/pixel estoura o payload; reduza o tamanho "
            "ou 'cam set_depth' primeiro\n",
            static_cast<unsigned>(value), static_cast<unsigned>(value),
            static_cast<unsigned>(bits_per_pixel));
        return RESULT_ERROR;
    }
    cam::g_matrix_size.store(value);
    cam_print("cam: matrix size set to %ux%u\n", static_cast<unsigned>(value),
             static_cast<unsigned>(value));
    return RESULT_OK;
}

std::uint8_t cam_depth() {
    const std::uint8_t bits_per_pixel = cam::g_bits_per_pixel.load();
    cam_print("cam: depth=%u bits/pixel (1=P&B via threshold; 2/4/8=cinza)\n",
             static_cast<unsigned>(bits_per_pixel));
    return RESULT_OK;
}

std::uint8_t cam_set_depth(std::uint8_t value) {
    if (!cam::is_valid_bit_depth(value)) {
        cam_print("cam: depth invalido (use 1, 2, 4 ou 8)\n");
        return RESULT_ERROR;
    }
    const std::uint8_t grid_size = cam::g_matrix_size.load();
    if (!cam::fits_matrix_budget(grid_size, value)) {
        cam_print(
            "cam: %u bits/pixel a %ux%u estoura o payload; reduza 'cam "
            "set_size' primeiro\n",
            static_cast<unsigned>(value), static_cast<unsigned>(grid_size),
            static_cast<unsigned>(grid_size));
        return RESULT_ERROR;
    }
    cam::g_bits_per_pixel.store(value);
    cam_print("cam: depth set to %u bits/pixel\n",
             static_cast<unsigned>(value));
    return RESULT_OK;
}

std::uint8_t cam_crop() {
    const std::uint8_t crop = cam::g_crop_size.load();
    cam_print("cam: crop=%ux%u (min=%u max=%u)\n", static_cast<unsigned>(crop),
             static_cast<unsigned>(crop), static_cast<unsigned>(cam::kMinMatrixSize),
             static_cast<unsigned>(cam::kMaxMatrixSize));
    return RESULT_OK;
}

std::uint8_t cam_set_crop(std::uint8_t value) {
    if (value < cam::kMinMatrixSize || value > cam::kMaxMatrixSize) {
        return RESULT_ERROR;
    }
    cam::g_crop_size.store(value);
    cam_print("cam: crop set to %ux%u\n", static_cast<unsigned>(value),
             static_cast<unsigned>(value));
    return RESULT_OK;
}

}  // namespace

void register_cam_shell_commands(TinyShell& shell,
                                 std::function<void(const std::string&)> emit) {
    g_emit = std::move(emit);

    shell.create_module("cam", "Camera de visao (ESP32-CAM)");
    shell.add(cam_status, "status",
             "Mostra state/frame_count/fps/threshold atuais", "cam");
    shell.add(cam_threshold, "threshold", "Mostra o limiar preto/branco atual",
             "cam");
    shell.add(cam_set_threshold, "set_threshold",
             "Ajusta o limiar preto/branco (0-255)", "cam");
    shell.add(cam_size, "size",
             "Mostra o tamanho atual da matriz (NxN) enviada", "cam");
    // Bounds mirror cam::kMinMatrixSize/kMaxMatrixSize (Camera.h) -- update
    // this description if those constants ever change. Effective max shrinks
    // below 120 once depth (cam set_depth) goes above 1 -- see
    // cam::fits_matrix_budget.
    shell.add(cam_set_size, "set_size",
             "Ajusta o tamanho da matriz (NxN) enviada (4-120, menos em "
             "profundidades >1 bit)",
             "cam");
    shell.add(cam_depth, "depth",
             "Mostra a profundidade atual (bits/pixel) da matriz enviada",
             "cam");
    shell.add(cam_set_depth, "set_depth",
             "Ajusta a profundidade da matriz (1=P&B, 2/4/8=escala de "
             "cinza); reduz o tamanho maximo permitido",
             "cam");
    shell.add(cam_crop, "crop",
             "Mostra o recorte central (NxN, em pixels) usado antes de "
             "reamostrar",
             "cam");
    // Same bounds as set_size -- shrinking this below max is a digital zoom;
    // update this description if kMinMatrixSize/kMaxMatrixSize ever change.
    shell.add(cam_set_crop, "set_crop",
             "Ajusta o recorte central (NxN, em pixels) usado antes de "
             "reamostrar (4-120); menor = mais zoom",
             "cam");
}
