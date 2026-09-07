#include "CamShell.h"

#include <cstdio>

#include <TinyShell.h>

#include "Camera.h"

namespace {

// TinyShell only auto-generates a generic "executado com sucesso"/status
// message through run_command_line() (see TinyShell.cpp) -- there is no
// per-command hook to inject custom text into that message. So diagnostic
// detail below goes to the local serial console via printf(), same as the
// original main.c's own logging; the remotely-visible, structured way to see
// this device's state (over either BTP channel) is the camera.status
// telemetry topic (CamRobot::bindProtocolTransport(), published from
// cam::task()), not this shell module. The command still runs and reports
// success/failure over COMMAND_RESULT/TERMINAL_OUT either way.

std::uint8_t cam_status() {
    printf("cam: state=%u frame_count=%lu fps=%.1f threshold=%u\n",
          static_cast<unsigned>(cam::g_state.load()),
          static_cast<unsigned long>(cam::g_frame_count.load()),
          cam::g_fps_x10.load() / 10.0, static_cast<unsigned>(cam::g_threshold.load()));
    return RESULT_OK;
}

std::uint8_t cam_threshold() {
    printf("cam: threshold=%u\n", static_cast<unsigned>(cam::g_threshold.load()));
    return RESULT_OK;
}

std::uint8_t cam_set_threshold(int value) {
    if (value < 0 || value > 255) return RESULT_ERROR;
    cam::g_threshold.store(static_cast<std::uint8_t>(value));
    printf("cam: threshold set to %d\n", value);
    return RESULT_OK;
}

}  // namespace

void register_cam_shell_commands(TinyShell& shell) {
    shell.create_module("cam", "Camera de visao (ESP32-CAM)");
    shell.add(cam_status, "status",
             "Mostra state/frame_count/fps/threshold atuais", "cam");
    shell.add(cam_threshold, "threshold", "Mostra o limiar preto/branco atual",
             "cam");
    shell.add(cam_set_threshold, "set_threshold",
             "Ajusta o limiar preto/branco (0-255)", "cam");
}
