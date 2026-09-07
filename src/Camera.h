#ifndef CAMERA_H
#define CAMERA_H

#include <atomic>
#include <cstdint>

class CamRobot;

namespace cam {

// Runtime-adjustable via the "cam" shell module (CamShell.cpp) -- same
// BW_THRESHOLD the original main.c hardcoded, now a live atomic so it can be
// read/set over the shell (both BTP channels, or the local serial console).
extern std::atomic<std::uint8_t> g_threshold;

// Updated once per captured frame; read by "cam -status" and published as
// the camera.status telemetry sample.
extern std::atomic<std::uint32_t> g_frame_count;
extern std::atomic<std::uint16_t> g_fps_x10;  // frames/sec * 10
extern std::atomic<std::uint8_t> g_state;     // 0 = capturing, 1 = camera error

// FreeRTOS task entry point. `arg` is a CamRobot* -- used only to publish
// camera.status telemetry roughly once a second; the ASCII frame dump this
// firmware already produced keeps going to the serial port unchanged.
void task(void* arg);

}  // namespace cam

#endif  // CAMERA_H
