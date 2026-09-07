#ifndef CAM_SHELL_H
#define CAM_SHELL_H

#include <functional>
#include <string>

class TinyShell;

// Registers the "cam" shell module (status/threshold/set_threshold), reachable
// from both BTP channels (COMMAND_REQUEST via the dongle, TERMINAL_IN via
// TraceView) and the local serial console alike -- it is the same TinyShell
// instance either way.
//
// `emit` receives every cam_* command's diagnostic line (the same text
// printf() sends to the local console) -- pass the exact sink main.cpp also
// hands to TinyShell::set_output_callback() (appends to g_shell_output) so
// this text rides along with the generic pass/fail line
// TerminalResponder::deliver_command_output() mirrors back over TERMINAL_OUT.
// Without this, printf() output never reaches a remote terminal at all: only
// the generic status line does.
void register_cam_shell_commands(TinyShell& shell,
                                 std::function<void(const std::string&)> emit);

#endif  // CAM_SHELL_H
