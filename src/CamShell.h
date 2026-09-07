#ifndef CAM_SHELL_H
#define CAM_SHELL_H

class TinyShell;

// Registers the "cam" shell module (status/threshold/set_threshold), reachable
// from both BTP channels (COMMAND_REQUEST via the dongle, TERMINAL_IN via
// TraceView) and the local serial console alike -- it is the same TinyShell
// instance either way.
void register_cam_shell_commands(TinyShell& shell);

#endif  // CAM_SHELL_H
