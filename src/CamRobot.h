#ifndef CAM_ROBOT_H
#define CAM_ROBOT_H

#include <cstdint>
#include <optional>
#include <string>

#include <TinyShell.h>

#include <BtpTransport.h>
#include <CommandProcessor.h>
#include <KeyStore.h>
#include <SDCard.h>
#include <StatusReporter.h>
#include <TelemetryPublisher.h>
#include <TerminalResponder.h>
#include <TxScheduler.h>
#include <btp/node.hpp>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "CamLink.h"

// One command handed to the shell task, regardless of which BTP channel it
// arrived on. Mirrors bally_OS's QueuedCommand (BallyRobot.h): the shell task
// only needs to know how to route the result back once run_command_line()
// returns.
struct QueuedCommand {
    bool from_terminal = false;
    // COMMAND_REQUEST path (from_terminal == false):
    std::uint8_t cache_slot = 0U;
    // TERMINAL_IN path (from_terminal == true):
    std::uint32_t terminal_source_id = 0U;
    std::uint32_t terminal_boot_id = 0U;
    char text[btp_command::kMaxShellCommandSize + 1U] = {};
};

/**
 * @brief Composition root for this device's BTP/shell stack -- the ESP32-CAM
 * equivalent of bally_OS's ROBOT (utils/BallyRobot), minus every
 * motor/encoder/IMU-specific piece. Owns every producer/consumer that needs
 * a BtpSealFn (CommandProcessor, TerminalResponder, StatusReporter,
 * TelemetryPublisher), the shared BtpEndpoint they all send through, and the
 * btp::Node that answers MANIFEST_REQUEST/SUBSCRIBE/UNSUBSCRIBE for the
 * single camera.status topic.
 */
class CamRobot {
public:
    static constexpr std::uint64_t kNodeReassemblyTimeoutMs = 4000U;

    // Identity, keys and every BtpSealFn-consuming module's configure() call
    // -- mirrors ROBOT::configureProtocolIdentity(). Safe to call before the
    // SD card mounts: RadioSeal reads the KeyStore's live state at call time,
    // not a snapshot (see RadioSeal::configure's own comment).
    bool configureIdentity() noexcept;

    // Emplaces node_, declares the camera.status catalog topic and binds
    // every producer to the shared BtpEndpoint -- mirrors
    // ROBOT::bindProtocolTransport(). Call after tx_scheduler.configure()
    // and protocol.set_send_callback() (main.cpp's setup order).
    void bindProtocolTransport() noexcept;

    void set_shell_queue(QueueHandle_t queue) noexcept { shell_queue_ = queue; }

    // Handles one message node_->receive() decoded but did not answer
    // itself -- in practice this is only ever a fresh COMMAND_REQUEST
    // (has_command() stays false on CamLink, same as RobotLink: the async
    // shell-task model CommandProcessor already has does not fit Node's
    // synchronous NodeActionFn). Runs on the Wi-Fi RX callback context --
    // CommandProcessor::intake() is O(1) and never blocks, so this is safe
    // there, same as bally_OS's handleReceiveStatic. Returns true when the
    // message was this dispatcher's business (whether or not a WorkItem was
    // queued), false when the caller should route it another way (nothing
    // else reaches this today, since Terminal/Control are already handled by
    // node_->receive() itself).
    bool handleCompleteMessage(const btp::Header& header, btp::ByteView payload,
                               std::uint64_t now_us) noexcept;

    // Public members every task/callback in main.cpp reads directly, same
    // shape as ROBOT's own public members (BallyRobot.h).
    TinyShell shell;
    CommandProcessor command_processor;
    TerminalResponder terminal_responder;
    StatusReporter status_reporter;
    TelemetryPublisher telemetry;
    TxScheduler tx_scheduler;
    KeyStore key_store;
    SDCard sd_card;
    BtpEndpoint protocol;

    std::uint32_t source_id() const noexcept { return source_id_; }
    std::uint32_t boot_id() const noexcept { return boot_id_; }
    std::uint32_t dongle_source_id() const noexcept { return dongle_source_id_; }

    btp::Node* node() noexcept { return node_.has_value() ? &*node_ : nullptr; }

private:
    friend class CamLink;

    static bool submitTerminalCommandStatic(void* context, std::uint32_t source_id,
                                            std::uint32_t boot_id,
                                            const char* command_line) noexcept;

    std::uint32_t source_id_ = 0U;
    std::uint32_t boot_id_ = 0U;
    std::uint32_t dongle_source_id_ = 0U;

    QueueHandle_t shell_queue_ = nullptr;

    // protocol_link_ before node_ on purpose: node_ holds it by reference
    // (btp::NodeConfig is abstract, held by reference -- see btp/node.hpp),
    // so it must already be constructed.
    CamLink protocol_link_{*this};
    std::optional<btp::SizedNode<btp::NodeSize::Low>> node_;
};

#endif  // CAM_ROBOT_H
