#ifndef CAM_LINK_H
#define CAM_LINK_H

#include <cstdint>

#include <bally_channels.h>
#include <btp/node.hpp>

class CamRobot;

/**
 * @brief This device's btp::NodeConfig -- mirrors bally_OS's RobotLink
 * (utils/BallyRobot/BallyRobot.h) exactly, minus everything that depends on
 * catalog/telemetry complexity this firmware does not have. Held BY
 * REFERENCE by CamRobot's node_, so it must outlive it (CamRobot declares
 * protocol_link_ before node_ for that reason).
 *
 * send() enqueues through the same TxScheduler every other producer already
 * uses. seal()/open() classify the frame's channel (B_Endpoint vs C_Link)
 * exactly the way ROBOT::classifyChannel()/protocolOpen() do, then delegate
 * to RadioSeal -- the only place that touches btp::aead. terminal() forwards
 * to TerminalResponder; reply_seal() picks the key for node_'s own automatic
 * SUBSCRIBE_RESULT/UNSUBSCRIBE_RESULT the same way, by peer.
 */
class CamLink : public btp::NodeConfig {
public:
    explicit CamLink(CamRobot& robot) noexcept : robot_(robot) {}

    bool send(const std::uint8_t* frame, std::size_t frame_size) override;

    bool has_seal() const noexcept override { return true; }
    bool seal(const btp::Header& header, std::uint16_t payload_size,
             const std::uint8_t* plaintext, std::uint8_t* out) override;

    bool has_open() const noexcept override { return true; }
    bool open(const btp::Header& header, std::uint16_t sealed_size,
             const std::uint8_t* sealed, std::uint8_t* out_plaintext) override;

    bool has_terminal() const noexcept override { return true; }
    void terminal(btp::Node& node, const btp::Header& header,
                 btp::ByteView payload, std::uint64_t now_ms) override;

    void reply_seal(const btp::Header& request_header,
                    btp::EndpointSealFn* out_seal, void** out_seal_ctx) override;

    // Shared with CamRobot's own COMMAND_REQUEST/RESULT dispatch (which does
    // not go through open()/seal() above -- CommandProcessor::intake()/
    // send_result() classify by peer themselves), so the rule lives in
    // exactly one place. Mirrors ROBOT::classifyChannel() verbatim.
    static bally::Channel classify_channel(const btp::Header& header,
                                           std::uint32_t dongle_source_id) noexcept;

private:
    CamRobot& robot_;
};

#endif  // CAM_LINK_H
