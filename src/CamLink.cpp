#include "CamLink.h"

#include <RadioSeal.h>
#include <btp/messages.hpp>

#include "CamRobot.h"

bool CamLink::send(const std::uint8_t* frame, std::size_t frame_size) {
    return robot_.tx_scheduler.enqueue(frame, frame_size);
}

bally::Channel CamLink::classify_channel(
    const btp::Header& header, std::uint32_t dongle_source_id) noexcept {
    // Mirrors ROBOT::classifyChannel() verbatim: COMMAND, TERMINAL and
    // CONTROL/SUBSCRIBE|UNSUBSCRIBE are classified BY PEER; every other type
    // (in particular CONTROL/MANIFEST_REQUEST) is forced to C_Link -- only
    // the dongle's own aggregation cache legitimately asks for a manifest.
    const bool classify_by_peer =
        header.type == btp::MessageType::Command ||
        header.type == btp::MessageType::Terminal ||
        (header.type == btp::MessageType::Control &&
         (header.object_id == btp::object_id::kSubscribe ||
          header.object_id == btp::object_id::kUnsubscribe));
    return classify_by_peer
               ? bally::channel_of_peer(bally::Vantage::Robot,
                                        header.source_id, dongle_source_id)
               : bally::Channel::C_Link;
}

bool CamLink::seal(const btp::Header& header, std::uint16_t payload_size,
                   const std::uint8_t* plaintext, std::uint8_t* out) {
    // Only ever used for THIS node's own channel-C sends (node_'s own
    // MANIFEST_DATA / SUBSCRIBE_RESULT reply_seal() fallback, see the class
    // comment) -- Node::seal_thunk hands this the same canonical header (
    // kFlagEncrypted already set) BtpEndpoint's own seal callback would get,
    // so it is passed straight through: RadioSeal computes the AEAD
    // associated data from every header field, not just the flags.
    return RadioSeal::seal(nullptr, header, payload_size, plaintext, out);
}

bool CamLink::open(const btp::Header& header, std::uint16_t sealed_size,
                   const std::uint8_t* sealed, std::uint8_t* out_plaintext) {
    const bally::Channel channel =
        classify_channel(header, robot_.dongle_source_id());
    return channel == bally::Channel::B_Endpoint
               ? RadioSeal::open_e(header, sealed_size, sealed, out_plaintext)
               : RadioSeal::open(header, sealed_size, sealed, out_plaintext);
}

void CamLink::terminal(btp::Node& /*node*/, const btp::Header& header,
                      btp::ByteView payload, std::uint64_t /*now_ms*/) {
    robot_.terminal_responder.on_terminal_in(header, payload);
}

void CamLink::reply_seal(const btp::Header& request_header,
                         btp::EndpointSealFn* out_seal, void** out_seal_ctx) {
    // Only ever called for node_'s own automatic replies: MANIFEST_DATA
    // (classify_channel forces C_Link, so the branch below is skipped and
    // the default has_seal()/seal() -- key L -- runs unchanged) and
    // SUBSCRIBE_RESULT/UNSUBSCRIBE_RESULT (classified by peer, same as
    // COMMAND/TERMINAL): a TraceView-originated SUBSCRIBE needs its RESULT
    // sealed with key E, which the general seal() above never uses.
    const bally::Channel channel =
        classify_channel(request_header, robot_.dongle_source_id());
    if (channel == bally::Channel::B_Endpoint) {
        *out_seal = &RadioSeal::seal_e;
        *out_seal_ctx = nullptr;
        return;
    }
    *out_seal = nullptr;
    *out_seal_ctx = nullptr;
}
