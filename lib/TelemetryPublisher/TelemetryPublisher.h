#ifndef TELEMETRY_PUBLISHER_H
#define TELEMETRY_PUBLISHER_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>
#include <btp/subscription.hpp>  // btp::SubscriptionTable
#include <btp/telemetry.hpp>  // btp::WireType, btp::FieldSpec, btp::SampleWriter

// Needed for BtpSealFn (a file-scope type alias, not a member of BtpEndpoint),
// which configure() below names -- same reason StatusReporter.h already
// includes this.
#include <BtpTransport.h>

class BtpEndpoint;

// Trimmed from bally_OS's lib/TelemetryPublisher (which carries protocol.test,
// robot.state and system.monitor) down to the single topic this firmware
// needs: camera.status. Same architecture (SPSC sample queue, subscription
// binding through node_'s own btp::SubscriptionTable, per-topic byte/drop
// counters for STATUS section 5.1) -- only the schema table shrank.
class TelemetryPublisher {
public:
    static constexpr std::uint16_t kCameraStatusTopicId = 0x0201U;

    // Reserved subscriber identity for a subscription created ON this device
    // (a future "telemetry -sub" shell command) rather than by a SUBSCRIBE
    // arriving over the radio. See bally_OS's TelemetryPublisher.h for the
    // full reasoning: source_id derives from a factory MAC and is never zero,
    // so zero is safe to reserve here.
    static constexpr std::uint32_t kLocalSubscriberSourceId = 0U;
    static constexpr std::uint32_t kLocalSubscriberBootId = 0U;
    static constexpr std::uint16_t kSchemaVersion = 1U;
    static constexpr std::size_t kQueueCapacity = 8U;
    // schema_version(2) + state:u8(1) + frame_count:u32(4) + fps_x10:u16(2),
    // same non-nullable fixed-width layout btp::SampleWriter uses for
    // bally_OS's robot.state/protocol.test (see its TelemetryPublisher.cpp).
    static constexpr std::size_t kCameraStatusPayloadSize = 9U;

    enum class Encoding : std::uint8_t { Utf8 = 0x02U, PackedLe = 0x05U };

    // The field's wire type is btp::WireType (telemetry.md section 13's octet
    // values). The subset btp::SampleWriter/btp::SampleReader consume.
    struct FieldSchema {
        std::uint16_t field_id;
        std::uint16_t order;
        const char* name;
        btp::WireType type;
        const char* unit;
        float scale;
        float offset;
        std::uint16_t element_count;
        bool nullable;

        btp::FieldSpec spec() const noexcept {
            btp::FieldSpec s{};
            s.field_id = field_id;
            s.order = order;
            s.type = static_cast<std::uint8_t>(type);
            s.flags = nullable ? btp::kFieldNullable : std::uint8_t{0};
            s.element_count = element_count;
            s.max_element_count = 0U;
            s.scale = static_cast<double>(scale);
            s.offset = static_cast<double>(offset);
            return s;
        }
    };

    struct TopicSchema {
        std::uint16_t topic_id;
        std::uint16_t schema_version;
        const char* name;
        Encoding encoding;
        const FieldSchema* fields;
        std::size_t field_count;
        std::size_t payload_size;
        std::uint32_t max_rate_millihz;
        std::uint32_t min_rate_millihz;
        std::uint32_t default_rate_millihz;
    };

    enum class PublishResult : std::uint8_t {
        Queued,
        NotConfigured,
        QueueFull,
        InvalidValue,
        SequenceUnavailable,
    };

    struct Stats {
        std::uint32_t queued;
        std::uint32_t sent;
        std::uint32_t dropped_full;
        std::uint32_t dropped_invalid;
        std::uint32_t send_failed;
    };

    struct TopicStats {
        std::uint16_t topic_id;
        std::uint16_t subscriber_count;
        std::uint32_t effective_rate_millihz;
        std::uint64_t bytes_sent_total;
        std::uint64_t samples_dropped_total;
    };

    // `seal`/`seal_context` (default nullptr) are forwarded verbatim to
    // BtpEndpoint::send_fragment in flush() -- see BtpSealFn. TELEMETRY is
    // channel B (TraceView<->device, key E -- bally_channels.h): the dongle
    // relays it and never reads it, so the real caller passes
    // RadioSeal::seal_e.
    void configure(BtpEndpoint& endpoint, BtpSealFn seal = nullptr,
                   void* seal_context = nullptr) noexcept;

    // Binds the btp::SubscriptionTable node_ owns -- called once, after node_
    // exists (bindProtocolTransport()), separately from configure().
    void bind_subscriptions(const btp::SubscriptionTable& subscriptions) noexcept;

    static const TopicSchema* find_schema(std::uint16_t topic_id) noexcept;

    bool topic_active(std::uint16_t topic_id) const noexcept;
    std::uint16_t topic_subscriber_count(std::uint16_t topic_id) const noexcept;
    std::uint32_t topic_effective_rate_millihz(std::uint16_t topic_id) const noexcept;
    std::size_t active_subscription_count() const noexcept;
    std::uint64_t topic_period_us(std::uint16_t topic_id) const noexcept;

    std::size_t topic_stats(TopicStats* out, std::size_t max_count) const noexcept;

    // Producer side of a bounded SPSC queue. Called from the camera capture
    // task; flush() (comms task) is the sole consumer. A full queue drops the
    // new sample immediately and never waits.
    PublishResult publish_camera_status(std::uint8_t state,
                                        std::uint32_t frame_count,
                                        std::uint16_t fps_x10,
                                        std::uint64_t timestamp_us) noexcept;

    // Sends and removes at most max_samples. A radio rejection drops that
    // sample and is counted, so one bad sample cannot stall the queue.
    std::size_t flush(std::size_t max_samples) noexcept;

    std::size_t queued_count() const noexcept;
    Stats stats() const noexcept;

    static const TopicSchema* schemas(std::size_t* count) noexcept;
    static bool pack_camera_status(
        std::uint8_t state, std::uint32_t frame_count, std::uint16_t fps_x10,
        std::uint8_t output[kCameraStatusPayloadSize]) noexcept;

private:
    static constexpr std::size_t kMaxPayloadSize = kCameraStatusPayloadSize;
    static constexpr std::size_t kMaxTopics = 1U;  // matches kSchemas today
public:
    // One topic times a handful of concurrent desktop sessions behind the
    // single ESP-NOW peer -- node_'s own btp::SubscriptionTable is what
    // actually enforces this (CamRobot's node_ MaxSubscriptions template
    // argument), sized from this same constant so the two can never drift.
    static constexpr std::size_t kMaxSubscriptions = 4U;

private:
    struct Sample {
        std::uint64_t timestamp_us;
        std::uint32_t sequence;
        std::uint16_t topic_id;
        std::uint16_t payload_size;
        std::uint8_t payload[kMaxPayloadSize];
    };

    struct TopicRuntime {
        std::uint16_t topic_id = 0U;
        std::uint64_t bytes_sent_total = 0U;
        std::uint64_t samples_dropped_total = 0U;
    };

    PublishResult enqueue(std::uint16_t topic_id,
                          const std::uint8_t* payload,
                          std::size_t payload_size,
                          std::uint64_t timestamp_us) noexcept;
    void count_invalid() noexcept;
    int find_topic_index(std::uint16_t topic_id) const noexcept;
    void init_runtime_if_needed() noexcept;

    BtpEndpoint* endpoint_ = nullptr;
    BtpSealFn seal_ = nullptr;
    void* seal_context_ = nullptr;
    Sample queue_[kQueueCapacity]{};
    TopicRuntime runtime_[kMaxTopics]{};
    const btp::SubscriptionTable* subscriptions_ = nullptr;
    bool runtime_initialized_ = false;
    mutable std::atomic_flag runtime_lock_ = ATOMIC_FLAG_INIT;
    std::atomic<std::uint32_t> write_index_{0U};
    std::atomic<std::uint32_t> read_index_{0U};
    std::atomic<std::uint32_t> queued_total_{0U};
    std::atomic<std::uint32_t> sent_total_{0U};
    std::atomic<std::uint32_t> dropped_full_{0U};
    std::atomic<std::uint32_t> dropped_invalid_{0U};
    std::atomic<std::uint32_t> send_failed_{0U};
};

#endif  // TELEMETRY_PUBLISHER_H
