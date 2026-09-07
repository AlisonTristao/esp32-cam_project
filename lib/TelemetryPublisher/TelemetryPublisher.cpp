#include <TelemetryPublisher.h>

#include <BtpTransport.h>

#include <cstring>

namespace {

constexpr TelemetryPublisher::FieldSchema kCameraStatusFields[] = {
    {1U, 0U, "state", btp::WireType::Uint8, "1", 1.0F, 0.0F, 1U, false},
    {2U, 1U, "frame_count", btp::WireType::Uint32, "1", 1.0F, 0.0F, 1U, false},
    {3U, 2U, "fps", btp::WireType::Uint16, "fps", 0.1F, 0.0F, 1U, false},
};

constexpr TelemetryPublisher::TopicSchema kSchemas[] = {
    {TelemetryPublisher::kCameraStatusTopicId,
     TelemetryPublisher::kSchemaVersion,
     "camera.status",
     TelemetryPublisher::Encoding::PackedLe,
     kCameraStatusFields,
     sizeof(kCameraStatusFields) / sizeof(kCameraStatusFields[0]),
     TelemetryPublisher::kCameraStatusPayloadSize,
     2000U,  // max: 2 Hz -- this topic is a low-rate status summary, not a
             // frame stream (pixel data stays on the serial ASCII path)
     100U,   // min: 0.1 Hz
     1000U},  // default/nominal: 1 Hz
    {TelemetryPublisher::kCameraMatrixTopicId,
     TelemetryPublisher::kSchemaVersion,
     "camera.matrix",
     TelemetryPublisher::Encoding::OpaqueBytes,
     nullptr,
     0U,  // OpaqueBytes is the complete topic body; it has no structured fields
     TelemetryPublisher::kMaxCameraMatrixPayloadSize,
     // max: 10 Hz -- not a real ceiling, a permissive one. The document is
     // ~1.9 KB, fragmented into ~10 sealed ESP-NOW frames
     // (kEspNowMaxPayloadSize - kBtpAeadTagSize = 194 B/fragment), and
     // TxScheduler::pump() starts at most one send per comms-task tick
     // (~10 ms) -- so ~100 ms/document (~10 Hz) is already close to the
     // radio's own pacing limit. publish_camera_matrix()'s replace-in-place
     // drop (matrix_stage_ still pending) is what actually throttles this to
     // whatever the radio can sustain; this ceiling just stops it from
     // being lower than that on purpose.
     10000U,
     100U,    // min: 0.1 Hz
     10000U},  // default/nominal: same as max -- a subscriber that does not
               // ask for a specific rate gets the fastest one right away
};

// camera.matrix (OpaqueBytes) is the one topic btp::SampleWriter does not
// build -- its body is a packed bitmap, not schema fields -- so its
// two-octet schema_version prefix is still written here, same as bally_OS's
// system.monitor.
void write_u16_le(std::uint8_t* output, std::uint16_t value) noexcept {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
}

// Bounded spin: this lock is only ever held for a handful of field
// reads/writes (no allocation, no radio call), same reasoning as bally_OS's
// TelemetryPublisher.cpp.
constexpr int kLockSpinAttempts = 256;

class RuntimeLockGuard {
public:
    explicit RuntimeLockGuard(std::atomic_flag& flag) noexcept : flag_(flag) {
        for (int i = 0; i < kLockSpinAttempts; ++i) {
            if (!flag_.test_and_set(std::memory_order_acquire)) {
                acquired_ = true;
                return;
            }
        }
    }
    ~RuntimeLockGuard() noexcept {
        if (acquired_) flag_.clear(std::memory_order_release);
    }
    bool acquired() const noexcept { return acquired_; }

private:
    std::atomic_flag& flag_;
    bool acquired_ = false;
};

}  // namespace

void TelemetryPublisher::configure(BtpEndpoint& endpoint, BtpSealFn seal,
                                   void* seal_context) noexcept {
    endpoint_ = &endpoint;
    seal_ = seal;
    seal_context_ = seal_context;
}

void TelemetryPublisher::bind_subscriptions(
    const btp::SubscriptionTable& subscriptions) noexcept {
    subscriptions_ = &subscriptions;
}

TelemetryPublisher::PublishResult TelemetryPublisher::publish_camera_status(
    std::uint8_t state, std::uint32_t frame_count, std::uint16_t fps_x10,
    std::uint64_t timestamp_us) noexcept {
    std::uint8_t payload[kCameraStatusPayloadSize];
    if (!pack_camera_status(state, frame_count, fps_x10, payload)) {
        count_invalid();
        return PublishResult::InvalidValue;
    }
    return enqueue(kCameraStatusTopicId, payload, sizeof(payload), timestamp_us);
}

TelemetryPublisher::PublishResult TelemetryPublisher::publish_camera_matrix(
    const char* body, std::size_t length, std::uint64_t timestamp_us) noexcept {
    if (endpoint_ == nullptr) {
        count_invalid();
        return PublishResult::NotConfigured;
    }

    // Replace-in-place: if flush() has not taken the previous document yet,
    // drop this one (a fresh matrix follows on the next capture) rather than
    // overwrite a buffer the consumer may be reading.
    if (matrix_stage_.pending.load(std::memory_order_acquire)) {
        dropped_full_.fetch_add(1U, std::memory_order_relaxed);
        RuntimeLockGuard guard(runtime_lock_);
        if (guard.acquired()) {
            init_runtime_if_needed();
            const int idx = find_topic_index(kCameraMatrixTopicId);
            if (idx >= 0) {
                runtime_[static_cast<std::size_t>(idx)].samples_dropped_total += 1U;
            }
        }
        return PublishResult::QueueFull;
    }

    // Not pending -> the producer owns matrix_stage_.payload. pack validates
    // fully before writing a byte, so a rejected document leaves it untouched.
    std::size_t payload_size = 0U;
    if (!pack_camera_matrix(body, length, matrix_stage_.payload,
                            &payload_size)) {
        count_invalid();
        return PublishResult::InvalidValue;
    }

    std::uint32_t sequence = 0U;
    if (!endpoint_->try_reserve_sequence(&sequence)) {
        count_invalid();
        return PublishResult::SequenceUnavailable;
    }

    matrix_stage_.timestamp_us = timestamp_us;
    matrix_stage_.sequence = sequence;
    matrix_stage_.payload_size = static_cast<std::uint16_t>(payload_size);
    matrix_stage_.pending.store(true, std::memory_order_release);

    queued_total_.fetch_add(1U, std::memory_order_relaxed);
    return PublishResult::Queued;
}

std::size_t TelemetryPublisher::flush(std::size_t max_samples) noexcept {
    if (endpoint_ == nullptr) return 0U;

    std::size_t processed = 0U;

    // The out-of-band camera.matrix document is one logical sample no matter
    // how many wire fragments it needs. Drain it before the numeric queue,
    // same ordering bally_OS uses for system.monitor: its sequence was
    // reserved earlier (low-rate topic), so sending it first keeps sequences
    // ascending on the wire in the common case. send_logical_reserved() seals
    // the whole document once, then fragments it; a failed seal stays
    // fail-closed.
    if (processed < max_samples &&
        matrix_stage_.pending.load(std::memory_order_acquire)) {
        const bool sent = endpoint_->send_logical_reserved(
            btp::MessageType::Telemetry, kCameraMatrixTopicId,
            matrix_stage_.sequence, matrix_stage_.payload,
            matrix_stage_.payload_size, matrix_stage_.timestamp_us, seal_,
            seal_context_);
        if (sent) {
            sent_total_.fetch_add(1U, std::memory_order_relaxed);
            RuntimeLockGuard guard(runtime_lock_);
            if (guard.acquired()) {
                init_runtime_if_needed();
                const int idx = find_topic_index(kCameraMatrixTopicId);
                if (idx >= 0) {
                    runtime_[static_cast<std::size_t>(idx)].bytes_sent_total +=
                        matrix_stage_.payload_size;
                }
            }
        } else {
            send_failed_.fetch_add(1U, std::memory_order_relaxed);
        }
        matrix_stage_.pending.store(false, std::memory_order_release);
        ++processed;
    }

    while (processed < max_samples) {
        const std::uint32_t read = read_index_.load(std::memory_order_relaxed);
        const std::uint32_t write = write_index_.load(std::memory_order_acquire);
        if (read == write) break;

        const Sample& sample = queue_[read % kQueueCapacity];
        const bool sent = endpoint_->send_fragment(
            btp::MessageType::Telemetry, sample.topic_id, sample.sequence,
            sample.timestamp_us, sample.payload, sample.payload_size, 0U, 1U,
            seal_, seal_context_);
        if (sent) {
            sent_total_.fetch_add(1U, std::memory_order_relaxed);
            RuntimeLockGuard guard(runtime_lock_);
            if (guard.acquired()) {
                init_runtime_if_needed();
                const int idx = find_topic_index(sample.topic_id);
                if (idx >= 0) {
                    runtime_[static_cast<std::size_t>(idx)].bytes_sent_total +=
                        sample.payload_size;
                }
            }
        } else {
            send_failed_.fetch_add(1U, std::memory_order_relaxed);
        }

        read_index_.store(read + 1U, std::memory_order_release);
        ++processed;
    }
    return processed;
}

std::size_t TelemetryPublisher::queued_count() const noexcept {
    const std::uint32_t write = write_index_.load(std::memory_order_acquire);
    const std::uint32_t read = read_index_.load(std::memory_order_acquire);
    const std::size_t staged =
        matrix_stage_.pending.load(std::memory_order_acquire) ? 1U : 0U;
    return static_cast<std::size_t>(write - read) + staged;
}

TelemetryPublisher::Stats TelemetryPublisher::stats() const noexcept {
    return {
        queued_total_.load(std::memory_order_relaxed),
        sent_total_.load(std::memory_order_relaxed),
        dropped_full_.load(std::memory_order_relaxed),
        dropped_invalid_.load(std::memory_order_relaxed),
        send_failed_.load(std::memory_order_relaxed),
    };
}

const TelemetryPublisher::TopicSchema* TelemetryPublisher::schemas(
    std::size_t* count) noexcept {
    if (count != nullptr) *count = sizeof(kSchemas) / sizeof(kSchemas[0]);
    return kSchemas;
}

bool TelemetryPublisher::pack_camera_status(
    std::uint8_t state, std::uint32_t frame_count, std::uint16_t fps_x10,
    std::uint8_t output[kCameraStatusPayloadSize]) noexcept {
    if (output == nullptr) return false;

    const btp::FieldSpec specs[] = {kCameraStatusFields[0].spec(),
                                    kCameraStatusFields[1].spec(),
                                    kCameraStatusFields[2].spec()};
    btp::SampleWriter writer(output, kCameraStatusPayloadSize, specs, 3U);
    std::size_t written = 0U;
    return writer.begin(kSchemaVersion) == btp::MessageError::Ok &&
           writer.put_u64(state) == btp::MessageError::Ok &&
           writer.put_u64(frame_count) == btp::MessageError::Ok &&
           writer.put_u64(fps_x10) == btp::MessageError::Ok &&
           writer.finish(&written) == btp::MessageError::Ok;
}

bool TelemetryPublisher::pack_camera_matrix(
    const char* body, std::size_t length,
    std::uint8_t output[kMaxCameraMatrixPayloadSize],
    std::size_t* bytes_written) noexcept {
    if (output == nullptr || bytes_written == nullptr ||
        (body == nullptr && length != 0U) ||
        length > kMaxCameraMatrixBodySize) {
        return false;
    }
    write_u16_le(output, kSchemaVersion);
    if (length != 0U) {
        std::memcpy(output + 2U, body, length);
    }
    *bytes_written = length + 2U;
    return true;
}

TelemetryPublisher::PublishResult TelemetryPublisher::enqueue(
    std::uint16_t topic_id,
    const std::uint8_t* payload,
    std::size_t payload_size,
    std::uint64_t timestamp_us) noexcept {
    if (endpoint_ == nullptr) {
        count_invalid();
        return PublishResult::NotConfigured;
    }
    if (payload == nullptr || payload_size == 0U ||
        payload_size > kMaxPayloadSize || topic_id == 0U) {
        count_invalid();
        return PublishResult::InvalidValue;
    }

    const std::uint32_t write = write_index_.load(std::memory_order_relaxed);
    const std::uint32_t read = read_index_.load(std::memory_order_acquire);
    if (write - read >= kQueueCapacity) {
        dropped_full_.fetch_add(1U, std::memory_order_relaxed);
        RuntimeLockGuard guard(runtime_lock_);
        if (guard.acquired()) {
            init_runtime_if_needed();
            const int idx = find_topic_index(topic_id);
            if (idx >= 0) {
                runtime_[static_cast<std::size_t>(idx)].samples_dropped_total += 1U;
            }
        }
        return PublishResult::QueueFull;
    }

    std::uint32_t sequence = 0U;
    if (!endpoint_->try_reserve_sequence(&sequence)) {
        count_invalid();
        return PublishResult::SequenceUnavailable;
    }

    Sample& sample = queue_[write % kQueueCapacity];
    sample.timestamp_us = timestamp_us;
    sample.sequence = sequence;
    sample.topic_id = topic_id;
    sample.payload_size = static_cast<std::uint16_t>(payload_size);
    std::memcpy(sample.payload, payload, payload_size);

    write_index_.store(write + 1U, std::memory_order_release);
    queued_total_.fetch_add(1U, std::memory_order_relaxed);
    return PublishResult::Queued;
}

void TelemetryPublisher::count_invalid() noexcept {
    dropped_invalid_.fetch_add(1U, std::memory_order_relaxed);
}

int TelemetryPublisher::find_topic_index(std::uint16_t topic_id) const noexcept {
    for (std::size_t i = 0U; i < kMaxTopics; ++i) {
        if (runtime_[i].topic_id == topic_id) return static_cast<int>(i);
    }
    return -1;
}

void TelemetryPublisher::init_runtime_if_needed() noexcept {
    if (runtime_initialized_) return;
    std::size_t count = 0U;
    const TopicSchema* schema_list = schemas(&count);
    for (std::size_t i = 0U; i < count && i < kMaxTopics; ++i) {
        runtime_[i].topic_id = schema_list[i].topic_id;
    }
    runtime_initialized_ = true;
}

const TelemetryPublisher::TopicSchema* TelemetryPublisher::find_schema(
    std::uint16_t topic_id) noexcept {
    std::size_t schema_count = 0U;
    const TopicSchema* schema_list = schemas(&schema_count);
    for (std::size_t i = 0U; i < schema_count; ++i) {
        if (schema_list[i].topic_id == topic_id) return &schema_list[i];
    }
    return nullptr;
}

bool TelemetryPublisher::topic_active(std::uint16_t topic_id) const noexcept {
    return topic_subscriber_count(topic_id) != 0U;
}

std::uint16_t TelemetryPublisher::topic_subscriber_count(
    std::uint16_t topic_id) const noexcept {
    if (topic_id == 0U || subscriptions_ == nullptr) return 0U;
    return static_cast<std::uint16_t>(subscriptions_->subscriber_count(topic_id));
}

std::uint32_t TelemetryPublisher::topic_effective_rate_millihz(
    std::uint16_t topic_id) const noexcept {
    if (topic_id == 0U || subscriptions_ == nullptr) return 0U;
    return subscriptions_->aggregate_rate_millihz(topic_id);
}

std::size_t TelemetryPublisher::active_subscription_count() const noexcept {
    if (subscriptions_ == nullptr) return 0U;
    std::size_t schema_count = 0U;
    const TopicSchema* schema_list = schemas(&schema_count);
    std::size_t count = 0U;
    for (std::size_t i = 0U; i < schema_count; ++i) {
        count += subscriptions_->subscriber_count(schema_list[i].topic_id);
    }
    return count;
}

std::uint64_t TelemetryPublisher::topic_period_us(std::uint16_t topic_id) const noexcept {
    const std::uint32_t rate = topic_effective_rate_millihz(topic_id);
    if (rate == 0U) return 0U;
    return 1000000000ULL / static_cast<std::uint64_t>(rate);
}

std::size_t TelemetryPublisher::topic_stats(TopicStats* out, std::size_t max_count) const noexcept {
    if (out == nullptr) return 0U;
    RuntimeLockGuard guard(runtime_lock_);
    if (!guard.acquired()) return 0U;
    const_cast<TelemetryPublisher*>(this)->init_runtime_if_needed();

    std::size_t written = 0U;
    for (std::size_t i = 0U; i < kMaxTopics && written < max_count; ++i) {
        const TopicRuntime& runtime = runtime_[i];
        if (runtime.topic_id == 0U) continue;
        out[written].topic_id = runtime.topic_id;
        out[written].subscriber_count = topic_subscriber_count(runtime.topic_id);
        out[written].effective_rate_millihz = topic_effective_rate_millihz(runtime.topic_id);
        out[written].bytes_sent_total = runtime.bytes_sent_total;
        out[written].samples_dropped_total = runtime.samples_dropped_total;
        ++written;
    }
    return written;
}
