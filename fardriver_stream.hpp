#pragma once

#include <cstdint>
#include <cstring>

#include "fardriver.hpp"
#include "fardriver_message.hpp"

// Incremental decoder for the controller's rotating 16-byte status stream.
// It performs no writes and is suitable for fragmented UART or BLE input.
class FardriverFrameParser {
public:
    enum class Event : uint8_t {
        None,
        Frame,
        BadHeader,
        BadCRC,
        TimedOut
    };

    explicit FardriverFrameParser(uint32_t timeout_ms = 250)
        : timeout_ms_(timeout_ms) {}

    void Reset() {
        length_ = 0;
        active_ = false;
    }

    Event Tick(uint32_t now_ms) {
        if (active_ && timeout_ms_ != 0 &&
            static_cast<uint32_t>(now_ms - last_byte_ms_) >= timeout_ms_) {
            Reset();
            return Event::TimedOut;
        }
        return Event::None;
    }

    Event Push(uint8_t byte, uint32_t now_ms, FardriverMessage &out) {
        Event pending = Tick(now_ms);

        if (!active_) {
            if (byte != 0xAA) {
                return pending;
            }
            buffer_[0] = byte;
            length_ = 1;
            active_ = true;
            last_byte_ms_ = now_ms;
            return pending;
        }

        buffer_[length_++] = byte;
        last_byte_ms_ = now_ms;

        if (length_ == 2 && !ValidStatusHeader(buffer_[1])) {
            const bool byte_is_start = byte == 0xAA;
            Reset();
            if (byte_is_start) {
                buffer_[0] = 0xAA;
                length_ = 1;
                active_ = true;
                last_byte_ms_ = now_ms;
            }
            return Event::BadHeader;
        }

        if (length_ < sizeof(FardriverMessage)) {
            return pending;
        }

        FardriverMessage candidate{};
        std::memcpy(candidate.GetRaw(), buffer_, sizeof(candidate));
        if (candidate.VerifyCRC()) {
            out = candidate;
            Reset();
            return Event::Frame;
        }

        PreservePossibleStart();
        return Event::BadCRC;
    }

    static bool ValidStatusHeader(uint8_t header) {
        // Vendor PC behavior: require bit 7, then mask the lower seven bits and
        // accept IDs 0..0x36. This is equivalent to flag==2/id<0x37.
        return (header & 0x80u) != 0 && (header & 0x7fu) < 0x37u;
    }

private:
    void PreservePossibleStart() {
        for (uint8_t index = 1; index < length_; ++index) {
            if (buffer_[index] == 0xAA) {
                const uint8_t retained = static_cast<uint8_t>(length_ - index);
                std::memmove(buffer_, buffer_ + index, retained);
                length_ = retained;
                active_ = true;
                return;
            }
        }
        Reset();
    }

    uint8_t buffer_[sizeof(FardriverMessage)]{};
    uint8_t length_ = 0;
    bool active_ = false;
    uint32_t last_byte_ms_ = 0;
    uint32_t timeout_ms_;
};

// A deliberately read-only adapter around caller-supplied byte-stream callbacks.
// It cannot send configuration, system, or firmware commands.
struct FardriverReadStream {
    uint32_t (*read)(uint8_t *data, uint32_t length);
    uint32_t (*available)();
};

class FardriverTelemetryReader {
public:
    enum class PollResult : uint8_t {
        Idle,
        Updated,
        BadHeader,
        BadCRC,
        TimedOut,
        ReadError
    };

    explicit FardriverTelemetryReader(FardriverReadStream *stream,
                                      uint32_t timeout_ms = 250,
                                      uint32_t max_bytes_per_poll = 64)
        : stream_(stream), parser_(timeout_ms),
          max_bytes_per_poll_(max_bytes_per_poll == 0 ? 1 : max_bytes_per_poll) {}

    // Processes at most max_bytes_per_poll bytes so a fast or noisy stream
    // cannot monopolise the caller. On an RTOS target an unbounded drain here
    // starves peer tasks and the watchdog, so the quota is a hard bound rather
    // than a tuning hint. Returns after the first complete frame; any bytes
    // still buffered in the transport are picked up by the next call.
    PollResult Poll(FardriverData &data, uint32_t now_ms, uint8_t *address = nullptr) {
        if (stream_ == nullptr || stream_->available == nullptr || stream_->read == nullptr) {
            return PollResult::ReadError;
        }
        PollResult result = Map(parser_.Tick(now_ms));
        uint32_t budget = max_bytes_per_poll_;
        while (budget-- > 0 && stream_->available() > 0) {
            uint8_t byte = 0;
            if (stream_->read(&byte, 1) != 1) {
                return PollResult::ReadError;
            }
            FardriverMessage message{};
            const auto event = parser_.Push(byte, now_ms, message);
            if (event == FardriverFrameParser::Event::Frame) {
                const uint8_t id = message.HeaderId();
                const uint8_t addr = FardriverMessage::flashReadAddr[id];
                std::memcpy(data.GetAddr(addr), message.data, sizeof(message.data));
                if (address != nullptr) {
                    *address = addr;
                }
                // A frame is the caller's payload, but errors seen earlier in
                // this same drain would otherwise vanish. Count them so link
                // health stays observable.
                ++frames_;
                return PollResult::Updated;
            }
            if (event != FardriverFrameParser::Event::None) {
                result = Map(event);
                ++errors_;
            }
        }
        return result;
    }

    // Cumulative link-health counters. Poll() returns one result per call, so
    // these are the only way to see errors that occurred before a good frame.
    uint32_t frames() const { return frames_; }
    uint32_t errors() const { return errors_; }

    void Reset() { parser_.Reset(); }

private:
    static PollResult Map(FardriverFrameParser::Event event) {
        switch (event) {
        case FardriverFrameParser::Event::Frame: return PollResult::Updated;
        case FardriverFrameParser::Event::BadHeader: return PollResult::BadHeader;
        case FardriverFrameParser::Event::BadCRC: return PollResult::BadCRC;
        case FardriverFrameParser::Event::TimedOut: return PollResult::TimedOut;
        default: return PollResult::Idle;
        }
    }

    FardriverReadStream *stream_;
    FardriverFrameParser parser_;
    uint32_t max_bytes_per_poll_;
    uint32_t frames_ = 0;
    uint32_t errors_ = 0;
};
