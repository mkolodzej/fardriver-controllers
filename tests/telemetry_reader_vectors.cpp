#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>

#include "fardriver_stream.hpp"

namespace {
constexpr std::array<uint8_t, 19> kInput = {
    0x00, 0x55, 0x7F,
    0xAA, 0x80, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
    0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x07, 0xFE,
};
size_t input_pos = 0;

uint32_t available() {
    return static_cast<uint32_t>(kInput.size() - input_pos);
}

uint32_t read_one(uint8_t *data, uint32_t length) {
    if (length == 0 || input_pos == kInput.size()) {
        return 0;
    }
    *data = kInput[input_pos++];
    return 1;
}

// 0xAA followed by an invalid status header (bit 7 clear) is a real BadHeader
// event, immediately followed by a valid frame in the same drain.
constexpr std::array<uint8_t, 18> kBadHeaderInput = {
    0xAA, 0x7F,
    0xAA, 0x80, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
    0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x07, 0xFE,
};
size_t bad_header_pos = 0;

uint32_t bad_header_available() {
    return static_cast<uint32_t>(kBadHeaderInput.size() - bad_header_pos);
}

uint32_t bad_header_read(uint8_t *data, uint32_t length) {
    if (length == 0 || bad_header_pos == kBadHeaderInput.size()) {
        return 0;
    }
    *data = kBadHeaderInput[bad_header_pos++];
    return 1;
}
} // namespace

int main() {
    FardriverTelemetryReader null_reader(nullptr);
    FardriverData null_telemetry{};
    assert(null_reader.Poll(null_telemetry, 0) ==
           FardriverTelemetryReader::PollResult::ReadError);

    FardriverReadStream stream{read_one, available};
    FardriverTelemetryReader reader(&stream);
    FardriverData telemetry{};
    uint8_t address = 0;

    assert(reader.Poll(telemetry, 10, &address) ==
           FardriverTelemetryReader::PollResult::Updated);
    assert(address == 0xE2);
    const uint8_t expected[12] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
        0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
    };
    assert(std::memcmp(telemetry.GetAddr(0xE2), expected, sizeof(expected)) == 0);

    assert(reader.frames() == 1);
    // Leading bytes that are not 0xAA are hunted past, not counted as errors:
    // the parser is resynchronising, which is normal mid-stream behaviour.
    assert(reader.errors() == 0);

    // Poll() must honour its per-call byte quota so a saturated stream cannot
    // starve the caller. With a quota of 4 the 19-byte input cannot complete a
    // frame in one call, and the reader must yield instead of draining.
    input_pos = 0;
    FardriverTelemetryReader bounded(&stream, 250, 4);
    FardriverData bounded_telemetry{};
    assert(bounded.Poll(bounded_telemetry, 10) !=
           FardriverTelemetryReader::PollResult::Updated);
    assert(input_pos == 4);
    // A zero quota must be clamped to forward progress, never to a stall.
    input_pos = 0;
    FardriverTelemetryReader clamped(&stream, 250, 0);
    (void)clamped.Poll(bounded_telemetry, 10);
    assert(input_pos == 1);

    // Repeated bounded polls still assemble the frame across calls.
    input_pos = 0;
    FardriverTelemetryReader resumed(&stream, 250, 4);
    FardriverData resumed_telemetry{};
    auto resumed_result = FardriverTelemetryReader::PollResult::Idle;
    for (int i = 0; i < 16 && resumed_result != FardriverTelemetryReader::PollResult::Updated; ++i) {
        resumed_result = resumed.Poll(resumed_telemetry, 10, &address);
    }
    assert(resumed_result == FardriverTelemetryReader::PollResult::Updated);
    assert(address == 0xE2);
    assert(std::memcmp(resumed_telemetry.GetAddr(0xE2), expected, sizeof(expected)) == 0);

    // A BadHeader occurring BEFORE a good frame in the same drain is masked by
    // the Updated return value, so the counter is the only way to see it. This
    // is the link-health visibility the counters exist for.
    bad_header_pos = 0;
    FardriverReadStream bad_header_stream{bad_header_read, bad_header_available};
    FardriverTelemetryReader counting(&bad_header_stream, 250, 64);
    FardriverData counting_telemetry{};
    assert(counting.Poll(counting_telemetry, 10, &address) ==
           FardriverTelemetryReader::PollResult::Updated);
    assert(counting.frames() == 1);
    assert(counting.errors() == 1);

    // Header decoding must not depend on implementation-defined bitfield order.
    FardriverMessage header_probe{};
    header_probe.GetRaw()[1] = 0x80; // flag=2, id=0
    assert(header_probe.HeaderFlag() == 2 && header_probe.HeaderId() == 0);
    header_probe.GetRaw()[1] = 0xB6; // flag=2, id=0x36
    assert(header_probe.HeaderFlag() == 2 && header_probe.HeaderId() == 0x36);
}
