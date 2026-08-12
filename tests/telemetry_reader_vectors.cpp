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
}
