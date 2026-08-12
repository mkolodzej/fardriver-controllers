#include <Arduino.h>
#include <fardriver_controller.hpp>
#include <fardriver_stream.hpp>

static_assert(sizeof(FardriverMessage) == 16,
              "FarDriver status messages must remain 16 bytes");
static_assert(sizeof(FardriverData) == 512,
              "FarDriver register map must remain 512 bytes");

void setup() {
    FardriverMessage message{};
    FardriverFrameParser parser{};
    const uint8_t frame[16] = {
        0xAA, 0x80, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
        0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x07, 0xFE,
    };
    FardriverFrameParser::Event event = FardriverFrameParser::Event::None;
    for (const uint8_t byte : frame) {
        event = parser.Push(byte, 0, message);
    }
    if (event != FardriverFrameParser::Event::Frame) {
        abort();
    }
    if (!message.VerifyStart()) {
        abort();
    }
}

void loop() {}
