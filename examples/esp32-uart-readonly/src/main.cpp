#include <Arduino.h>
#include <fardriver_stream.hpp>

#ifndef FARDRIVER_UART_RX_PIN
#define FARDRIVER_UART_RX_PIN -1
#endif
namespace {
HardwareSerial controller_uart(2);
FardriverFrameParser parser(250);
FardriverData telemetry{};

void print_capture(const FardriverMessage &message, uint8_t address) {
    Serial.printf("ms=%lu addr=%02X frame=", static_cast<unsigned long>(millis()), address);
    const uint8_t *raw = message.GetRaw();
    for (size_t i = 0; i < sizeof(message); ++i) {
        Serial.printf("%02X", raw[i]);
    }
    Serial.println();
}
} // namespace

void setup() {
    Serial.begin(115200);
    if (FARDRIVER_UART_RX_PIN < 0) {
        Serial.println("UART disabled: qualify the controller TX, then set the RX build flag.");
        return;
    }

    // 3.3 V TTL only. Do not connect BW5V. Connect only qualified GND,
    // controller TX -> ESP32 RX. Leave ESP32 TX at -1 for passive capture.
    controller_uart.begin(19200, SERIAL_8N1,
                          FARDRIVER_UART_RX_PIN, -1);
    Serial.println("Passive FarDriver UART capture active; no commands will be transmitted.");
}

void loop() {
    if (FARDRIVER_UART_RX_PIN < 0) {
        delay(1000);
        return;
    }

    FardriverMessage message{};
    while (controller_uart.available() > 0) {
        const int value = controller_uart.read();
        if (value < 0) {
            break;
        }
        const auto event = parser.Push(static_cast<uint8_t>(value), millis(), message);
        if (event == FardriverFrameParser::Event::Frame) {
            const uint8_t id = message.GetRaw()[1] & 0x7f;
            const uint8_t address = FardriverMessage::flashReadAddr[id];
            memcpy(telemetry.GetAddr(address), message.data, sizeof(message.data));
            print_capture(message, address);
        } else if (event == FardriverFrameParser::Event::BadCRC) {
            Serial.println("drop=bad-crc");
        } else if (event == FardriverFrameParser::Event::BadHeader) {
            Serial.println("drop=bad-header");
        }
    }

    if (parser.Tick(millis()) == FardriverFrameParser::Event::TimedOut) {
        Serial.println("drop=partial-timeout");
    }
}
