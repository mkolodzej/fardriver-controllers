#include <Arduino.h>
#include <NimBLEDevice.h>
#include <fardriver_stream.hpp>

#ifndef FARDRIVER_BLE_ADDRESS
#define FARDRIVER_BLE_ADDRESS ""
#endif
#ifndef FARDRIVER_BLE_ADDRESS_TYPE
#define FARDRIVER_BLE_ADDRESS_TYPE BLE_ADDR_PUBLIC
#endif
#ifndef FARDRIVER_BLE_CHARACTERISTIC_UUID
#define FARDRIVER_BLE_CHARACTERISTIC_UUID ""
#endif

namespace {
constexpr uint16_t kQueueSize = 512;
const NimBLEUUID kServiceUuid("FFE0");
const NimBLEUUID kCharacteristicUuid(FARDRIVER_BLE_CHARACTERISTIC_UUID);

QueueHandle_t rx_queue = nullptr;
FardriverFrameParser parser(500);
FardriverData telemetry{};

struct RxEvent {
    uint8_t kind; // 0 = byte, 1 = stream reset
    uint8_t byte;
};

void notification(NimBLERemoteCharacteristic *, uint8_t *data,
                  size_t length, bool) {
    for (size_t i = 0; i < length; ++i) {
        const RxEvent event{0, data[i]};
        if (xQueueSend(rx_queue, &event, 0) != pdTRUE) {
            xQueueReset(rx_queue);
            const RxEvent reset{1, 0};
            xQueueSend(rx_queue, &reset, 0);
            break;
        }
    }
}

bool connect_and_subscribe() {
    if (strlen(FARDRIVER_BLE_ADDRESS) != 17) {
        Serial.println("BLE disabled: set an explicitly observed adapter address.");
        return false;
    }
    if (strlen(FARDRIVER_BLE_CHARACTERISTIC_UUID) == 0) {
        Serial.println("BLE disabled: set an observed notification characteristic.");
        return false;
    }

    NimBLEClient *client = NimBLEDevice::createClient();
    client->setConnectTimeout(30000);
    const NimBLEAddress peer(FARDRIVER_BLE_ADDRESS, FARDRIVER_BLE_ADDRESS_TYPE);
    if (!client->connect(peer)) {
        Serial.println("BLE connect failed");
        return false;
    }

    NimBLERemoteService *service = client->getService(kServiceUuid);
    if (service == nullptr) {
        Serial.println("FFE0 service absent");
        client->disconnect();
        return false;
    }
    NimBLERemoteCharacteristic *characteristic =
        service->getCharacteristic(kCharacteristicUuid);
    if (characteristic == nullptr ||
        (!characteristic->canNotify() && !characteristic->canIndicate())) {
        Serial.println("Configured notification characteristic absent");
        client->disconnect();
        return false;
    }

    const bool notifications = characteristic->canNotify();
    if (!characteristic->subscribe(notifications, notification)) {
        Serial.println("Notification subscription failed");
        client->disconnect();
        return false;
    }
    Serial.println("BLE payload-read-only subscription active; link state is still modified.");
    return true;
}

void print_capture(const FardriverMessage &message, uint8_t address) {
    Serial.printf("ms=%lu addr=%02X frame=", static_cast<unsigned long>(millis()), address);
    for (size_t i = 0; i < sizeof(message); ++i) {
        Serial.printf("%02X", message.GetRaw()[i]);
    }
    Serial.println();
}
} // namespace

void setup() {
    Serial.begin(115200);
    rx_queue = xQueueCreate(kQueueSize, sizeof(RxEvent));
    if (rx_queue == nullptr) {
        Serial.println("BLE RX queue allocation failed");
        return;
    }
    NimBLEDevice::init("FarDriver-passive-reader");
    connect_and_subscribe();
}

void loop() {
    RxEvent rx{};
    FardriverMessage message{};
    while (rx_queue != nullptr && xQueueReceive(rx_queue, &rx, 0) == pdTRUE) {
        if (rx.kind == 1) {
            parser.Reset();
            Serial.println("drop=ble-queue-overflow");
            continue;
        }
        const auto event = parser.Push(rx.byte, millis(), message);
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
    delay(1);
}
