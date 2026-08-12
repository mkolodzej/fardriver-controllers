#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>

#include "fardriver_controller.hpp"

namespace {
std::array<uint8_t, 512> written{};
uint32_t written_length = 0;
std::array<uint8_t, 32> incoming{};
uint32_t incoming_length = 0;
uint32_t read_limit = UINT32_MAX;

uint32_t write_bytes(const uint8_t *data, uint32_t length) {
    assert(length <= written.size());
    std::memcpy(written.data(), data, length);
    written_length = length;
    return length;
}

uint32_t read_bytes(uint8_t *data, uint32_t length) {
    const uint32_t count = length < incoming_length ? length : incoming_length;
    const uint32_t limited = count < read_limit ? count : read_limit;
    std::memcpy(data, incoming.data(), limited);
    return limited;
}
uint32_t available_bytes() { return incoming_length; }

void expect_packet(const std::array<uint8_t, 8> &expected) {
    assert(written_length == expected.size());
    assert(std::memcmp(written.data(), expected.data(), expected.size()) == 0);
}
} // namespace

int main() {
    FardriverSerial serial{write_bytes, read_bytes, available_bytes};
    FardriverController controller(&serial);

    controller.Open();
    expect_packet({0xAA, 0x13, 0xEC, 0x07, 0x01, 0xF1, 0xA2, 0x5D});

    controller.KeepAlive();
    expect_packet({0xAA, 0x13, 0xEC, 0x07, 0x5F, 0x5F, 0x6E, 0x91});

    controller.ObservedPcPollExperimental();
    expect_packet({0xAA, 0x05, 0xFA, 0x01, 0x5F, 0x5F, 0x68, 0x97});

    const uint8_t word[] = {0x12, 0x34};
    assert(controller.WriteAddr(word, 0x22, sizeof(word)) == FardriverController::WriteResult::Success);
    assert(written_length == 8);
    assert(written[0] == 0xAA && written[1] == 0xC6);
    assert(written[2] == 0x22 && written[3] == 0x22);
    assert(written[4] == 0x12 && written[5] == 0x34);
    assert(controller.WriteAddr(word, 0x22, 60) == FardriverController::WriteResult::UnsupportedLength);

    std::array<uint8_t, 0x180> cflash{};
    for (size_t i = 0; i < cflash.size(); ++i)
        cflash[i] = static_cast<uint8_t>(i);
    assert(controller.SaveCANParameterImage(cflash.data(), cflash.size()) ==
           FardriverController::WriteResult::Success);
    assert(written_length == 390);
    assert(written[0] == 0xAA && written[1] == 0xFF);
    assert(std::memcmp(written.data() + 4, cflash.data(), cflash.size()) == 0);

    const uint8_t expected_crc[8] = {1,2,3,4,5,6,7,8};
    incoming_length = 0;
    assert(controller.VerifyCRCMessage(9, expected_crc, 3) ==
           FardriverController::CRCMessageResult::Timeout);
    incoming = {0xAA, 0x1F, 0x09, 0x09, 1,2,3,4,5,6,7,8, 0,0,0,0};
    uint16_t response_sum = 0;
    for (uint8_t i = 0; i < 14; ++i)
        response_sum = static_cast<uint16_t>(response_sum + incoming[i]);
    incoming[14] = static_cast<uint8_t>(response_sum >> 8);
    incoming[15] = static_cast<uint8_t>(response_sum & 0xFF);
    incoming_length = 16;
    assert(controller.VerifyCRCMessage(9, expected_crc) ==
           FardriverController::CRCMessageResult::Success);
    incoming[2] = incoming[3] = 10;
    response_sum = 0;
    for (uint8_t i = 0; i < 14; ++i)
        response_sum = static_cast<uint16_t>(response_sum + incoming[i]);
    incoming[14] = static_cast<uint8_t>(response_sum >> 8);
    incoming[15] = static_cast<uint8_t>(response_sum & 0xFF);
    assert(controller.VerifyCRCMessage(9, expected_crc) ==
           FardriverController::CRCMessageResult::IndexMismatch);
    incoming[2] = incoming[3] = 9;
    incoming[4] ^= 1;
    response_sum = 0;
    for (uint8_t i = 0; i < 14; ++i)
        response_sum = static_cast<uint16_t>(response_sum + incoming[i]);
    incoming[14] = static_cast<uint8_t>(response_sum >> 8);
    incoming[15] = static_cast<uint8_t>(response_sum & 0xFF);
    assert(controller.VerifyCRCMessage(9, expected_crc) ==
           FardriverController::CRCMessageResult::CRCMismatch);
    incoming[4] ^= 1;
    assert(controller.VerifyCRCMessage(9, expected_crc) ==
           FardriverController::CRCMessageResult::ChecksumMismatch);
    response_sum = 0;
    for (uint8_t i = 0; i < 14; ++i)
        response_sum = static_cast<uint16_t>(response_sum + incoming[i]);
    incoming[14] = static_cast<uint8_t>(response_sum >> 8);
    incoming[15] = static_cast<uint8_t>(response_sum & 0xFF);
    read_limit = 8;
    assert(controller.VerifyCRCMessage(9, expected_crc) ==
           FardriverController::CRCMessageResult::ShortRead);
    read_limit = UINT32_MAX;

    FardriverMessage frame{};
    frame.start = 0xAA;
    frame.header.id = 0;
    frame.header.flag = 2;
    for (uint8_t i = 0; i < sizeof(frame.data); ++i) {
        frame.data[i] = i;
    }

    uint8_t a = 0x3C;
    uint8_t b = 0x7F;
    for (uint8_t i = 0; i < 14; ++i) {
        const uint8_t index = a ^ frame.GetRaw()[i];
        a = b ^ FardriverMessage::crcTableHi[index];
        b = FardriverMessage::crcTableLo[index];
    }
    frame.crc[0] = a;
    frame.crc[1] = b;
    assert(frame.VerifyCRC());
    assert(FardriverMessage::flashReadAddr[frame.header.id] == 0xE2);

    frame.crc[1] ^= 1;
    assert(!frame.VerifyCRC());
}
