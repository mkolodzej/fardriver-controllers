#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

#include "fardriver_controller.hpp"

namespace {
std::array<uint8_t, 512> written{};
uint32_t written_length = 0;
std::array<uint8_t, 32> incoming{};
uint32_t incoming_length = 0;
uint32_t read_limit = UINT32_MAX;

// Accepts at most write_chunk_limit bytes per call, emulating a transport whose
// TX buffer fills mid-frame.
uint32_t write_chunk_limit = UINT32_MAX;
uint32_t write_calls = 0;

uint32_t write_bytes(const uint8_t *data, uint32_t length) {
    const uint32_t accepted = length < write_chunk_limit ? length : write_chunk_limit;
    assert(written_length + accepted <= written.size());
    std::memcpy(written.data() + written_length, data, accepted);
    written_length += accepted;
    ++write_calls;
    return accepted;
}

void reset_write_capture() {
    written_length = 0;
    write_calls = 0;
}

// A separate byte-accurate stream for exercising legacy Read() resynchronisation.
std::vector<uint8_t> stream_bytes;
size_t stream_pos = 0;

uint32_t stream_available() {
    return static_cast<uint32_t>(stream_bytes.size() - stream_pos);
}

uint32_t stream_read(uint8_t *data, uint32_t length) {
    uint32_t count = 0;
    while (count < length && stream_pos < stream_bytes.size())
        data[count++] = stream_bytes[stream_pos++];
    return count;
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
    reset_write_capture();
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
    reset_write_capture();
    assert(controller.WriteAddr(word, 0x22, sizeof(word)) == FardriverController::WriteResult::Success);
    assert(written_length == 8);
    assert(written[0] == 0xAA && written[1] == 0xC6);
    assert(written[2] == 0x22 && written[3] == 0x22);
    assert(written[4] == 0x12 && written[5] == 0x34);
    assert(controller.WriteAddr(word, 0x22, 60) == FardriverController::WriteResult::UnsupportedLength);

    std::array<uint8_t, 0x180> cflash{};
    for (size_t i = 0; i < cflash.size(); ++i)
        cflash[i] = static_cast<uint8_t>(i);
    reset_write_capture();
    assert(controller.SaveCANParameterImage(cflash.data(), cflash.size()) ==
           FardriverController::WriteResult::Success);
    assert(written_length == 390);
    assert(written[0] == 0xAA && written[1] == 0xFF);
    assert(std::memcmp(written.data() + 4, cflash.data(), cflash.size()) == 0);

    // A transport that accepts only part of each write must still put the whole
    // frame on the wire. Abandoning it mid-frame leaves the controller waiting
    // on a truncated packet, so short writes are retried rather than reported
    // as a failure.
    const std::array<uint8_t, 390> full_frame = [&] {
        std::array<uint8_t, 390> copy{};
        std::memcpy(copy.data(), written.data(), copy.size());
        return copy;
    }();
    for (const uint32_t limit : {1u, 7u, 64u, 389u}) {
        reset_write_capture();
        write_chunk_limit = limit;
        assert(controller.SaveCANParameterImage(cflash.data(), cflash.size()) ==
               FardriverController::WriteResult::Success);
        assert(written_length == 390);
        assert(std::memcmp(written.data(), full_frame.data(), full_frame.size()) == 0);
        assert(write_calls > 1);
    }
    // A transport that accepts nothing must fail rather than spin forever.
    reset_write_capture();
    write_chunk_limit = 0;
    assert(controller.SaveCANParameterImage(cflash.data(), cflash.size()) ==
           FardriverController::WriteResult::TransportFailure);
    write_chunk_limit = UINT32_MAX;

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

    // Layout-independent header accessors must agree with the wire byte rather
    // than with implementation-defined bitfield allocation order.
    FardriverMessage probe{};
    probe.GetRaw()[1] = 0xB6;
    assert(probe.HeaderFlag() == 2 && probe.HeaderId() == 0x36);

    // Legacy Read() must discard leading noise within a single call. Previously
    // it consumed one byte per call while requiring 16 available, so it stayed
    // permanently behind the stream after losing sync.
    frame.crc[1] ^= 1;
    assert(frame.VerifyCRC());
    stream_bytes.clear();
    for (int i = 0; i < 5; ++i)
        stream_bytes.push_back(0x00); // noise, not a start byte
    for (size_t i = 0; i < sizeof(frame); ++i)
        stream_bytes.push_back(frame.GetRaw()[i]);
    stream_pos = 0;
    FardriverSerial stream_serial{write_bytes, stream_read, stream_available};
    FardriverController stream_controller(&stream_serial);
    FardriverData decoded{};
    const auto read_result = stream_controller.Read(&decoded);
    assert(read_result.error == FardriverController::Success);
    assert(read_result.addr == 0xE2);
    assert(stream_pos == stream_bytes.size());
}
