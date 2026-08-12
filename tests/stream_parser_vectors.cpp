#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "fardriver_stream.hpp"

namespace {
using Parser = FardriverFrameParser;
using Event = Parser::Event;
using Bytes = std::array<uint8_t, sizeof(FardriverMessage)>;

constexpr Bytes kGoldenZeroToEleven = {
    0xAA, 0x80, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
    0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x07, 0xFE,
};

constexpr Bytes kGoldenLastStatusId = {
    0xAA, 0xB6, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60,
    0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xE6, 0x3B,
};

void expect_message(const FardriverMessage &message, const Bytes &expected) {
    assert(std::memcmp(&message, expected.data(), expected.size()) == 0);
}

Event push(Parser &parser, const Bytes &bytes, FardriverMessage &out,
           uint32_t &now_ms) {
    Event result = Event::None;
    for (const uint8_t byte : bytes) {
        result = parser.Push(byte, now_ms++, out);
    }
    return result;
}

void test_golden_frames() {
    Parser parser;
    FardriverMessage out{};
    uint32_t now_ms = 0;

    assert(push(parser, kGoldenZeroToEleven, out, now_ms) == Event::Frame);
    expect_message(out, kGoldenZeroToEleven);
    assert(out.header.id == 0);
    assert(out.header.flag == 2);

    assert(push(parser, kGoldenLastStatusId, out, now_ms) == Event::Frame);
    expect_message(out, kGoldenLastStatusId);
    assert(out.header.id == 0x36);
    assert(out.header.flag == 2);
}

void test_noise_resynchronization() {
    Parser parser;
    FardriverMessage out{};
    uint32_t now_ms = 10;
    constexpr std::array<uint8_t, 8> noise = {
        0x00, 0x55, 0xFE, 0x7F, 0x01, 0xA9, 0xAB, 0x42,
    };

    for (const uint8_t byte : noise) {
        assert(parser.Push(byte, now_ms++, out) == Event::None);
    }
    assert(push(parser, kGoldenZeroToEleven, out, now_ms) == Event::Frame);
    expect_message(out, kGoldenZeroToEleven);
}

void test_split_frame() {
    Parser parser;
    FardriverMessage out{};
    uint32_t now_ms = 20;

    for (std::size_t i = 0; i < 7; ++i) {
        assert(parser.Push(kGoldenZeroToEleven[i], now_ms++, out) == Event::None);
    }
    // A caller may return to its event loop between any two bytes.
    for (std::size_t i = 7; i + 1 < kGoldenZeroToEleven.size(); ++i) {
        assert(parser.Push(kGoldenZeroToEleven[i], now_ms++, out) == Event::None);
    }
    assert(parser.Push(kGoldenZeroToEleven.back(), now_ms++, out) == Event::Frame);
    expect_message(out, kGoldenZeroToEleven);
}

void test_back_to_back_frames() {
    Parser parser;
    FardriverMessage out{};
    uint32_t now_ms = 30;

    assert(push(parser, kGoldenZeroToEleven, out, now_ms) == Event::Frame);
    expect_message(out, kGoldenZeroToEleven);
    assert(push(parser, kGoldenLastStatusId, out, now_ms) == Event::Frame);
    expect_message(out, kGoldenLastStatusId);
}

void test_bad_crc_then_good_frame() {
    Parser parser;
    FardriverMessage out{};
    uint32_t now_ms = 40;
    Bytes corrupt = kGoldenZeroToEleven;
    corrupt[6] ^= 0x01;

    assert(push(parser, corrupt, out, now_ms) == Event::BadCRC);
    assert(push(parser, kGoldenLastStatusId, out, now_ms) == Event::Frame);
    expect_message(out, kGoldenLastStatusId);
}

void test_bad_header_then_good_frame() {
    Parser parser;
    FardriverMessage out{};
    uint32_t now_ms = 50;

    assert(parser.Push(0xAA, now_ms++, out) == Event::None);
    // Status headers require bit 7 and an id in the vendor-observed 0..0x36 range.
    assert(parser.Push(0x7F, now_ms++, out) == Event::BadHeader);
    assert(push(parser, kGoldenZeroToEleven, out, now_ms) == Event::Frame);

    assert(parser.Push(0xAA, now_ms++, out) == Event::None);
    assert(parser.Push(0xB7, now_ms++, out) == Event::BadHeader);
    assert(push(parser, kGoldenLastStatusId, out, now_ms) == Event::Frame);
}

void test_reset_discards_partial_frame() {
    Parser parser;
    FardriverMessage out{};
    uint32_t now_ms = 60;

    for (std::size_t i = 0; i < 9; ++i) {
        assert(parser.Push(kGoldenZeroToEleven[i], now_ms++, out) == Event::None);
    }
    parser.Reset();
    for (std::size_t i = 9; i < kGoldenZeroToEleven.size(); ++i) {
        assert(parser.Push(kGoldenZeroToEleven[i], now_ms++, out) == Event::None);
    }
    assert(push(parser, kGoldenLastStatusId, out, now_ms) == Event::Frame);
    expect_message(out, kGoldenLastStatusId);
}

void test_timeout_discards_partial_frame() {
    Parser parser(50);
    FardriverMessage out{};

    assert(parser.Push(0xAA, 100, out) == Event::None);
    assert(parser.Push(0x80, 101, out) == Event::None);
    assert(parser.Push(0x00, 102, out) == Event::None);
    assert(parser.Tick(151) == Event::None);
    assert(parser.Tick(152) == Event::TimedOut);
    assert(parser.Tick(1000) == Event::None); // timeout is reported once

    uint32_t now_ms = 1001;
    assert(push(parser, kGoldenZeroToEleven, out, now_ms) == Event::Frame);
    expect_message(out, kGoldenZeroToEleven);
}
} // namespace

int main() {
    test_golden_frames();
    test_noise_resynchronization();
    test_split_frame();
    test_back_to_back_frames();
    test_bad_crc_then_good_frame();
    test_bad_header_then_good_frame();
    test_reset_discards_partial_frame();
    test_timeout_discards_partial_frame();
}
