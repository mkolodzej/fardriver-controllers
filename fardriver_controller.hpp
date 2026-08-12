#pragma once

#include <string.h>
#include <stdio.h>
#include "fardriver.hpp"
#include "fardriver_crc.hpp"
#include "fardriver_message.hpp"

struct FardriverSerial {
    uint32_t (*write)(const uint8_t * data, uint32_t length);
    uint32_t (*read)(uint8_t * data, uint32_t length);
    uint32_t (*available)(void);
};

struct FardriverController {
    FardriverController(FardriverSerial * _serial) : serial(_serial) {

    }

    enum class WriteResult {
        Success,
        InvalidArgument,
        UnsupportedLength,
        TransportFailure,
        UnsupportedSourceLayout
    };

    // A single serial->write() may accept fewer bytes than requested once the
    // transport's TX buffer fills. Abandoning the frame there leaves a torn
    // packet on the wire and the controller waiting for the remainder, so drain
    // the whole buffer and only report failure when the transport stops
    // accepting bytes entirely.
    bool WriteAll(const uint8_t * data, uint32_t length) {
        uint32_t written = 0;
        while (written < length) {
            const uint32_t chunk = serial->write(data + written, length - written);
            if (chunk == 0)
                return false;
            if (chunk > length - written)
                return false;
            written += chunk;
        }
        return true;
    }

    // Frame a payload using the protocol implemented by the vendor app's
    // ConnectPage.WriteAddr. Payload is never modified and writes must complete.
    WriteResult WriteAddr(const uint8_t * payload, uint8_t addr, uint16_t payload_length) {
        if (serial == NULL || serial->write == NULL ||
            (payload == NULL && payload_length != 0))
            return WriteResult::InvalidArgument;
        // The app has explicit encodings for its 312/384-byte flash images.
        // Other writes encode the framed length in one byte (0xC0 + length),
        // which is unambiguous only through 59 payload bytes.
        if (payload_length != 0x180 && payload_length != 0x138 && payload_length > 59)
            return WriteResult::UnsupportedLength;

        uint8_t data[0x180 + 6];
        const uint16_t framed_length = payload_length + 4;
        data[0] = 0xAA; // 170
        if (framed_length == 0x184) {
            // CAN Data, 0x180 bytes long
            // addr is 0
            data[1] = 0xFF; 
        } else if (framed_length == 0x13C) {
            // All params?, 0x138 bytes long
            // addr is 0 or 1
            data[1] = 0xFE;
        } else {
            data[1] = static_cast<uint8_t>(0xC0 + framed_length);
        }
        data[2] = addr;
        data[3] = addr;
        if (payload_length != 0)
            memcpy(data + 4, payload, payload_length);
        uint8_t a = 0x3C; // 60
        uint8_t b = 0x7F; // 127
        for (uint16_t pos = 0; pos < framed_length; ++pos) {
            auto crc_i = a ^ data[pos];
            a = b ^ FardriverMessage::crcTableHi[crc_i];
            b = FardriverMessage::crcTableLo[crc_i];
        }
        data[framed_length] = a;
        data[framed_length + 1] = b;
        const uint32_t wire_length = framed_length + 2;
        return WriteAll(data, wire_length)
            ? WriteResult::Success : WriteResult::TransportFailure;
    }

    WriteResult UpdateWord(uint8_t addr, uint8_t first, uint8_t second) {
        const uint8_t payload[2] = { first, second };
        return WriteAddr(payload, addr, sizeof(payload));
    }

    // used when !App.NewVersion
    WriteResult SendRS232Command(uint8_t command, uint8_t sub_command, uint8_t value_1, uint8_t value_2) {
        if (serial == NULL || serial->write == NULL)
            return WriteResult::InvalidArgument;
        uint8_t data[8];
        data[0] = 0xAA; // 170
        data[1] = command;
        data[2] = ~command;
        data[3] = sub_command;
        data[4] = value_1;
        data[5] = value_2;
        data[6] = data[0] + data[1] + data[2] + data[3] + data[4] + data[5];
        data[7] = ~data[6];
        return WriteAll(data, 8)
            ? WriteResult::Success : WriteResult::TransportFailure;
    }

    WriteResult SendRS323Data(uint8_t command, uint8_t sub_command, uint8_t value_1, uint8_t value_2) {
        return SendRS232Command(command, sub_command, value_1, value_2);
    }

    WriteResult WriteSystemCommand(uint8_t command) {
        const uint8_t payload[2] = { 0x88, command };
        return WriteAddr(payload, 0xA0, sizeof(payload));
    }

    WriteResult WriteSysCmd(uint8_t command) { return WriteSystemCommand(command); }

    // sent immediately after opening port
    // name is a guess
    WriteResult OpenSession(void) {
       return SendRS232Command(0x13, 0x07, 0x01, 0xF1);
    }
    WriteResult Open(void) { return OpenSession(); }

    // Legacy reverse-engineered guess retained for API compatibility.
    WriteResult KeepAlive(void) {
       return SendRS232Command(0x13, 0x07, 0x5F, 0x5F);
    }

    // Observed recurring in MotorcEnglish2026 v1.2.2.1 while its
    // communications state is active. Semantics are not proven universal.
    WriteResult ObservedPcPollExperimental(void) {
       return SendRS232Command(0x05, 0x01, 0x5F, 0x5F);
    }

    WriteResult Reset(void) {
        return WriteSystemCommand(0x5);
    }

    enum EReadError {
        Success = 0,
        NotEnoughBytesAvailable = 1,
        IncorrectMessageStart = 2,
        UnhandleMessageHeader = 3,
        CouldNotVerifyCRC = 4
    };

    struct ReadResult {
        EReadError error;
        // address of the message received, if no error
        uint8_t addr;
    };

    // gets data from a message, if available
    //
    // Prefer FardriverFrameParser/FardriverTelemetryReader in fardriver_stream.hpp
    // for new code; this signature is retained for API compatibility. It now
    // discards leading noise within a single call instead of dropping one byte
    // per call, which previously left the decoder permanently ~15 bytes behind
    // the stream whenever it fell out of sync.
    ReadResult Read(FardriverData * data) {
        if (serial == NULL || serial->available == NULL || serial->read == NULL)
            return { NotEnoughBytesAvailable, 0 };
        bool saw_bad_start = false;
        while (serial->available() >= sizeof(message)) {
            if (serial->read(&message.start, 1) != 1)
                return { NotEnoughBytesAvailable, 0 };
            if (!message.VerifyStart()) {
                saw_bad_start = true;
                continue;
            }

            if (serial->read((uint8_t*)&message.header, 1) != 1)
                return { NotEnoughBytesAvailable, 0 };
            if (message.HeaderFlag() != 2 || message.HeaderId() >= 0x37)
                return { UnhandleMessageHeader, 0 };

            if (serial->read(message.data, sizeof(message.data)) != sizeof(message.data))
                return { NotEnoughBytesAvailable, 0 };
            if (serial->read(message.crc, sizeof(message.crc)) != sizeof(message.crc))
                return { NotEnoughBytesAvailable, 0 };
            if (!message.VerifyCRC())
                return { CouldNotVerifyCRC, 0 };

            uint8_t addr = FardriverMessage::flashReadAddr[message.HeaderId()];
            memcpy(data->GetAddr(addr), &message.data, 12);

            return { Success, addr };
        }
        return { saw_bad_start ? IncorrectMessageStart : NotEnoughBytesAvailable, 0 };
    }

    // The app stores cflash as a separate 384-byte image. FardriverData does
    // not establish that layout, so this legacy signature must fail closed.
    WriteResult SaveCANParameters(FardriverData *) {
        return WriteResult::UnsupportedSourceLayout;
    }

    WriteResult SaveCANParameterImage(const uint8_t * cflash, uint16_t size) {
        if (size != 0x180)
            return WriteResult::UnsupportedLength;
        return WriteAddr(cflash, 0x00, size);
    }

    // saves "wflash"
    WriteResult SaveParameters(FardriverData * fd) {
        if (fd == NULL)
            return WriteResult::InvalidArgument;
        uint8_t data[0x138];
        uint8_t * pos = data;
        uint16_t size = (0x36) * 2;
        memcpy(pos, fd->GetAddr(0x00), size);
        pos += size;
        size = (0x6F - 0x63) * 2;
        memcpy(pos, fd->GetAddr(0x63), size);
        pos += size;
        size = (0xD6 - 0x7C) * 2;
        memcpy(pos, fd->GetAddr(0x7C), size);
        pos += size;
        return WriteAddr(data, 0x01, sizeof(data));
    }
    WriteResult SendDetectPacket(void) {
        if (serial == NULL || serial->write == NULL)
            return WriteResult::InvalidArgument;
        uint8_t message[8];
        message[0] = 0x5A;
        message[1] = 0xAA;
        message[2] = 0x33;
        message[3] = 0x03;
        message[4] = 0x33;
        message[5] = 0x3C;
        message[6] = 0xFD;
        message[7] = 0xFE;
        return WriteAll(message, 8)
            ? WriteResult::Success : WriteResult::TransportFailure;
    }

    WriteResult SendACK(uint8_t index) {
        if (serial == NULL || serial->write == NULL)
            return WriteResult::InvalidArgument;
        uint8_t message[8];
        message[0] = 0x5A;
        message[1] = 0xBB;
        message[2] = index;
        message[3] = 0x72;
        message[4] = 0x73;
        message[5] = 0x74;
        message[6] = 0x75;
        message[7] = 0x76;
        return WriteAll(message, 8)
            ? WriteResult::Success : WriteResult::TransportFailure;
    }

    // can be used for regular packets & crc packet
    //
    // Streams the fixed 2048-byte payload in bounded chunks. An earlier version
    // built the whole 2055-byte frame in one stack buffer, which overflows the
    // default task stack on an ESP32-class target. The CRC is accumulated
    // incrementally so the wire bytes are identical to that version.
    bool SendPacket(uint8_t index, const uint8_t * data, uint32_t length) {
        if (serial == NULL || serial->write == NULL || length > 2048 ||
            (data == NULL && length != 0))
            return false;
        static const uint32_t kPayload = 2048;
        uint8_t header[3];
        header[0] = 0x5A;
        header[1] = 0xA5;
        header[2] = index;
        crc.Reset();
        // The vendor frame CRCs the index byte plus the full padded payload.
        crc.Add(&header[2], 1);
        if (!WriteAll(header, sizeof(header)))
            return false;
        if (length != 0) {
            crc.Add(data, length);
            if (!WriteAll(data, length))
                return false;
        }
        uint8_t padding[64];
        memset(padding, 0xFF, sizeof(padding));
        uint32_t remaining = kPayload - length;
        while (remaining > 0) {
            const uint32_t chunk =
                remaining < sizeof(padding) ? remaining : sizeof(padding);
            crc.Add(padding, chunk);
            if (!WriteAll(padding, chunk))
                return false;
            remaining -= chunk;
        }
        uint8_t trailer[4];
        crc.Assign(trailer);
        return WriteAll(trailer, sizeof(trailer));
    }

    enum class CRCMessageResult {
        Success,
        InvalidArgument,
        Timeout,
        ShortRead,
        InvalidHeader,
        ChecksumMismatch,
        IndexMismatch,
        CRCMismatch
    };

    CRCMessageResult VerifyCRCMessage(uint8_t index, const uint8_t * file_crc,
                                      uint32_t max_availability_polls = 1) {
        if (serial == NULL || serial->available == NULL || serial->read == NULL || file_crc == NULL)
            return CRCMessageResult::InvalidArgument;
        // wait for 0xaa 0x1f <error> <index> <packet_crc[8]> <crc[2]>
        // no error if error < 0x7E && error == index
        //
        // NOTE: this spins without yielding or sleeping, so it is not a real
        // timeout — a bare loop retires in nanoseconds and cannot outlast the
        // firmware's response latency. Callers that need to wait must either
        // pre-buffer the 16-byte reply or supply a blocking/yielding
        // serial->available(); max_availability_polls only bounds the spin.
        uint32_t polls = 0;
        while (serial->available() < 16 && polls < max_availability_polls)
            ++polls;
        if (serial->available() < 16)
            return CRCMessageResult::Timeout;
        uint8_t message[16] = { 0 };
        if (serial->read(message, sizeof(message)) != sizeof(message))
            return CRCMessageResult::ShortRead;
        if (message[0] != 0xAA || message[1] != 0x1F)
            return CRCMessageResult::InvalidHeader;
        uint16_t checksum = 0;
        for (uint8_t i = 0; i < 14; ++i)
            checksum = static_cast<uint16_t>(checksum + message[i]);
        if (message[14] != static_cast<uint8_t>(checksum >> 8) ||
            message[15] != static_cast<uint8_t>(checksum & 0xFF))
            return CRCMessageResult::ChecksumMismatch;
        // The vendor app accepts this firmware response only when bytes 2 and
        // 3 agree, then switches on that duplicated result/index value.
        if (message[2] != message[3] || message[3] != index)
            return CRCMessageResult::IndexMismatch;
        const uint8_t * read_crc = &message[4];
        for (uint8_t i = 0; i < 8; i++) {
            if (file_crc[i] != read_crc[i]) {
                return CRCMessageResult::CRCMismatch;
            }
        }
        return CRCMessageResult::Success;
    }

    CRC crc;
    FardriverSerial * serial;
    FardriverMessage message;
};
