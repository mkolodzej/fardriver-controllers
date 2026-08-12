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
        return serial->write(data, wire_length) == wire_length
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
        return serial->write(data, 8) == 8
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
    ReadResult Read(FardriverData * data) {
        if (serial->available() < sizeof(message))
            return { NotEnoughBytesAvailable, 0 };
        
        serial->read(&message.start, 1);
        if (!message.VerifyStart())
            return { IncorrectMessageStart, 0 };

        serial->read((uint8_t*)&message.header, 1);
        if (message.header.flag != 2 || message.header.id >= 0x37)
            return { UnhandleMessageHeader, 0 };

        serial->read(message.data, sizeof(message.data));
        serial->read(message.crc, sizeof(message.crc));
        if (!message.VerifyCRC())
            return { CouldNotVerifyCRC, 0 };

        uint8_t addr = FardriverMessage::flashReadAddr[message.header.id];
        memcpy(data->GetAddr(addr), &message.data, 12);

        return { Success, addr };
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
        return serial->write(message, 8) == 8
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
        return serial->write(message, 8) == 8
            ? WriteResult::Success : WriteResult::TransportFailure;
    }

    // can be used for regular packets & crc packet
    bool SendPacket(uint8_t index, const uint8_t * data, uint32_t length) {
        if (serial == NULL || serial->write == NULL || length > 2048 ||
            (data == NULL && length != 0))
            return false;
        uint8_t message[2048 + 3 + 4];
        message[0] = 0x5A;
        message[1] = 0xA5;
        message[2] = index;
        printf("  Copying packet data\n");
        memcpy(&message[3], data, length);
        if (2048 - length > 0) {
            memset(&message[3 + length], 0xFF, 2048 - length);
        }
        crc.Reset();
        crc.Add(&message[2], 1 + 2048);
        crc.Assign(&message[3 + 2048]);
        printf("  Writing message\n");
        return serial->write(message, sizeof(message)) == sizeof(message);
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
