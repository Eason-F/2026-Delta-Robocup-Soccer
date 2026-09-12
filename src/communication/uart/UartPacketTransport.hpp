#pragma once

#include <Arduino.h>

class UartPacketTransport {
    public:
        using PacketHandler = void (*)(void *, const uint8_t *, uint8_t);

        explicit UartPacketTransport(HardwareSerial &serialPort,
                                     uint32_t baudRate = 115200);

        void setup();
        void update();
        bool sendPacket(uint8_t type, const uint8_t *data, uint8_t length);
        bool setPacketHandler(uint8_t type, PacketHandler handler,
                              void *context);

    private:
        static constexpr uint8_t MARKER_0 = 0xA5;
        static constexpr uint8_t MARKER_1 = 0x5A;
        static constexpr uint8_t MAX_PAYLOAD_LENGTH = 64;
        static constexpr uint8_t MAX_PACKET_HANDLERS = 4;

        enum class ReceiveState : uint8_t {
            WAITING_FOR_MARKER_0,
            WAITING_FOR_MARKER_1,
            READING_TYPE,
            READING_LENGTH,
            READING_SEQUENCE,
            READING_PAYLOAD,
            READING_CRC_LOW,
            READING_CRC_HIGH,
        };

        struct HandlerEntry {
            uint8_t type = 0;
            PacketHandler handler = nullptr;
            void *context = nullptr;
        };

        HardwareSerial &serialPort;
        uint32_t baudRate;
        ReceiveState receiveState = ReceiveState::WAITING_FOR_MARKER_0;
        HandlerEntry handlers[MAX_PACKET_HANDLERS] = {};
        uint8_t packetType = 0;
        uint8_t payloadLength = 0;
        uint8_t sequence = 0;
        uint8_t transmitSequence = 0;
        uint8_t payload[MAX_PAYLOAD_LENGTH] = {};
        uint8_t payloadPosition = 0;
        uint16_t calculatedCrc = 0xFFFF;
        uint16_t receivedCrc = 0;

        void processByte(uint8_t value);
        void processPacket();
        void resetReceiver(uint8_t currentByte = 0);
        static uint16_t updateCrc(uint16_t crc, uint8_t value);
};
