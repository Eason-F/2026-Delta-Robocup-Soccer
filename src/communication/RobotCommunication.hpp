#pragma once

#include <Arduino.h>
#include <communication/uart/UartPacketTransport.hpp>

#pragma pack(push, 1)

struct RobotPacket {
    int16_t x;
    int16_t y;
    int16_t heading;
    int16_t ballBearing;
    uint8_t ballStrength;
    uint8_t attackScore;
    uint8_t state;
    uint8_t role;
    uint8_t flags;
    uint8_t sequence;
};

#pragma pack(pop)

static_assert(sizeof(RobotPacket) == 14,
              "RobotPacket wire format must be exactly 14 bytes");

class RobotCommunication {
    public:
        explicit RobotCommunication(UartPacketTransport &transport);

        void setup();
        bool sendPacket(const RobotPacket &packet);
        bool hasReceivedPacket() const;
        const RobotPacket &getReceivedPacket() const;
        uint32_t getLastUpdateMillis() const;

    private:
        static constexpr uint8_t ESP_NOW_TO_TEENSY_TYPE = 0x02;
        static constexpr uint8_t TEENSY_TO_ESP_NOW_TYPE = 0x03;

        UartPacketTransport &transport;
        RobotPacket receivedPacket = {};
        bool receivedPacketValid = false;
        uint32_t lastUpdateMillis = 0;

        void processPacket(const uint8_t *data, uint8_t length);
        static void handlePacket(void *context, const uint8_t *data,
                                 uint8_t length);
};
