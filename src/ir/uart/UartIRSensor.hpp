#pragma once

// Ball bearing and signal-strength decoder for UART measurement packets.

#include <Arduino.h>
#include <communication/uart/UartPacketTransport.hpp>
#include <util/util.hpp>

class UartIRSensor {
    public:
        explicit UartIRSensor(UartPacketTransport &transport);

        void setup();

        float getDirectionDegrees() const;
        float getDirectionRadians() const;
        float getSignalStrength() const;
        bool ballFound() const;
        uint32_t getLastUpdateMillis() const;

        static float strengthToDistance(const uint16_t &strength);

    private:
        // Payload: signed little-endian centidegrees, then unsigned strength.
        static constexpr uint8_t IR_MEASUREMENT_TYPE = 0x01;
        static constexpr uint8_t IR_PAYLOAD_LENGTH = 4;

        UartPacketTransport &transport;
        float directionDegrees = 0.0f;
        float signalStrength = 0.0f;
        bool readingValid = false;
        uint32_t lastUpdateMillis = 0;

        void processPacket(const uint8_t *data, uint8_t length);
        static void handlePacket(void *context, const uint8_t *data,
                                 uint8_t length);
        static uint16_t readU16(const uint8_t *data);
};
