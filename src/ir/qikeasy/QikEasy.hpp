#pragma once

// Legacy dual-module I2C infrared ball-sensor interface.

#include <Wire.h>
#include <util/Vector.hpp>

class QikEasy {
    public:
        QikEasy(TwoWire &wirePort);
        void setup();

        void updateReadings();
        float getDirectionDegrees();
        float getSignalStrength();
        bool ballFound();
        static float strengthToDistance(const uint16_t &strength);

    private:
        TwoWire &wirePort;

        // QikEasy device addresses and measurement registers.
        static constexpr uint8_t MODULE_ADDRESSES[2] = {0x14, 0x15};
        static constexpr uint8_t RECEIVER_REGISTERS[5] = {0x4A, 0x4C, 0x4E, 0x50, 0x52};
        static constexpr uint8_t DIRECTION_REGISTER = 0x49;
        static constexpr uint8_t STRENGTH_REGISTER = 0x54;

        uint16_t signalStrength(const int &receiverID);
        Vector readMeasuredSignal(const uint8_t &moduleIndex);
        Vector qikeasyReadings();
        Vector vectorReadings();

        // Number of strongest receiver vectors combined into the result.
        static constexpr uint8_t AVERAGED_VECTOR_MAX = 3;
        Vector signalVec;
        Vector allVecs[10];
};
