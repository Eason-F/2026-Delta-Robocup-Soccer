#pragma once

// BNO055 yaw reader with a resettable robot-relative heading origin.

#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <Wire.h>
#include <utility/imumaths.h>

class IMU {
    public:
        IMU(TwoWire &wirePort);

        bool setup();
        void update();

        float getYaw();
        float getRelativeYaw();
        void resetYawOrigin();

    private:
        static constexpr int32_t SENSOR_ID = 55;
        static constexpr uint32_t I2C_CLOCK = 400000;

        TwoWire &wirePort;
        Adafruit_BNO055 bno;

        // Headings are normalised degrees in the range [-180, 180].
        float yaw, yawOrigin;
        static float normaliseYaw(float value);
};
