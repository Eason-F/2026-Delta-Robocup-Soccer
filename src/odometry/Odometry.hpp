#pragma once

#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>

class OpticalOdometry {
    public:
        OpticalOdometry(TwoWire &wirePort);

        void setup();
        void update();
        void resetPosition();
        void setPosition(sfe_otos_pose2d_t &pose);

        float getX();
        float getY();
        float getHeading();

    private:
        TwoWire &wirePort;
        QwiicOTOS odometrySensor;
        sfe_otos_pose2d_t position;

        static constexpr float LINEAR_MULTIPLIER = -1.9f;
        static sfe_otos_pose2d_t SENSOR_OFFSET;
};
