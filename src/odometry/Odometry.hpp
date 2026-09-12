#pragma once

#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>

#include <util/util.hpp>
#include <util/Vector.hpp>

class OpticalOdometry {
    public:
        OpticalOdometry(TwoWire &wirePort);

        void setup();
        void update();
        void resetPosition();
        void setPosition(sfe_otos_pose2d_t &pose);
        void boundaryAlignOdometry(Vector boundaryVector);

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
