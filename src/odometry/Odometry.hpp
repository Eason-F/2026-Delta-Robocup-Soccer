#pragma once

// SparkFun Qwiic OTOS wrapper exposing the robot pose in field coordinates.

#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>

#include <util/util.hpp>
#include <util/Vector.hpp>
#include <util/FieldConstants.hpp>

class OpticalOdometry {
    public:
        OpticalOdometry(TwoWire &wirePort);

        void setup();
        void update();
        void resetPosition();
        void setPosition(sfe_otos_pose2d_t &pose);
        void setFieldPosition(const Position2D &fieldPosition, float heading = 0.0f);
        void boundaryAlignOdometry(const Vector &boundaryVector, const float &heading);

        float getX() const;
        float getY() const;
        float getHeading() const;
        Position2D getPosition() const;

    private:
        TwoWire &wirePort;
        QwiicOTOS odometrySensor;
        sfe_otos_pose2d_t position = {};

        // Converts the sensor's configured metres into calibrated field units.
        static constexpr float LINEAR_MULTIPLIER = 1380.0f;
        static sfe_otos_pose2d_t SENSOR_OFFSET;

        static constexpr float BOUNDARY_CORRECTION_OFFSET = 100.0f;
        static constexpr float BOUNDARY_AXIS_ALIGNMENT_MIN = 0.85f;
};
