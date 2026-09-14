#pragma once

// Four-wheel holonomic drive mixing and closed-loop position control.

#include <drive/motor/Motor.hpp>
#include <odometry/Odometry.hpp>
#include <util/PID.hpp>
#include <util/util.hpp>

#define ENCODER_USE_INTERRUPTS

class Drive {
    public:
        // Per-wheel speed controller tuning.
        static constexpr float motorKP = 0.3f;
        static constexpr float motorKI = 0.001f;
        static constexpr float motorKD = 0.0f;
        static constexpr float motorMax = 255.0f;
        static constexpr float motorMin = -255.0f;

        PIDController motorPID1 = PIDController(motorKP, motorKI, motorKD, motorMin, motorMax);
        PIDController motorPID2 = PIDController(motorKP, motorKI, motorKD, motorMin, motorMax);
        PIDController motorPID3 = PIDController(motorKP, motorKI, motorKD, motorMin, motorMax);
        PIDController motorPID4 = PIDController(motorKP, motorKI, motorKD, motorMin, motorMax);

        // Field-position controller tuning.
        static constexpr float positionKP = 0.01f;
        static constexpr float positionKI = 0.0f;
        static constexpr float positionKD = 0.0f;
        static constexpr float positionMax = 1.0f;
        static constexpr float positionMin = -1.0f;
        PIDController positionPIDX = PIDController(positionKP, positionKI, positionKD, positionMin, positionMax);
        PIDController positionPIDY = PIDController(positionKP, positionKI, positionKD, positionMin, positionMax);

        // Motors are public to support direct hardware testing and telemetry.
        Motor motor1;
        Motor motor2;
        Motor motor3;
        Motor motor4;

        Drive();
        void setup();

        void updateRPM(const float &dt);
        
        void moveInDirection(const float &dt, int directionDegrees, int rpm);
        void moveToPoint(const float &dt, const int &rpm, const Position2D &target, OpticalOdometry &odometry);
        void moveToPoint(const float &dt, const int &rpm, const float &targetX, const float &targetY, OpticalOdometry &odometry);
        void turnInDirection(const float &dt, int rpm);
        void stop();

        float lastDirection = 0;
        float rotationRpm = 0;

    private: 
        // Motor driver and quadrature encoder pins, grouped by wheel.
        static constexpr int DIRECTION_PIN1_1 = 2;
        static constexpr int DIRECTION_PIN2_1 = 3;
        static constexpr int ENCODER_PIN1_1 = 33;
        static constexpr int ENCODER_PIN2_1 = 34;

        static constexpr int DIRECTION_PIN1_2 = 4;
        static constexpr int DIRECTION_PIN2_2 = 5;
        static constexpr int ENCODER_PIN1_2 = 35;
        static constexpr int ENCODER_PIN2_2 = 36;

        static constexpr int DIRECTION_PIN1_3 = 6;
        static constexpr int DIRECTION_PIN2_3 = 7;
        static constexpr int ENCODER_PIN1_3 = 37;
        static constexpr int ENCODER_PIN2_3 = 38;

        static constexpr int DIRECTION_PIN1_4 = 8;
        static constexpr int DIRECTION_PIN2_4 = 9;
        static constexpr int ENCODER_PIN1_4 = 39;
        static constexpr int ENCODER_PIN2_4 = 40;
};
