#pragma once

// Top-level robot controller, strategy state machine, and hardware ownership.
#include <Arduino.h>

#include <strategy/strategy.hpp>

#include <communication/RobotCommunication.hpp>
#include <communication/uart/UartPacketTransport.hpp>

#include <drive/Drive.hpp>

#include <odometry/Odometry.hpp>
#include <ir/uart/UartIRSensor.hpp>
#include <colour/colour.hpp>
#include <imu/imu.hpp>

#include <util/Logger.hpp>
#include <util/Vector.hpp>
#include <util/util.hpp>
#include <util/FieldConstants.hpp>

// Active-low start button configured with the Teensy's internal pull-up.
class Button {
    private:
        const int buttonPin;

    public:
        Button(const int &pin);
        void setup();

        bool isPressed();
};

class Robot {
    friend class Strategy;

    public:
        Robot();

        void setup();
        void run();

    private:
        // Scheduler and telemetry timing.
        static constexpr uint8_t LOOP_TIME_MS = 15;
        static constexpr uint16_t LOG_INTERVAL_MS = 100;
        uint8_t packetSequence = 0;
        
        // Heading controller and ball-dependent heading offset.
        static constexpr uint8_t TURN_SPD = 80;
        static constexpr uint8_t HEADING_TOLERANCE = 15;
        static constexpr uint8_t BALL_TILT_RANGE = 70;
        static constexpr uint8_t BALL_TILT_MAX = 20;
        PIDController headingPID = PIDController(0.01, 0.0, 0.001, -1.0, 1.0);
        
        // Boundary escape timing and last safe direction.
        static constexpr uint8_t BOUNDARY_ESCAPE_SPD = 80;
        static constexpr uint16_t ESCAPE_DURATION = 7;
        static constexpr uint16_t ESCAPE_BUFFER = 10;
        elapsedMillis elapsedEscapeTime = ESCAPE_DURATION;
        float escapeDirection = 0.0f;

        // Runtime state and loop clocks.
        elapsedMicros elapsedLastUpdateTime;
        elapsedMillis elapsedLastLoopTime;
        float targetHeading;

        bool handleEdgeDetection(const float dt);
        void handleHeadingCorrection(const float dt, const float targetHeading);
        void handleTargetHeading();

        void sendBluetoothUpdate();

        // Hardware and service modules.
        Button button;
        UartPacketTransport uartTransport;
        UartIRSensor irSensor;
        RobotCommunication robotCommunication;
        Drive drive;
        IMU imu;
        OpticalOdometry odometry;
        ColourSensor colourSensor;
        Logger logger;

        // Strategy control
        Strategy strategy;
};
