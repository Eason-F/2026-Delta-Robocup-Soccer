#pragma once

// Bidirectional PWM motor output with quadrature-encoder RPM feedback.

#include <string>
#include <Encoder.h>

#include <util/Logger.hpp>
#include <util/PID.hpp>

class Motor {
    public:
        static constexpr int PULSE_PER_REVOLUTION = 2000;
        static constexpr int MAX_RPM = 250;
        const String name;

        float angularVelocityRPM = 0;

        ~Motor();
        Motor(const String &name, const int &directionPin1, const int &directionPin2,  const int &encoderPin1, const int &encoderPin2, PIDController &pidController);
        void setup();

        float getRPM(const float &dt);

        void setMotorDutyCycle(int speed);
        void setMotorRPM(int rpm, const float &dt);

        void brake();

    private:
        // Limit each PID update to avoid abrupt PWM changes.
        static constexpr int MAX_PWM_CHANGE = 30;
    
        const int DIRECTION_PIN1;
        const int DIRECTION_PIN2;
        const int ENCODER_PIN1;
        const int ENCODER_PIN2;
        float lastInput;
        int lastTarget;

        Encoder *encoder = nullptr;
        PIDController &pidController;
};
