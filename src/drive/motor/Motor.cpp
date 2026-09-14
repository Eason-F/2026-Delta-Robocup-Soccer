// Motor driver output, encoder sampling, and incremental PID control.
#include <Arduino.h>

#include <drive/motor/Motor.hpp>

Motor::~Motor() {
    delete encoder;
}

Motor::Motor(const String &name, const int &directionPin1, const int &directionPin2,  const int &encoderPin1, const int &encoderPin2, PIDController &pidController) :
    name(name),
    DIRECTION_PIN1(directionPin1),
    DIRECTION_PIN2(directionPin2),
    ENCODER_PIN1(encoderPin1),
    ENCODER_PIN2(encoderPin2),
    encoder(nullptr),
    pidController(pidController) {}

void Motor::setup() {
    pinMode(DIRECTION_PIN1, OUTPUT);
    pinMode(DIRECTION_PIN2, OUTPUT);
    analogWriteFrequency(DIRECTION_PIN1, 20000);
    analogWriteFrequency(DIRECTION_PIN2, 20000);

    if ((ENCODER_PIN1 > 0 && ENCODER_PIN2 > 0) && (encoder == nullptr)) {
        encoder = new Encoder(ENCODER_PIN1, ENCODER_PIN2);
    }
    brake();
}

float Motor::getRPM(const float &dt) {
    if (encoder == nullptr || dt <= 0.0f)
        return 0.0f;

    long delta = encoder-> readAndReset();
    angularVelocityRPM = (delta / (float)PULSE_PER_REVOLUTION) * (60.0 / dt);
    return angularVelocityRPM;
};

void Motor::brake() {
    analogWrite(DIRECTION_PIN1, 255);
    analogWrite(DIRECTION_PIN2, 255);
    lastInput = 0;
}

void Motor::setMotorDutyCycle(int speed) {
    // Convert the signed -100..100 command into complementary driver PWM.
    int motorSpeed = abs((speed / 100.0) * 255);
    if (speed > 0) {
        analogWrite(DIRECTION_PIN1, motorSpeed);
        analogWrite(DIRECTION_PIN2, 0);
    } else if (speed < 0) {
        analogWrite(DIRECTION_PIN1, 0);
        analogWrite(DIRECTION_PIN2, motorSpeed);
    } else {
        brake();
    }
}

void Motor::setMotorRPM(int rpm, const float &dt) {
    // Treat the bounded PID output as a change to the accumulated PWM command.
    float currentRPM = getRPM(dt);
    float pidOutput = constrain(pidController.adjustmentValue(dt, rpm, currentRPM), -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
    lastInput += pidOutput;
    lastInput = constrain(lastInput, -255.0f, 255.0f);

    if (lastInput < 0) {
        analogWrite(DIRECTION_PIN1, (int)abs(lastInput));
        analogWrite(DIRECTION_PIN2, 0);
    } else if (lastInput > 0) {
        analogWrite(DIRECTION_PIN1, 0);
        analogWrite(DIRECTION_PIN2, (int)lastInput);
    } else {
        brake();
    }
}
