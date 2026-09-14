// PID error integration, differentiation, and output limiting.
#include <Arduino.h>

#include <util/PID.hpp>

PIDController::PIDController(const float &kP, const float &kI, const float &kD, const float &min, const float &max) :
    kP(kP), kI(kI), kD(kD),
    max(max), min(min) {}

float PIDController::adjustmentValue(const float &dt, const float &target, const float &current) {
    return adjustmentValue(dt, target - current);
}

float PIDController::adjustmentValue(const float &dt, const float &error) {
    float derivative = (error - lastError) / dt;
    integral += error * dt;
    value = kP * error + kI * integral + kD * derivative;
    value = constrain(value, min, max);
    lastError = error;
    return value;
}
