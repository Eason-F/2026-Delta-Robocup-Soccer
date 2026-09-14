#pragma once

// Stateful PID controller with a clamped output and second-based time delta.

class PIDController {
    private:
        // Controller gains and output bounds.
        const float kP = 0;
        const float kI = 0;
        const float kD = 0;
        const float max = 0.0f;
        const float min = 0.0f;

        // State retained between adjustment calls.
        float value;
        float lastError;
        float integral;

    public:
        PIDController(const float &kP, const float &kI, const float &kD, const float &min, const float &max);

        float adjustmentValue(const float &dt, const float &target, const float &current);
        float adjustmentValue(const float &dt, const float &error);
};
