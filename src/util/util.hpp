#pragma once

// Shared geometry types, angle helpers, and lightweight serial macros.

#include <Arduino.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

#include <util/Vector.hpp>

// Legacy direct logging helpers; prefer Logger for rate-limited telemetry.
#define LOG_NEXT Serial.println();
#define LOG_PRINT(text) Serial.print(text);
#define LOG(header, text) LOG_PRINT(header) LOG_PRINT(": ") LOG_PRINT(text) LOG_PRINT("  | ")

#define conditionallyBreakLoop(bool) if (bool) {return;}

// Field position in millimetres using the coordinate convention in README.md.
struct Position2D {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Position2D() = default;
    constexpr Position2D(float x, float y) : x(x), y(y) {}

    constexpr sfe_otos_pose2d_t toPose2D(float heading) const {
        return {x, y, heading};
    }

    float distanceTo(const Position2D other) const{
        return hypot(abs(x - other.x), abs(y - other.y));
    }

    // Bearing in degrees using the field convention: 0 is +Y and 90 is +X.
    float angleTo(const Position2D other) const {
        return degrees(atan2(other.x - x, other.y - y));
    }

    Position2D operator+(const Vector &vector) const {
        return {x + vector.x, y + vector.y};
    }

    Position2D operator-(const Vector &vector) const {
        return {x - vector.x, y - vector.y};
    }

    Position2D &operator+=(const Vector &vector) {
        x += vector.x;
        y += vector.y;
        return *this;
    }

    Position2D &operator-=(const Vector &vector) {
        x -= vector.x;
        y -= vector.y;
        return *this;
    }
};

namespace util {
    // Wrap a degree angle to the platform remainder range near [-180, 180].
    inline float wrapAngle180(const float angle) {
        return std::remainder(angle, 360.0);
    }

    // Linearly map a value between ranges without constraining the result.
    inline float mapRange(const float value, const float fromMin, const float fromMax, const float toMin, const float toMax) {
        if (fromMin == fromMax) return toMin; 
        
        return toMin + (value - fromMin) * (toMax - toMin) / (fromMax - fromMin);
    }

    // Sigmoid function
    inline float sigmoid(float x) {
        return 1.0f / (1.0f + std::exp(-x));
    }
}
