#pragma once

// Two-dimensional vector represented in both Cartesian and polar forms.

#include <Arduino.h>

class Vector {
    public:
        float x = 0;
        float y = 0;
        float angle = 0;
        float magnitude = 0;

        // Tag types disambiguate Cartesian and angle/magnitude constructors.
        struct Position {};
        struct AngMag {};

        Vector();
        Vector(Position, const float &posX, const float &posY);
        Vector(AngMag, const float &angle, const float &length);

        Vector operator+(const Vector &vec);
        Vector operator-(const Vector &vec);
        Vector operator*(const float &n);
        Vector operator/(const float &n);

        Vector rotateBy(const float &angle);
};
