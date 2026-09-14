// Vector construction, arithmetic, and degree-based rotation.
#include <util/Vector.hpp>

Vector::Vector() = default;

Vector::Vector(Position, const float &posX, const float &posY)
    : x(posX),
      y(posY),
      angle(atan2(posY, posX)),
      magnitude(sqrt(posX * posX + posY * posY)) {
}

Vector::Vector(AngMag, const float &angle, const float &length)
    : x(length * cos(angle)),
      y(length * sin(angle)),
      angle(angle),
      magnitude(length) {
}

Vector Vector::operator+(const Vector &vec) {
    return Vector(Position {}, x + vec.x, y + vec.y);
}

Vector Vector::operator-(const Vector &vec) {
    return Vector(Position {}, x - vec.x, y - vec.y);
}

Vector Vector::operator*(const float &n) {
    return Vector(Position {}, x * n, y * n);
}

Vector Vector::operator/(const float &n) {
    return Vector(Position {}, x / n, y / n);
}

Vector Vector::rotateBy(const float &angle) {
    float radians = angle * DEG_TO_RAD;
    float cosAngle = cos(radians);
    float sinAngle = sin(radians);

    return Vector(Position {}, x * cosAngle - y * sinAngle, x * sinAngle + y * cosAngle);
}
