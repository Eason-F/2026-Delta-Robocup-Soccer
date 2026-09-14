// Boundary detection and vector combination for the four sensor directions.
#include "colour.hpp"

ColourSensor::ColourSensor(const int &pinFront, const int &pinRight, const int &pinBack, const int &pinLeft) :
    front(pinFront, 0.0f),
    right(pinRight, PI / 2.0f),
    back(pinBack, PI),
    left(pinLeft, 3.0f * PI / 2.0f) {}

void ColourSensor::setup() {
    front.setup();
    right.setup();
    back.setup();
    left.setup();
}

void ColourSensor::update(long elapsedMillis) {
    front.update(elapsedMillis);
    right.update(elapsedMillis);
    back.update(elapsedMillis);
    left.update(elapsedMillis);
}

bool ColourSensor::detectedEdge() {
    return front.detectedEdge()
        || right.detectedEdge()
        || back.detectedEdge()
        || left.detectedEdge();
}

Vector ColourSensor::getVector() {
    // Vector addition points toward the weighted centre of active sensors.
    return front.getVector()
        + right.getVector()
        + back.getVector()
        + left.getVector();
}

float ColourSensor::getDirectionDegrees() {
    if (!detectedEdge()) {
        return -1.0f;
    }

    return degrees(getVector().angle);
}
