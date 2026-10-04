// Debounced digital edge detection for one colour sensor.
#include "ColourModule.hpp"

ColourModule::ColourModule(const int &pin, const float &direction): pin(pin), direction(direction) {}

void ColourModule::setup() {
    pinMode(pin, INPUT_PULLUP);
}

void ColourModule::update() {
    // A high reading refreshes the latch; low readings count it down to zero.
    if (digitalReadFast(pin)) {
        return;
    }
    accumulatedDetectionTime = 0;
}

bool ColourModule::detectedEdge() {
    return accumulatedDetectionTime >= DEBOUNCE_BUFFER_MS;
} 

Vector ColourModule::getVector() {
    return Vector(Vector::AngMag{}, direction, detectedEdge() ? 1.0f : 0.0f);
}
