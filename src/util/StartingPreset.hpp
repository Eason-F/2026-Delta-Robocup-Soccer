#pragma once

#include <Arduino.h>
#include <util/FieldConstants.hpp>

// Preset input is accepted only while the start switch is off.
class StartingPreset {
public:
    static constexpr uint32_t DOUBLE_PRESS_MS = 500;
    static constexpr uint32_t DEBOUNCE_MS = 25;
    static constexpr uint8_t PRESSED_LEVEL = LOW;

    void setup();
    // Returns true when the selection changes (including reset to null).
    bool update(bool running);
    bool hasPosition() const { return positionValid; }
    bool bothAttack() const { return forceBothAttack; }
    bool hasPendingPress() const { return pending >= 0; }
    Position2D getPosition() const { return position; }

private:
    struct Input {
        uint8_t pin;
        bool rawPressed = false;
        bool stablePressed = false;
        uint32_t changedAt = 0;
    };
    Input inputs[3] = {{12}, {13}, {14}};
    bool wasRunning = false;
    bool positionValid = false;
    bool forceBothAttack = false;
    Position2D position;
    int8_t pending = -1;
    uint32_t firstPressAt = 0;

    void select(uint8_t index, bool doublePress);
};
