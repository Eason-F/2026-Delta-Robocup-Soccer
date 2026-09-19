#include <util/StartingPreset.hpp>

void StartingPreset::setup() {
    for (Input &input : inputs) {
        pinMode(input.pin, PRESSED_LEVEL == LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
        input.rawPressed = input.stablePressed = digitalRead(input.pin) == PRESSED_LEVEL;
        input.changedAt = millis();
    }
}

bool StartingPreset::update(bool running) {
    const uint32_t now = millis();
    bool changed = false;
    if (wasRunning && !running) {
        positionValid = false;
        forceBothAttack = false;
        position = {};
        pending = -1;
        changed = true;
        // A button held during play is not a new reset-period press.
        for (Input &input : inputs) {
            input.rawPressed = input.stablePressed = digitalRead(input.pin) == PRESSED_LEVEL;
            input.changedAt = now;
        }
    }

    wasRunning = running;

    for (uint8_t i = 0; i < 3; ++i) {
        Input &input = inputs[i];
        const bool pressed = digitalRead(input.pin) == PRESSED_LEVEL;
        if (pressed != input.rawPressed) {
            input.rawPressed = pressed;
            input.changedAt = now;
        }
        if (pressed == input.stablePressed || now - input.changedAt < DEBOUNCE_MS) continue;
        input.stablePressed = pressed;
        if (running || !pressed) continue;

        if (pending == i && now - firstPressAt <= DOUBLE_PRESS_MS) {
            select(i, true);
            pending = -1;
            changed = true;
        } else {
            // The newest button supersedes an older pending single press.
            pending = i;
            firstPressAt = now;
        }
    }

    if (pending >= 0 && now - firstPressAt >= DOUBLE_PRESS_MS) {
        select(static_cast<uint8_t>(pending), false);
        pending = -1;
        changed = true;
    }
    return changed;
}

void StartingPreset::select(uint8_t index, bool doublePress) {
    if (index == 0 && doublePress) {
        forceBothAttack = true;
        return;
    }
    forceBothAttack = false;
    positionValid = true;
    if (index == 0) position = {0.0f, -150.0f};
    if (index == 1) position = doublePress ? FieldConstants::friendlyGoalBoxTopLeft
                                         : Position2D{0.0f, -615.0f};
    if (index == 2) position = doublePress ? FieldConstants::friendlyGoalBoxTopRight
                                         : Position2D{0.0f, -815.0f};
}
