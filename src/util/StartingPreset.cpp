#include <util/StartingPreset.hpp>

void StartingPreset::setup() {
    for (Input &input : inputs) {
        pinMode(input.pin, PRESSED_LEVEL == LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
        const bool pressed = digitalRead(input.pin) == PRESSED_LEVEL;
        input.rawPressed = pressed;
        input.stablePressed = pressed;
        input.rawStateChangedAt = millis();
    }
}

bool StartingPreset::update(bool running) {
    const uint32_t now = millis();
    bool selectionChanged = false;

    // Clear the old selection when a run ends.
    if (wasRunning && !running) {
        resetAfterRun(now);
        selectionChanged = true;
    }
    wasRunning = running;

    // Read each button and accept new presses only while stopped.
    for (uint8_t inputIndex = 0; inputIndex < INPUT_COUNT; ++inputIndex) {
        if (!readNewPress(inputs[inputIndex], now) || running) {
            continue;
        }

        selectionChanged |= recordPress(inputIndex, now);
    }

    // Wait for the double-press window before accepting a single press.
    selectionChanged |= finishPendingSinglePress(now);
    return selectionChanged;
}

bool StartingPreset::readNewPress(Input &input, uint32_t now) {
    const bool pressed = digitalRead(input.pin) == PRESSED_LEVEL;

    if (pressed != input.rawPressed) {
        input.rawPressed = pressed;
        input.rawStateChangedAt = now;
    }

    const bool debounceComplete = now - input.rawStateChangedAt >= DEBOUNCE_MS;
    if (pressed == input.stablePressed || !debounceComplete) {
        return false;
    }

    input.stablePressed = pressed;
    return pressed;
}

bool StartingPreset::recordPress(uint8_t inputIndex, uint32_t now) {
    const bool isDoublePress = pendingInputIndex == inputIndex &&
        now - firstPressAt <= DOUBLE_PRESS_MS;

    if (isDoublePress) {
        selectPreset(inputIndex, true);
        pendingInputIndex = NO_PENDING_INPUT;
        return true;
    }

    // The newest button replaces an older pending press.
    pendingInputIndex = inputIndex;
    firstPressAt = now;
    return false;
}

bool StartingPreset::finishPendingSinglePress(uint32_t now) {
    if (pendingInputIndex == NO_PENDING_INPUT ||
        now - firstPressAt < DOUBLE_PRESS_MS) {
        return false;
    }

    selectPreset(static_cast<uint8_t>(pendingInputIndex), false);
    pendingInputIndex = NO_PENDING_INPUT;
    return true;
}

void StartingPreset::resetAfterRun(uint32_t now) {
    positionValid = false;
    forceBothAttack = false;
    position = {};
    pendingInputIndex = NO_PENDING_INPUT;

    // A held button must be released before it can select a preset.
    for (Input &input : inputs) {
        const bool pressed = digitalRead(input.pin) == PRESSED_LEVEL;
        input.rawPressed = pressed;
        input.stablePressed = pressed;
        input.rawStateChangedAt = now;
    }
}

void StartingPreset::selectPreset(uint8_t inputIndex, bool doublePress) {
    if (inputIndex == 0 && doublePress) {
        // Keep the current position and make both robots attack.
        forceBothAttack = true;
        return;
    }

    forceBothAttack = false;
    positionValid = true;

    switch (inputIndex) {
        case 0:
            position = {0.0f, -150.0f};
            break;
        case 1:
            position = doublePress ? FieldConstants::friendlyGoalBoxTopLeft
                                   : Position2D{0.0f, -615.0f};
            break;
        case 2:
            position = doublePress ? FieldConstants::friendlyGoalBoxTopRight
                                   : Position2D{0.0f, -815.0f};
            break;
    }
}
