#pragma once

#include <Arduino.h>
#include <util/FieldConstants.hpp>

class StartingPreset {
    public:
        static constexpr uint32_t DOUBLE_PRESS_MS = 500;
        static constexpr uint32_t DEBOUNCE_MS = 25;
        static constexpr uint8_t PRESSED_LEVEL = LOW;

        void setup();
        // True when the preset or game mode changes.
        bool update(bool running);

        bool hasPosition() const { return positionValid; }
        bool bothAttack() const { return forceBothAttack; }
        bool hasPendingPress() const { return pendingInputIndex >= 0; }
        Position2D getPosition() const { return position; }

    private:
        static constexpr uint8_t INPUT_COUNT = 3;
        static constexpr int8_t NO_PENDING_INPUT = -1;

        struct Input {
            uint8_t pin;
            bool rawPressed = false;
            bool stablePressed = false;
            uint32_t rawStateChangedAt = 0;
        };

        Input inputs[INPUT_COUNT] = {{12}, {13}, {14}};
        bool wasRunning = false;
        bool positionValid = false;
        bool forceBothAttack = false;
        Position2D position = {};
        int8_t pendingInputIndex = NO_PENDING_INPUT;
        uint32_t firstPressAt = 0;

        bool readNewPress(Input &input, uint32_t now);
        bool recordPress(uint8_t inputIndex, uint32_t now);
        bool finishPendingSinglePress(uint32_t now);
        void resetAfterRun(uint32_t now);
        void selectPreset(uint8_t inputIndex, bool doublePress);
};
