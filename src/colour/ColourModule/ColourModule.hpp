#pragma once

// One digital boundary sensor with a short release debounce window.

#include <Arduino.h>
#include <util/Vector.hpp>

class ColourModule {
    public:
        ColourModule(const int &pin, const float &direction);
        void setup();
        void update();
        bool detectedEdge();
        Vector getVector();

    private:
        const int pin;
        const float direction;
        
        static constexpr uint8_t DEBOUNCE_BUFFER_MS = 10;
        elapsedMillis accumulatedDetectionTime = DEBOUNCE_BUFFER_MS;
};
