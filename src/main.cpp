// Arduino entry points and ownership of the single robot controller.
#include <Robot.hpp>

Robot robot;

void setup() {
    Serial.begin(115200);
    robot.setup();

    if (CrashReport) {
        Serial.print(CrashReport);
    }
}

void loop() {
    robot.run();
}
