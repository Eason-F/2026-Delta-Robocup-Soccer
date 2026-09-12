#include <Arduino.h>

#include <communication/RobotCommunication.hpp>
#include <communication/uart/UartPacketTransport.hpp>
#include <drive/Drive.hpp>
#include <odometry/Odometry.hpp>
#include <ir/uart/UartIRSensor.hpp>
#include <imu/imu.hpp>
#include <util/Logger.hpp>
#include <util/Vector.hpp>
#include <util/util.hpp>
#include <colour/colour.hpp>

class Button {
    private:
        const int buttonPin;

    public:
        Button(const int &pin);
        void setup();

        bool isPressed();
};

enum State {
    SEARCH,
    APPROACH,
    ORBIT,
    CAPTURED
};

class Robot {
    public:
        Robot();

        void setup();
        void run();

    private:
        static constexpr uint8_t LOOP_TIME_MS = 15;
        static constexpr uint16_t LOG_INTERVAL_MS = 100;
        uint8_t packetSequence = 0;
        
        static constexpr uint16_t SEARCH_SPD = 100;
        static constexpr uint16_t APPROACH_SPD = 130;
        
        static constexpr uint16_t ORBIT_APPROACH_SPD = 130;
        static constexpr uint16_t ORBIT_SPD = 150;
        static constexpr uint16_t ORBIT_DISTANCE = 55;
        static constexpr uint16_t ORBIT_ENTRY_TOLERANCE = 20;
        static constexpr uint16_t ORBIT_EXIT_TOLERANCE = 30;
        static constexpr uint16_t ORBIT_DEBOUNCE_MS = 100;
        unsigned long accumulatedOrbitTime = 0;
        
        static constexpr uint16_t CAPTURED_MAX_SPD = 200;
        static constexpr uint16_t CAPTURED_MIN_SPD = 270;
        static constexpr uint16_t ENTER_ALIGNMENT_TOLERANCE = 15;
        static constexpr uint16_t EXIT_ALIGNMENT_TOLERANCE = 30;
        static constexpr uint16_t HEADING_DEADBAND = 7;
        static constexpr uint16_t SPEED_RAMP_MAX_MS = 1000;
        static constexpr uint16_t ALIGNED_DEBOUNCE_MS = 0;
        unsigned long accumulatedAlignedTime = 0;
        
        PIDController approachPID = PIDController(0.5, 0, 0, 0.0, 1.0);
        PIDController orbitTangentPID = PIDController(0.04, 0, 0.001, -1.0, 1.0);
        PIDController orbitDistancePID = PIDController(0.3, 0, 0.001, -0.2, 1.0);
        
        static constexpr uint8_t TURN_SPD = 80;
        static constexpr uint8_t HEADING_TOLERANCE = 15;
        static constexpr uint8_t BALL_TILT_RANGE = 70;
        static constexpr uint8_t BALL_TILT_MAX = 20;
        PIDController headingPID = PIDController(0.01, 0.0, 0.001, -1.0, 1.0);
        
        static constexpr uint8_t BOUNDARY_ESCAPE_SPD = 80;
        static constexpr uint16_t ESCAPE_DURATION = 7;
        static constexpr uint16_t ESCAPE_BUFFER = 10;
        elapsedMillis elapsedEscapeTime = ESCAPE_DURATION;
        float escapeDirection = 0.0f;

        elapsedMicros elapsedLastUpdateTime;
        elapsedMillis elapsedLastLoopTime;
        State robotState = State::SEARCH;
        float targetHeading;

        bool handleEdgeDetection(const float dt);
        void handleHeadingCorrection(const float dt, const float targetHeading);
        void handleTargetHeading();
        void checkRobotState(const float dt, const float targetBallHeading);
        void maneuverAroundBall(const float dt, const float targetBallHeading);

        void sendBluetoothUpdate();

        Button button;
        UartPacketTransport uartTransport;
        UartIRSensor irSensor;
        RobotCommunication robotCommunication;
        Drive drive;
        IMU imu;
        OpticalOdometry odometry;
        ColourSensor colourSensor;
        Logger logger;
};
