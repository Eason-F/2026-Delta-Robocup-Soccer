#pragma once

#include <Arduino.h>
#include <util/PID.hpp>
#include <util/FieldConstants.hpp>

class Robot;

class Strategy {
    public:
        // tracking state relating to the attacking role
        enum class TrackingStage : uint8_t {
            SEARCH,
            APPROACH,
            ORBIT,
            TRANSITION,
            CAPTURED
        };

        // defence state relating to defending role
        enum class DefenceStage : uint8_t {
            PASSIVE,
            RETURN, 
            SHUFFLE,
        };

        enum class Role : uint8_t {
            ATTACK,
            DEFENCE
        };

        explicit Strategy(Robot &robot);

        void update();
        void configureGame(bool running, bool hasStartingPosition, bool forceBothAttack);
        Role getRole() const;
        uint8_t calculateAttackScore(); // returns 0-255; higher means more suitable to attack
        uint8_t getCommunicationFlags() const;
        void attack(const float dt);
        void defend(const float dt);

        // attack related
        TrackingStage getTrackingStage() const;
        void checkTrackingStage(const float dt, const float targetBallHeading);
        void maneuverAroundBall(const float dt, const float targetBallHeading);
        float calculateAngleToGoal() const;
        void pushCapturedBallToGoal(const float dt);

        
        // defence related
        DefenceStage getDefenceStage() const;
        void checkDefenceStage();
        void returnToHome(const float dt);
        void goalBallTrack(const float dt); // shuffle around the goal while following the ball


    private:
        Robot &robot;
        Role role = Role::ATTACK;
        TrackingStage trackingStage = TrackingStage::SEARCH;
        DefenceStage defenceStage = DefenceStage::PASSIVE;

        struct ScoreConfigs { // DEPRECATED
            // Each component has a bounded influence so millimetres, degrees,
            // and raw IR strength cannot accidentally dominate one another.
            static constexpr float BALL_STRENGTH_FULL_SCALE = 180.0f;
            static constexpr float BALL_STRENGTH_WEIGHT = 100.0f;
            static constexpr float BALL_ALIGNMENT_WEIGHT = 50.0f;

            static constexpr float DEFENCE_NOT_READY_PENALTY = 30.0f;
            static constexpr float DEFENCE_POSITION_PENALTY_MAX = 40.0f;
            static constexpr float DEFENCE_POSITION_FULL_SCALE = 900.0f;
            static constexpr float DEFENCE_IN_RANGE_BONUS = 40.0f;

            static constexpr float RETAIN_ATTACK_BIAS = 15.0f;
            static constexpr float ATTACK_OFFSIDE_PENALTY = 40.0f;
            static constexpr float ATTACK_SIGNAL_BONUS_MAX = 25.0f;
            static constexpr float ATTACK_SIGNAL_BONUS_START = 130.0f;

            static constexpr float OUT_OF_RESPONSE_ANGLE = 140.0f;
            static constexpr float OUT_OF_RESPONSE_STRENGTH = 40.0f;

            static constexpr uint16_t IR_READING_TIMEOUT_MS = 250;
            static constexpr uint16_t COMMUNICATION_TIMEOUT_MS = 500;
        };

        // Keep the 14-byte packet: five status bits and a three-bit handoff epoch.
        static constexpr uint8_t ROLE_READY_FLAG = 0x01;
        static constexpr uint8_t FRESH_BALL_FLAG = 0x02;
        static constexpr uint8_t POSITION_VALID_FLAG = 0x04;
        static constexpr uint8_t BOTH_ATTACK_FLAG = 0x08;
        static constexpr uint8_t RUNNING_FLAG = 0x10;
        static constexpr uint8_t EPOCH_SHIFT = 5;
        static constexpr uint8_t EPOCH_MASK = 0x07;
        bool rolesInitialized = false;
        bool standaloneAttack = false;
        bool gameplayActive = false;
        bool startingPositionValid = false;
        bool localBothAttack = false;
        bool bothAttackLatched = false;
        uint8_t roleEpoch = 0;

        struct AttackConfig {
            // Positions where goal tilt aims
            static constexpr Position2D GOAL_AIM_LEFT = {
                FieldConstants::opponentGoalLeftPost.x + 70.0f,
                FieldConstants::opponentGoalLeftPost.y
            };
            static constexpr Position2D GOAL_AIM_RIGHT = {
                FieldConstants::opponentGoalRightPost.x - 70.0f,
                FieldConstants::opponentGoalRightPost.y
            };

            // Search and approach tuning (motor targets are in RPM).
            static constexpr uint16_t SEARCH_SPD = 270;
            static constexpr uint16_t APPROACH_SPD = 130;
            static constexpr uint16_t TRANSITION_MIN_MS = 300;
            static constexpr uint16_t TRANSITION_TIMEOUT = 700;

            // Orbit controller tuning and transition hysteresis.
            static constexpr uint16_t ORBIT_APPROACH_SPD = 130;
            static constexpr uint16_t ORBIT_SPD = 120;
            static constexpr uint16_t ORBIT_DISTANCE = 50;
            static constexpr uint16_t ORBIT_ENTRY_TOLERANCE = 20;
            static constexpr uint16_t ORBIT_EXIT_TOLERANCE = 30;
            static constexpr uint16_t ORBIT_DEBOUNCE_MS = 100;

            // Once aligned, charge through the final gap to secure the ball.
            static constexpr uint16_t TRANSITION_SPD = 220;
            static constexpr uint16_t CAPTURED_DISTANCE = 130;
            static constexpr uint16_t CAPTURED_EXIT_DISTANCE = 110;

            // Captured-ball alignment and forward-speed ramp.
            static constexpr uint16_t CAPTURED_MAX_SPD = 270;
            static constexpr uint16_t CAPTURED_MIN_SPD = 200;
            static constexpr uint16_t ENTER_ALIGNMENT_TOLERANCE = 15;
            static constexpr uint16_t EXIT_ALIGNMENT_TOLERANCE = 30;
            static constexpr uint16_t HEADING_DEADBAND = 7;
            static constexpr float GOAL_ALIGNMENT_FULL_SPEED_DEG = 60.0f;
            static constexpr float GOAL_ALIGNMENT_MIN_SPEED_FACTOR = 0.25f;
            static constexpr uint16_t SPEED_RAMP_MAX_MS = 1000;
            static constexpr uint16_t ALIGNED_DEBOUNCE_MS = 0;
        };

        elapsedMillis transitionTime;
        elapsedMillis orbitDebounceTime;
        elapsedMillis alignedTime;
        Position2D capturedGoalTarget = AttackConfig::GOAL_AIM_LEFT;
        bool capturedGoalTargetLocked = false;

        void transitionToTrackingStage(TrackingStage nextStage);
        
        PIDController approachPID = PIDController(0.5, 0, 0, 0.0, 1.0);
        PIDController orbitTangentPID = PIDController(0.04, 0, 0.001, -1.0, 1.0);
        PIDController orbitDistancePID = PIDController(0.3, 0, 0.001, -0.2, 1.0);

        struct DefenceConfig {
            // Clearance from every goal-box edge used for normal tracking.
            static constexpr float BOX_INSET_MM = 50.0f;
            // Proportional return speed in RPM per millimetre outside the inset box.
            static constexpr float RETURN_GAIN = 3.0f;
            // Speed limits while returning from outside the full goal box.
            static constexpr float RETURN_MAX_SPD = 270.0f;
            static constexpr float RETURN_MIN_SPD = 100.0f;
            // Inset-edge correction in RPM per millimetre outside the safe area.
            static constexpr float BOX_CORRECTION_GAIN = 1.2f;
            // Minimum speed used to correct drift beyond an inset edge.
            static constexpr float BOX_CORRECTION_MIN_SPD = 35.0f;
            // Lateral tracking speed in RPM per degree outside the deadband.
            static constexpr float SHUFFLE_GAIN = 3.0f;
            // Maximum speed for lateral tracking and inset-edge correction.
            static constexpr float SHUFFLE_MAX_SPD = 100.0f;
            // Ball-bearing tolerance within which the defender stays centred.
            static constexpr float ALIGNMENT_DEADBAND_DEG = 20.0f;
            // Distance over which lateral movement slows near a side edge.
            static constexpr float EDGE_SLOWDOWN_MM = 100.0f;
            // Defender handoff response cone, measured either side of forward.
            static constexpr float RESPONSE_HALF_ANGLE_DEG = 60.0f;
            // Minimum local ball strength required to initiate a handoff.
            static constexpr float RESPONSE_MIN_STRENGTH = 30.0f;
            // Robot-relative bearing beyond which the attacker has overshot the ball.
            static constexpr float ATTACKER_BEHIND_ANGLE_DEG = 130.0f;
            // Maximum attacker ball strength treated as a distant overshoot.
            static constexpr float ATTACKER_FAR_STRENGTH = 45.0f;
        };

        void setRole(Role newRole);
        void moveInFieldDirection(float dt, float direction, float speed);
        bool isInsideDefenceInset() const;
        bool hasFreshCommunication() const;

        bool isInGoalBox();
        bool isPastOpponentGoalBox();
        bool hasFreshBallReading() const;
};
