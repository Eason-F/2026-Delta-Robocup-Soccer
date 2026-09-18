#pragma once

#include <Arduino.h>
#include <util/PID.hpp>

class Robot;
struct RobotPacket;

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
        Role getRole() const;
        uint8_t calculateAttackScore(); // returns 0-255; higher means more suitable to attack
        void attack(const float dt);
        void defend(const float dt);

        // attack related
        TrackingStage getTrackingStage() const;
        void checkTrackingStage(const float dt, const float targetBallHeading);
        void maneuverAroundBall(const float dt, const float targetBallHeading);
        void calculateAngleToGoal();
        void attackTransition(const float dt); // charge forward from goal

        
        // defence related
        DefenceStage getDefenceStage() const;
        void returnToHome(const float dt);
        void goalBallTrack(const float dt); // shuffle around the goal while following the ball


    private:
        Robot &robot;
        Role role = Role::ATTACK;
        TrackingStage trackingStage = TrackingStage::SEARCH;
        DefenceStage defenceStage = DefenceStage::PASSIVE;

        struct ScoreConfigs {
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
            static constexpr uint8_t ROLE_SWITCH_MARGIN = 8;
            static constexpr uint16_t ROLE_SWITCH_DEBOUNCE_MS = 200;
        };

        Role pendingRole = Role::ATTACK;
        elapsedMillis pendingRoleTime;

        struct AttackConfig {
            // Search and approach tuning (motor targets are in RPM).
            static constexpr uint16_t SEARCH_SPD = 100;
            static constexpr uint16_t APPROACH_SPD = 130;
            static constexpr uint16_t TRANSITION_MIN_MS = 300;
            static constexpr uint16_t TRANSITION_TIMEOUT = 700;

            // Orbit controller tuning and transition hysteresis.
            static constexpr uint16_t ORBIT_APPROACH_SPD = 130;
            static constexpr uint16_t ORBIT_SPD = 150;
            static constexpr uint16_t ORBIT_DISTANCE = 55;
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
            static constexpr uint16_t SPEED_RAMP_MAX_MS = 1000;
            static constexpr uint16_t ALIGNED_DEBOUNCE_MS = 0;
        };

        elapsedMillis transitionTime;
        elapsedMillis orbitDebounceTime;
        elapsedMillis alignedTime;
        
        PIDController approachPID = PIDController(0.5, 0, 0, 0.0, 1.0);
        PIDController orbitTangentPID = PIDController(0.04, 0, 0.001, -1.0, 1.0);
        PIDController orbitDistancePID = PIDController(0.3, 0, 0.001, -0.2, 1.0);

        struct DefenceConfig {
            
        };

        bool isInGoalBox();
        bool isPastOpponentGoalBox();
        bool hasFreshBallReading() const;
        bool winsScoreTie(const RobotPacket &teammate) const;
};
