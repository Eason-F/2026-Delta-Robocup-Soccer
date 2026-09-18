#pragma once

#include <Arduino.h>
#include <util/PID.hpp>

class Robot;

class Strategy {
    public:
        // tracking state relating to the attacking role
        enum class TrackingStage : uint8_t {
            SEARCH,
            APPROACH,
            ORBIT,
            CAPTURED
        };

        // defence state relating to defending role
        enum class DefenceStage : uint8_t {
            PASSIVE,
            RETURN, 
            SHUFFLE,
            ENGAGE
        };

        enum class Role : uint8_t {
            ATTACK,
            DEFENCE
        };

        explicit Strategy(Robot &robot);

        void update();
        Role getRole() const;
        uint8_t calculateAttackScore(); // returns values from 0-256 for how suitable robot is for attack
        void attack(const float dt);
        void defend(const float dt);

        // attack related
        void checkTrackingStage(const float dt, const float targetBallHeading);
        void maneuverAroundBall(const float dt, const float targetBallHeading);
        void calculateAngleToGoal();

        TrackingStage getTrackingStage() const;

        // defence related
        void returnToHome(const float dt);
        void goalBallTrack(const float dt); // shuffle around the goal while following the ball
        void attackTransition(const float dt); // charge forward from goal


    private:
        Robot &robot;
        Role role = Role::ATTACK;
        TrackingStage trackingStage = TrackingStage::SEARCH;
        DefenceStage defenceStage = DefenceStage::PASSIVE;

        struct ScoreConfigs {
            static constexpr uint8_t DEFENCE_NOT_READY_PENALTY = 100;
            static constexpr uint8_t DEFENCE_RANGE = 70;

            static constexpr uint8_t RETAIN_ATTACK_BIAS = 20;
            static constexpr uint8_t ATTACK_OFFSIDE_PENALTY = 70;
            static constexpr uint8_t ATTACK_SIGNAL_BONUS_RANGE = 130;
            
            static constexpr uint8_t OUT_OF_RESPONSE_ANGLE = 140;
            static constexpr uint8_t OUT_OF_RESPONSE_DISTANCE = 70;
            
        };

        struct AttackConfig {
            // Search and approach tuning (motor targets are in RPM).
            static constexpr uint16_t SEARCH_SPD = 100;
            static constexpr uint16_t APPROACH_SPD = 130;

            // Orbit controller tuning and transition hysteresis.
            static constexpr uint16_t ORBIT_APPROACH_SPD = 130;
            static constexpr uint16_t ORBIT_SPD = 150;
            static constexpr uint16_t ORBIT_DISTANCE = 55;
            static constexpr uint16_t ORBIT_ENTRY_TOLERANCE = 20;
            static constexpr uint16_t ORBIT_EXIT_TOLERANCE = 30;
            static constexpr uint16_t ORBIT_DEBOUNCE_MS = 100;

            // Captured-ball alignment and forward-speed ramp.
            static constexpr uint16_t CAPTURED_MAX_SPD = 200;
            static constexpr uint16_t CAPTURED_MIN_SPD = 270;
            static constexpr uint16_t ENTER_ALIGNMENT_TOLERANCE = 15;
            static constexpr uint16_t EXIT_ALIGNMENT_TOLERANCE = 30;
            static constexpr uint16_t HEADING_DEADBAND = 7;
            static constexpr uint16_t SPEED_RAMP_MAX_MS = 1000;
            static constexpr uint16_t ALIGNED_DEBOUNCE_MS = 0;
        };

        unsigned long accumulatedOrbitTime = 0;
        unsigned long accumulatedAlignedTime = 0;
        
        PIDController approachPID = PIDController(0.5, 0, 0, 0.0, 1.0);
        PIDController orbitTangentPID = PIDController(0.04, 0, 0.001, -1.0, 1.0);
        PIDController orbitDistancePID = PIDController(0.3, 0, 0.001, -0.2, 1.0);

        struct DefenceConfig {
            
        };

        bool isInGoalBox();
        bool isFarInOpponentHalf();
};
