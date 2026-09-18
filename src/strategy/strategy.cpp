#include "strategy.hpp"
#include <Robot.hpp>

Strategy::Strategy(Robot &robot) : robot(robot) {}

void Strategy::update() {
    RobotPacket communications = robot.robotCommunication.getReceivedPacket();
    role = (communications.attackScore > calculateAttackScore()) ? Role::ATTACK : Role::DEFENCE;
}

Strategy::Role Strategy::getRole() const{
    return role;
}

Strategy::TrackingStage Strategy::getTrackingStage() const {
    return trackingStage;
}

void Strategy::attack(const float dt) {
}

void Strategy::defend(const float dt) {
}

void Strategy::maneuverAroundBall(const float dt, const float targetBallHeading) {
    // Convert the current search state into one drive command.
    checkTrackingStage(dt, targetBallHeading);
    switch (trackingStage) {
        case TrackingStage::SEARCH: {
            robot.drive.moveToPoint(dt, AttackConfig::SEARCH_SPD, 0, 0, robot.odometry);
            break;
        }
        case TrackingStage::APPROACH: {
            float speed = approachPID.adjustmentValue(dt, AttackConfig::ORBIT_DISTANCE, robot.irSensor.getSignalStrength()) * AttackConfig::APPROACH_SPD;
            robot.drive.moveInDirection(dt, robot.irSensor.getDirectionDegrees(), speed);
            break;
        }
        case TrackingStage::ORBIT: {
            robot.handleTargetHeading();
            float headingError = util::wrapAngle180(robot.irSensor.getDirectionDegrees() - targetBallHeading - robot.targetHeading);
            float distanceError = AttackConfig::ORBIT_DISTANCE - robot.irSensor.getSignalStrength();

            float approach = orbitDistancePID.adjustmentValue(dt, distanceError);
            float tangent = -orbitTangentPID.adjustmentValue(dt, headingError);

            float orbitFactor = 1.0f - min(max(0.0, distanceError) / AttackConfig::ORBIT_DISTANCE, 1.0f);
            tangent *= orbitFactor;

            float approachSpeed = approach * AttackConfig::ORBIT_APPROACH_SPD;
            float tangentSpeed = tangent * AttackConfig::ORBIT_SPD;
            Vector approachVector = Vector(
                Vector::AngMag {}, 
                robot.irSensor.getDirectionRadians(), 
                approachSpeed
            );
            Vector tangentVector = Vector(
                Vector::Position {}, 
                sin(robot.irSensor.getDirectionRadians()), 
                -cos(robot.irSensor.getDirectionRadians())
            ) * tangentSpeed;
            Vector finalVector = tangentVector + approachVector;

            float movementAngle = degrees(finalVector.angle);
            float movementSpeed = min(finalVector.magnitude, AttackConfig::ORBIT_SPD);
            robot.drive.moveInDirection(dt, movementAngle, movementSpeed);
            // robot.logger.queue("headingErr", headingError);
            // robot.logger.queue("approachspd", approachSpeed);
            // robot.logger.queue("tangentspd", tangentSpeed);
            break;
        }
        case TrackingStage::CAPTURED: {
            robot.targetHeading = 0;
            float alignedTime = (accumulatedAlignedTime - AttackConfig::ALIGNED_DEBOUNCE_MS);
            float speed = AttackConfig::CAPTURED_MIN_SPD + min(alignedTime + 100 / AttackConfig::SPEED_RAMP_MAX_MS, 1.0f) * (AttackConfig::CAPTURED_MAX_SPD - AttackConfig::CAPTURED_MIN_SPD);
            float direction = (abs(robot.irSensor.getDirectionDegrees()) <= AttackConfig::HEADING_DEADBAND) ? 0 : robot.irSensor.getDirectionDegrees();

            robot.drive.moveInDirection(dt, direction, speed);
            break;
        }
    }
}

void Strategy::checkTrackingStage(const float dt, const float targetBallHeading) {
    // Apply distance/alignment hysteresis so noisy readings do not chatter.
    if (!robot.irSensor.ballFound()) {
        trackingStage = TrackingStage::SEARCH;
        accumulatedAlignedTime = 0;
        accumulatedOrbitTime = 0;
        return;
    }

    const float signalStrength = robot.irSensor.getSignalStrength();
    const float headingError = abs(util::wrapAngle180(targetBallHeading - robot.irSensor.getDirectionDegrees()));

    switch (trackingStage) {
        case TrackingStage::SEARCH:
        case TrackingStage::APPROACH:
            if (AttackConfig::ORBIT_DISTANCE - signalStrength < AttackConfig::ORBIT_ENTRY_TOLERANCE) {
                accumulatedOrbitTime += static_cast<unsigned long>(dt * 1000);

                if (accumulatedOrbitTime >= AttackConfig::ORBIT_DEBOUNCE_MS) {
                    trackingStage = TrackingStage::ORBIT;
                    accumulatedOrbitTime = 0;
                }
            } else {
                accumulatedOrbitTime = 0;
            }
            break;

        case TrackingStage::ORBIT:
            if (AttackConfig::ORBIT_DISTANCE - signalStrength > AttackConfig::ORBIT_EXIT_TOLERANCE) {
                trackingStage = TrackingStage::APPROACH;
                accumulatedAlignedTime = 0;
                return;
            }

            if (headingError > AttackConfig::ENTER_ALIGNMENT_TOLERANCE) {
                accumulatedAlignedTime = 0;
                return;
            }

            accumulatedAlignedTime += static_cast<unsigned long>(dt * 1000);

            if (accumulatedAlignedTime >= AttackConfig::ALIGNED_DEBOUNCE_MS && signalStrength > AttackConfig::ORBIT_DISTANCE) {
                trackingStage = TrackingStage::CAPTURED;
            }
            break;

        case TrackingStage::CAPTURED:
            if (headingError > AttackConfig::EXIT_ALIGNMENT_TOLERANCE) {
                trackingStage = TrackingStage::ORBIT;
                accumulatedAlignedTime = 0;
            }
            break;
    }
}

uint8_t Strategy::calculateAttackScore() {
    float score = 0.0f;
    switch (role) {
        case Role::DEFENCE:
            if (!isInGoalBox()) score -= ScoreConfigs::DEFENCE_NOT_READY_PENALTY; // penalise not being in goal box
            score -= robot.odometry.getPosition().distanceTo(FieldConstants::friendlyGoalBoxPosition);
            score += util::sigmoid(robot.irSensor.getSignalStrength() - ScoreConfigs::DEFENCE_RANGE);
            break;
        case Role::ATTACK:
            score += ScoreConfigs::RETAIN_ATTACK_BIAS; // bias to keep attacking robot in attack
            if (isFarInOpponentHalf() && 
                abs(robot.irSensor.getDirectionDegrees() > ScoreConfigs::OUT_OF_RESPONSE_ANGLE) &&
                abs(robot.irSensor.getSignalStrength() < ScoreConfigs::OUT_OF_RESPONSE_DISTANCE)) {
                score -= ScoreConfigs::ATTACK_OFFSIDE_PENALTY; // switch when too far forward
            }
            score += max(robot.irSensor.getSignalStrength() - ScoreConfigs::ATTACK_SIGNAL_BONUS_RANGE, 0);
            break;
    }
    return static_cast<uint8_t> (constrain(score, 0, 256));
}

bool Strategy::isInGoalBox() {
    float x = robot.odometry.getX();
    float y = robot.odometry.getY();
    return (
        abs(x) < FieldConstants::friendlyGoalBoxTopRight.x &&
        y < FieldConstants::friendlyGoalBoxTopRight.y
    );
}

bool Strategy::isFarInOpponentHalf() {
    return robot.odometry.getY() > FieldConstants::opponentGoalBoxTopLeft.y;
}