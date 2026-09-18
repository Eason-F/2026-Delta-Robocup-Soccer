#include "Strategy.hpp"
#include <Robot.hpp>

Strategy::Strategy(Robot &robot) : robot(robot) {}

void Strategy::update() {
    if (!robot.robotCommunication.hasReceivedPacket() ||
        millis() - robot.robotCommunication.getLastUpdateMillis() >
            ScoreConfigs::COMMUNICATION_TIMEOUT_MS) {
        role = Role::ATTACK;
        pendingRole = role;
        pendingRoleTime = 0;
        return;
    }

    const RobotPacket &teammate = robot.robotCommunication.getReceivedPacket();
    const uint8_t ownScore = calculateAttackScore();
    Role desiredRole = role;

    if (ownScore > teammate.attackScore + ScoreConfigs::ROLE_SWITCH_MARGIN) {
        desiredRole = Role::ATTACK;
    } else if (teammate.attackScore > ownScore + ScoreConfigs::ROLE_SWITCH_MARGIN) {
        desiredRole = Role::DEFENCE;
    } else if (teammate.role <= static_cast<uint8_t>(Role::DEFENCE) &&
               teammate.role != static_cast<uint8_t>(role)) {
        // Keep an already complementary assignment while scores are close.
        desiredRole = role;
    } else {
        desiredRole = winsScoreTie(teammate) ? Role::ATTACK : Role::DEFENCE;
    }

    if (desiredRole == role) {
        pendingRole = role;
        pendingRoleTime = 0;
        return;
    }

    if (desiredRole != pendingRole) {
        pendingRole = desiredRole;
        pendingRoleTime = 0;
        return;
    }

    if (pendingRoleTime >= ScoreConfigs::ROLE_SWITCH_DEBOUNCE_MS) {
        role = desiredRole;
        pendingRole = role;
        pendingRoleTime = 0;
    }
}

Strategy::Role Strategy::getRole() const{
    return role;
}

Strategy::TrackingStage Strategy::getTrackingStage() const {
    return trackingStage;
}

Strategy::DefenceStage Strategy::getDefenceStage() const {
    return defenceStage;
}

void Strategy::attack(const float dt) {
    maneuverAroundBall(dt, 0);
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
    if (!hasFreshBallReading()) return 0;

    const float signalStrength = robot.irSensor.getSignalStrength();
    const float absoluteBearing = abs(util::wrapAngle180(
        robot.irSensor.getDirectionDegrees()));
    const float strengthFactor = constrain(
        signalStrength / ScoreConfigs::BALL_STRENGTH_FULL_SCALE, 0.0f, 1.0f);
    const float alignmentFactor = constrain(
        1.0f - absoluteBearing / 180.0f, 0.0f, 1.0f);

    float score = strengthFactor * ScoreConfigs::BALL_STRENGTH_WEIGHT;
    score += alignmentFactor * ScoreConfigs::BALL_ALIGNMENT_WEIGHT;

    switch (role) {
        case Role::DEFENCE:
            if (!isInGoalBox()) {
                score -= ScoreConfigs::DEFENCE_NOT_READY_PENALTY;
            }
            score -= constrain(
                robot.odometry.getPosition().distanceTo(
                    FieldConstants::friendlyGoalBoxPosition) /
                    ScoreConfigs::DEFENCE_POSITION_FULL_SCALE,
                0.0f, 1.0f) * ScoreConfigs::DEFENCE_POSITION_PENALTY_MAX;

            if (absoluteBearing <= ScoreConfigs::OUT_OF_RESPONSE_ANGLE &&
                signalStrength >= ScoreConfigs::OUT_OF_RESPONSE_STRENGTH) {
                const float responseFactor = constrain(
                    (signalStrength - ScoreConfigs::OUT_OF_RESPONSE_STRENGTH) /
                    (ScoreConfigs::BALL_STRENGTH_FULL_SCALE -
                     ScoreConfigs::OUT_OF_RESPONSE_STRENGTH),
                    0.0f, 1.0f);
                score += responseFactor * ScoreConfigs::DEFENCE_IN_RANGE_BONUS;
            }
            break;
        case Role::ATTACK:
            score += ScoreConfigs::RETAIN_ATTACK_BIAS;
            if (isPastOpponentGoalBox() &&
                absoluteBearing > ScoreConfigs::OUT_OF_RESPONSE_ANGLE &&
                signalStrength < ScoreConfigs::OUT_OF_RESPONSE_STRENGTH) {
                score -= ScoreConfigs::ATTACK_OFFSIDE_PENALTY;
            }
            score += constrain(
                (signalStrength - ScoreConfigs::ATTACK_SIGNAL_BONUS_START) /
                (ScoreConfigs::BALL_STRENGTH_FULL_SCALE -
                 ScoreConfigs::ATTACK_SIGNAL_BONUS_START),
                0.0f, 1.0f) * ScoreConfigs::ATTACK_SIGNAL_BONUS_MAX;
            break;
    }
    return static_cast<uint8_t>(constrain(score, 0.0f, 255.0f));
}

bool Strategy::isInGoalBox() {
    float x = robot.odometry.getX();
    float y = robot.odometry.getY();
    return (
        abs(x) < FieldConstants::friendlyGoalBoxTopRight.x &&
        y < FieldConstants::friendlyGoalBoxTopRight.y
    );
}

bool Strategy::isPastOpponentGoalBox() {
    return robot.odometry.getY() > FieldConstants::opponentGoalBoxTopLeft.y;
}

bool Strategy::hasFreshBallReading() const {
    return robot.irSensor.ballFound() &&
           millis() - robot.irSensor.getLastUpdateMillis() <=
               ScoreConfigs::IR_READING_TIMEOUT_MS;
}

bool Strategy::winsScoreTie(const RobotPacket &teammate) const {
    // Both robots run this ordering with local/remote values reversed, yielding
    // complementary roles without adding an ID to the radio packet.
    const int16_t ownX = static_cast<int16_t>(robot.odometry.getX());
    const int16_t ownY = static_cast<int16_t>(robot.odometry.getY());
    const int16_t ownHeading = static_cast<int16_t>(robot.imu.getRelativeYaw());
    const int16_t ownBearing = static_cast<int16_t>(
        robot.irSensor.getDirectionDegrees());

    if (ownX != teammate.x) return ownX < teammate.x;
    if (ownY != teammate.y) return ownY > teammate.y;
    if (ownHeading != teammate.heading) return ownHeading < teammate.heading;
    if (ownBearing != teammate.ballBearing) return ownBearing < teammate.ballBearing;

    // The sequence fallback normally differs because packets arrive
    // asynchronously. A permanent exact tie needs an explicit robot ID.
    return robot.packetSequence < teammate.sequence;
}
