#include "strategy.hpp"
#include <Robot.hpp>

Strategy::Strategy(Robot &robot) : robot(robot) {}

void Strategy::configureGame(bool running, bool hasStartingPosition, bool forceBothAttack) {
    if (running != gameplayActive || hasStartingPosition != startingPositionValid ||
        forceBothAttack != localBothAttack) {
        rolesInitialized = false;
        standaloneAttack = false;
        roleEpoch = 0;
        setRole(Role::ATTACK);
    }
    gameplayActive = running;
    startingPositionValid = hasStartingPosition;
    localBothAttack = forceBothAttack;
    if (!running) bothAttackLatched = false;
    else if (!hasStartingPosition || forceBothAttack) bothAttackLatched = true;
}

bool Strategy::hasFreshCommunication() const {
    return robot.robotCommunication.hasReceivedPacket() &&
        millis() - robot.robotCommunication.getLastUpdateMillis() <=
            ScoreConfigs::COMMUNICATION_TIMEOUT_MS;
}

void Strategy::update() {
    // Read teammate state.
    const bool communicationFresh = hasFreshCommunication();
    const RobotPacket &teammate = robot.robotCommunication.getReceivedPacket();
    const bool teammateHasPosition = (teammate.flags & POSITION_VALID_FLAG) != 0;
    const bool teammateRequestsAttack = communicationFresh &&
        ((teammate.flags & BOTH_ATTACK_FLAG) != 0 ||
         ((teammate.flags & RUNNING_FLAG) != 0 && !teammateHasPosition));

    // Keep both-attack mode until the run ends.
    if (gameplayActive && teammateRequestsAttack) {
        bothAttackLatched = true;
    }

    // Attack alone when roles cannot be shared safely.
    if (!communicationFresh || localBothAttack || bothAttackLatched ||
        teammateRequestsAttack || !startingPositionValid || !teammateHasPosition) {
        if (!standaloneAttack) {
            setRole(Role::ATTACK);
            rolesInitialized = false;
            roleEpoch = 0;
            standaloneAttack = true;
        }
        return;
    }

    // Ignore invalid role data.
    if (teammate.role > static_cast<uint8_t>(Role::DEFENCE)) {
        return;
    }

    standaloneAttack = false;
    const bool teammateReady = (teammate.flags & ROLE_READY_FLAG) != 0;
    const uint8_t teammateEpoch = teammate.flags >> EPOCH_SHIFT;

    // Wait until play starts before choosing roles.
    if (!gameplayActive) {
        rolesInitialized = false;
        roleEpoch = 0;
        return;
    }

    // Choose the starting roles.
    if (!rolesInitialized) {
        if (teammateReady) {
            roleEpoch = teammateEpoch;
            setRole(teammate.role == static_cast<uint8_t>(Role::ATTACK)
                        ? Role::DEFENCE : Role::ATTACK);
        } else {
            const int16_t ownY = static_cast<int16_t>(robot.odometry.getY());
            if (ownY == teammate.y) {
                return;
            }

            setRole(ownY > teammate.y ? Role::ATTACK : Role::DEFENCE);
        }

        rolesInitialized = true;
        return;
    }

    // Follow a newer handoff from the teammate.
    if (!teammateReady) {
        return;
    }

    const uint8_t epochDifference = (teammateEpoch - roleEpoch) & EPOCH_MASK;
    if (epochDifference != 0) {
        if (epochDifference < 4) {
            roleEpoch = teammateEpoch;
            setRole(teammate.role == static_cast<uint8_t>(Role::ATTACK)
                        ? Role::DEFENCE : Role::ATTACK);
        }

        return;
    }

    // Only a defender in the goal box may start a handoff.
    if (role != Role::DEFENCE ||
        teammate.role != static_cast<uint8_t>(Role::ATTACK) ||
        !isInGoalBox()) {
        return;
    }

    // Respond to a nearby ball in front of the defender.
    const bool ballInResponseZone = hasFreshBallReading() &&
        robot.irSensor.getSignalStrength() >= DefenceConfig::RESPONSE_MIN_STRENGTH &&
        abs(util::wrapAngle180(robot.irSensor.getDirectionDegrees())) <=
            DefenceConfig::RESPONSE_HALF_ANGLE_DEG;

    // Take over when the attacker has passed a distant ball.
    const float attackerRelativeBearing = util::wrapAngle180(
        static_cast<float>(teammate.ballBearing));
    const bool attackerOvershot = (teammate.flags & FRESH_BALL_FLAG) != 0 &&
        teammate.ballStrength < DefenceConfig::ATTACKER_FAR_STRENGTH &&
        abs(attackerRelativeBearing) > DefenceConfig::ATTACKER_BEHIND_ANGLE_DEG;

    // Start a new handoff.
    if (ballInResponseZone || attackerOvershot) {
        roleEpoch = (roleEpoch + 1) & EPOCH_MASK;
        setRole(Role::ATTACK);
    }
}

void Strategy::setRole(Role newRole) {
    if (rolesInitialized && role == newRole) return;
    role = newRole;
    robot.targetHeading = 0;
    transitionTime = 0;
    orbitDebounceTime = 0;
    alignedTime = 0;
    capturedGoalTargetLocked = false;
    if (role == Role::ATTACK) {
        transitionToTrackingStage(
            hasFreshBallReading() ? TrackingStage::APPROACH
                                  : TrackingStage::SEARCH);
    } else {
        defenceStage = DefenceStage::RETURN;
    }
}

uint8_t Strategy::getCommunicationFlags() const {
    return static_cast<uint8_t>((roleEpoch << EPOCH_SHIFT) |
        (rolesInitialized ? ROLE_READY_FLAG : 0) |
        (hasFreshBallReading() ? FRESH_BALL_FLAG : 0) |
        (startingPositionValid ? POSITION_VALID_FLAG : 0) |
        ((localBothAttack || bothAttackLatched) ? BOTH_ATTACK_FLAG : 0) |
        (gameplayActive ? RUNNING_FLAG : 0));
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
    if (!rolesInitialized && !standaloneAttack) {
        robot.drive.stop();
        return;
    }
    switch (trackingStage) {
         case TrackingStage::TRANSITION: {
            const float direction =
                (abs(robot.irSensor.getDirectionDegrees()) <=
                 AttackConfig::HEADING_DEADBAND) ? 0.0f : robot.irSensor.getDirectionDegrees();
            robot.drive.moveInDirection(dt, direction,AttackConfig::TRANSITION_SPD);
            break;
        }
        default:
            maneuverAroundBall(dt, calculateAngleToGoal());
    }
}

void Strategy::defend(const float dt) {
    robot.targetHeading = 0;
    if (!rolesInitialized || dt <= 0.0f) {
        robot.drive.stop();
        return;
    }
    checkDefenceStage();
    switch (defenceStage) {
        case DefenceStage::RETURN:
            returnToHome(dt);
            break;
        case DefenceStage::SHUFFLE:
            goalBallTrack(dt);
            break;
        case DefenceStage::PASSIVE:
            // Zero translation retains heading correction.
            robot.drive.moveInDirection(dt, 0, 0);
            break;
    }
}

bool Strategy::isInsideDefenceInset() const {
    const float x = robot.odometry.getX();
    const float y = robot.odometry.getY();
    return x >= FieldConstants::friendlyGoalBoxBottomLeft.x + DefenceConfig::BOX_INSET_MM &&
           x <= FieldConstants::friendlyGoalBoxTopRight.x - DefenceConfig::BOX_INSET_MM &&
           y >= FieldConstants::friendlyGoalBoxBottomLeft.y + DefenceConfig::BOX_INSET_MM &&
           y <= FieldConstants::friendlyGoalBoxTopRight.y - DefenceConfig::BOX_INSET_MM;
}

void Strategy::checkDefenceStage() {
    if (!isInGoalBox()) {
        defenceStage = DefenceStage::RETURN;
    } else {
        // Near an edge, keep correcting position even if ball data is missing.
        defenceStage = hasFreshBallReading() || !isInsideDefenceInset()
            ? DefenceStage::SHUFFLE : DefenceStage::PASSIVE;
    }
}

void Strategy::returnToHome(const float dt) {
    const float x = robot.odometry.getX();
    const float y = robot.odometry.getY();
    // Return toward the nearest point inside the inset box.
    const float targetX = constrain(x,
        FieldConstants::friendlyGoalBoxBottomLeft.x + DefenceConfig::BOX_INSET_MM,
        FieldConstants::friendlyGoalBoxTopRight.x - DefenceConfig::BOX_INSET_MM);
    const float targetY = constrain(y,
        FieldConstants::friendlyGoalBoxBottomLeft.y + DefenceConfig::BOX_INSET_MM,
        FieldConstants::friendlyGoalBoxTopRight.y - DefenceConfig::BOX_INSET_MM);
    const Position2D targetPosition = {targetX, targetY};
    robot.drive.moveToPoint(dt, DefenceConfig::RETURN_SPD, targetPosition, robot.odometry, robot.imu.getRelativeYaw());
}

void Strategy::goalBallTrack(const float dt) {
    // Check bounds and freshness on every command, including direct calls.
    checkDefenceStage();
    if (defenceStage == DefenceStage::RETURN) {
        returnToHome(dt);
        return;
    }
    if (defenceStage == DefenceStage::PASSIVE) {
        robot.drive.moveInDirection(dt, 0, 0);
        return;
    }
    const float x = robot.odometry.getX();
    const float y = robot.odometry.getY();
    const float left = FieldConstants::friendlyGoalBoxBottomLeft.x + DefenceConfig::BOX_INSET_MM;
    const float right = FieldConstants::friendlyGoalBoxTopRight.x - DefenceConfig::BOX_INSET_MM;
    const float back = FieldConstants::friendlyGoalBoxBottomLeft.y + DefenceConfig::BOX_INSET_MM;
    const float front = FieldConstants::friendlyGoalBoxTopRight.y - DefenceConfig::BOX_INSET_MM;

    float velocityX = 0.0f;
    if (hasFreshBallReading()) {
        const float bearing = util::wrapAngle180(
            robot.irSensor.getDirectionDegrees() + robot.imu.getRelativeYaw());
        if (abs(bearing) > DefenceConfig::ALIGNMENT_DEADBAND_DEG && abs(bearing) <= 90.0f) {
            const bool moveRight = bearing > 0.0f;
            const float remaining = moveRight ? right - x : x - left;
            const float edgeFactor = constrain(remaining / DefenceConfig::EDGE_SLOWDOWN_MM, 0.0f, 1.0f);
            const float speed = min(
                (abs(bearing) - DefenceConfig::ALIGNMENT_DEADBAND_DEG) * DefenceConfig::SHUFFLE_GAIN,
                DefenceConfig::SHUFFLE_MAX_SPD) * edgeFactor;
            velocityX = moveRight ? speed : -speed;
        }
    }

    const auto correctionSpeed = [](float error) {
        if (error == 0.0f) return 0.0f;
        const float speed = constrain(abs(error) * DefenceConfig::BOX_CORRECTION_GAIN,
            DefenceConfig::BOX_CORRECTION_MIN_SPD, DefenceConfig::SHUFFLE_MAX_SPD);
        return error > 0.0f ? speed : -speed;
    };
    // Never chase outward through a side limit. Inward ball tracking can aid recovery.
    if (x < left) velocityX = max(velocityX, correctionSpeed(left - x));
    if (x > right) velocityX = min(velocityX, correctionSpeed(right - x));

    // Hold current Y throughout the safe band; correct only when near an edge.
    const float velocityY = correctionSpeed(constrain(y, back, front) - y);
    const float speed = min(hypot(velocityX, velocityY), DefenceConfig::SHUFFLE_MAX_SPD);
    robot.drive.moveInFieldDirection(
        dt, speed > 0.0f ? degrees(atan2(velocityX, velocityY)) : 0.0f, 
        speed, robot.imu.getRelativeYaw()
    );
}

void Strategy::maneuverAroundBall(const float dt, const float targetBallHeading) {
    // Convert the current search state into one drive command.
    checkTrackingStage(dt, targetBallHeading);
    switch (trackingStage) {
        case TrackingStage::SEARCH: {
            robot.drive.moveToPoint(
                dt, AttackConfig::SEARCH_SPD,
                FieldConstants::friendlyGoalBoxPosition.x,
                FieldConstants::friendlyGoalBoxPosition.y, 
                robot.odometry, robot.imu.getRelativeYaw());
            break;
        }
        case TrackingStage::APPROACH: {
            float speed = 
                approachPID.adjustmentValue(
                    dt, AttackConfig::ORBIT_DISTANCE, robot.irSensor.getSignalStrength()) * 
                    AttackConfig::APPROACH_SPD;
            robot.drive.moveInDirection(dt, robot.irSensor.getDirectionDegrees(), speed);
            break;
        }
        case TrackingStage::ORBIT: {
            orbitAroundBall(dt, targetBallHeading);
            break;
        }
        case TrackingStage::CAPTURED: {
            pushCapturedBallToGoal(dt);
            break;
        }
        default: {
            robot.drive.stop(); // should never happen because transition is handled elsewhere
            break;
        }
    }
}

void Strategy::transitionToTrackingStage(const TrackingStage nextStage) {
    const TrackingStage previousStage = trackingStage;
    trackingStage = nextStage;

    if (nextStage != TrackingStage::CAPTURED) {
        robot.targetHeading = 0;
        capturedGoalTargetLocked = false;
    }

    switch (nextStage) {
        case TrackingStage::SEARCH:
            alignedTime = 0;
            orbitDebounceTime = 0;
            transitionTime = 0;
            break;
        case TrackingStage::APPROACH:
            alignedTime = 0;
            orbitDebounceTime = 0;
            if (previousStage != TrackingStage::ORBIT) {
                transitionTime = 0;
            }
            break;
        case TrackingStage::ORBIT:
            alignedTime = 0;
            if (previousStage == TrackingStage::APPROACH) {
                orbitDebounceTime = 0;
            } else if (previousStage == TrackingStage::TRANSITION) {
                transitionTime = 0;
            }
            break;
        case TrackingStage::TRANSITION:
            transitionTime = 0;
            break;
        case TrackingStage::CAPTURED: {
            transitionTime = 0;
            const Position2D robotPosition = robot.odometry.getPosition();
            capturedGoalTarget =
                robotPosition.distanceTo(AttackConfig::GOAL_AIM_LEFT) <=
                        robotPosition.distanceTo(AttackConfig::GOAL_AIM_RIGHT)
                    ? AttackConfig::GOAL_AIM_LEFT
                    : AttackConfig::GOAL_AIM_RIGHT;
            capturedGoalTargetLocked = true;
            break;
        }
        case TrackingStage::RETURN: {
            alignedTime = 0;
            orbitDebounceTime = 0;
            transitionTime = 0;
            const Position2D robotPosition = robot.odometry.getPosition();
            returnTargetPosition =
                robotPosition.distanceTo(FieldConstants::centreLeftMark) <=
                        robotPosition.distanceTo(FieldConstants::centreRightMark)
                    ? FieldConstants::centreLeftMark
                    : FieldConstants::centreRightMark;
            returnTargetPosition = returnTargetPosition + Vector(Vector::Position {}, 0, -110.0f);
            break;
        }
    }
}

void Strategy::checkTrackingStage(const float, const float targetBallHeading) {
    // Apply distance/alignment hysteresis so noisy readings do not chatter.
    if (!hasFreshBallReading()) {
        transitionToTrackingStage(TrackingStage::SEARCH);
        return;
    }

    if (trackingStage != TrackingStage::RETURN &&
        ballOutsideBoundaryConfidence() >
            AttackConfig::OUTSIDE_BOUNDARY_CONFIDENCE_THRESHOLD) {
        transitionToTrackingStage(TrackingStage::RETURN);
        return;
    }

    const float signalStrength = robot.irSensor.getSignalStrength();
    const float distanceError = AttackConfig::ORBIT_DISTANCE - signalStrength;
    const float headingError = abs(util::wrapAngle180(
        targetBallHeading - robot.irSensor.getDirectionDegrees()));

    switch (trackingStage) {
        case TrackingStage::SEARCH:
            transitionToTrackingStage(TrackingStage::APPROACH);
            return;

        case TrackingStage::APPROACH:
            if (distanceError >= AttackConfig::ORBIT_ENTRY_TOLERANCE) {
                orbitDebounceTime = 0;
            } else if (orbitDebounceTime >= AttackConfig::ORBIT_DEBOUNCE_MS) {
                transitionToTrackingStage(TrackingStage::ORBIT);
            }
            return;

        case TrackingStage::ORBIT:
            if (distanceError > AttackConfig::ORBIT_EXIT_TOLERANCE) {
                transitionToTrackingStage(TrackingStage::APPROACH);
            } else if (headingError > AttackConfig::ENTER_ALIGNMENT_TOLERANCE) {
                alignedTime = 0;
            } else if (alignedTime >= AttackConfig::ALIGNED_DEBOUNCE_MS) {
                transitionToTrackingStage(TrackingStage::TRANSITION);
            }
            return;

        case TrackingStage::TRANSITION:
            if (transitionTime < AttackConfig::TRANSITION_MIN_MS) return;

            if (transitionTime >= AttackConfig::TRANSITION_TIMEOUT) {
                transitionToTrackingStage(TrackingStage::SEARCH);
            } else if (distanceError > AttackConfig::ORBIT_EXIT_TOLERANCE) {
                transitionToTrackingStage(TrackingStage::APPROACH);
            } else if (headingError > AttackConfig::EXIT_ALIGNMENT_TOLERANCE) {
                transitionToTrackingStage(TrackingStage::ORBIT);
            } else if (signalStrength >= AttackConfig::CAPTURED_DISTANCE) {
                transitionToTrackingStage(TrackingStage::CAPTURED);
            }
            return;

        case TrackingStage::CAPTURED:
            if (headingError > AttackConfig::EXIT_ALIGNMENT_TOLERANCE) {
                transitionToTrackingStage(TrackingStage::ORBIT);
            } else if (signalStrength < AttackConfig::CAPTURED_EXIT_DISTANCE) {
                transitionToTrackingStage(TrackingStage::TRANSITION);
            }
            return;

        case TrackingStage::RETURN:
            return;
    }
}

float Strategy::calculateAngleToGoal() const {
    const Position2D robotPosition = robot.odometry.getPosition();
    const Position2D target = capturedGoalTargetLocked
        ? capturedGoalTarget
        : (robotPosition.distanceTo(AttackConfig::GOAL_AIM_LEFT) <=
                   robotPosition.distanceTo(AttackConfig::GOAL_AIM_RIGHT)
               ? AttackConfig::GOAL_AIM_LEFT
               : AttackConfig::GOAL_AIM_RIGHT);
    return robotPosition.angleTo(target);
}

void Strategy::orbitAroundBall(const float dt, const float targetBallHeading) {
    float ballDirection = robot.irSensor.getDirectionDegrees();
    float ballStrength = robot.irSensor.getSignalStrength();
    orbitAroundPoint(dt, targetBallHeading, ballDirection, ballStrength);
}

void Strategy::orbitAroundPoint(const float dt, const float targetHeading,
                                const float currentHeading, const float currentDistance) {
    float headingError = util::wrapAngle180(
        currentHeading - targetHeading - robot.targetHeading);
    float distanceError = AttackConfig::ORBIT_DISTANCE - currentDistance;

    float approach = orbitDistancePID.adjustmentValue(dt, distanceError);
    float tangent = -orbitTangentPID.adjustmentValue(dt, headingError);

    float orbitFactor = 1.0f - min(max(0.0, distanceError) / AttackConfig::ORBIT_DISTANCE, 1.0f);
    tangent *= orbitFactor;

    float approachSpeed = approach * AttackConfig::ORBIT_APPROACH_SPD;
    float tangentSpeed = tangent * AttackConfig::ORBIT_SPD;
    const float currentHeadingRadians = radians(currentHeading);
    Vector approachVector = Vector(Vector::AngMag {}, currentHeadingRadians, approachSpeed);
    Vector tangentVector = 
        Vector(Vector::Position {}, sin(currentHeadingRadians), -cos(currentHeadingRadians)) * tangentSpeed;
    Vector finalVector = tangentVector + approachVector;

    float movementAngle = degrees(finalVector.angle);
    float movementSpeed = min(finalVector.magnitude, AttackConfig::ORBIT_SPD);
    robot.drive.moveInDirection(dt, movementAngle, movementSpeed);
}

void Strategy::pushCapturedBallToGoal(const float dt) {
    const float goalHeading = calculateAngleToGoal();
    robot.targetHeading = goalHeading;

    const float headingError = abs(util::wrapAngle180(
        goalHeading - robot.imu.getRelativeYaw()));
    const float alignmentFactor = constrain(
        1.0f - headingError / AttackConfig::GOAL_ALIGNMENT_FULL_SPEED_DEG,
        AttackConfig::GOAL_ALIGNMENT_MIN_SPEED_FACTOR, 1.0f);

    const float rampTime = 
        max(static_cast<float>(alignedTime) - AttackConfig::ALIGNED_DEBOUNCE_MS, 0.0f);
    const float rampFactor = 
        min((rampTime + 100.0f) / AttackConfig::SPEED_RAMP_MAX_MS, 1.0f);
    const float speed =
        (AttackConfig::CAPTURED_MIN_SPD +
         rampFactor * (AttackConfig::CAPTURED_MAX_SPD -
                       AttackConfig::CAPTURED_MIN_SPD)) *
        alignmentFactor;

    const float ballDirection =
        abs(robot.irSensor.getDirectionDegrees()) <=
                AttackConfig::HEADING_DEADBAND ? 0.0f : 
                robot.irSensor.getDirectionDegrees();
    robot.drive.moveInDirection(dt, ballDirection, speed);
}

void Strategy::returnToNeutralPoint(const float dt) {
    Position2D robotPosition = robot.odometry.getPosition();
    float distance = robotPosition.distanceTo(returnTargetPosition);
    float angle = robotPosition.angleTo(returnTargetPosition);
    orbitAroundPoint(dt, 0, angle, distance);
}

uint8_t Strategy::calculateAttackScore() {
    // DEPRECATED. Retained for telemetry.
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
                Position2D(robot.odometry.getX(), robot.odometry.getY()).distanceTo(
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

float Strategy::ballOutsideBoundaryConfidence() {
    if (!hasFreshBallReading()) return 0.0f;

    const Position2D robotPosition = robot.odometry.getPosition();
    const float fieldBearing = radians(util::wrapAngle180(
        robot.irSensor.getDirectionDegrees() + robot.imu.getRelativeYaw()));
    const float directionX = sin(fieldBearing);
    const float directionY = cos(fieldBearing);

    const float distancesToBoundary[] = {
        robotPosition.x - FieldConstants::boundaryBottomLeft.x,
        FieldConstants::boundaryBottomRight.x - robotPosition.x,
        robotPosition.y - FieldConstants::boundaryBottomLeft.y,
        FieldConstants::boundaryTopLeft.y - robotPosition.y,
    };
    const float outwardAlignments[] = {
        -directionX, directionX, -directionY, directionY,
    };

    float confidence = 0.0f;
    for (size_t side = 0; side < 4; ++side) {
        const float proximity = constrain(
            1.0f - distancesToBoundary[side] / AttackConfig::PROXIMITY_RANGE_MM, 0.0f, 1.0f);
        const float alignment = constrain(outwardAlignments[side], 0.0f, 1.0f);
        confidence = max(confidence, proximity * alignment);
    }
    return confidence;
}

bool Strategy::isInGoalBox() {
    float x = robot.odometry.getX();
    float y = robot.odometry.getY();
    return (
        x >= FieldConstants::friendlyGoalBoxBottomLeft.x &&
        x <= FieldConstants::friendlyGoalBoxTopRight.x &&
        y >= FieldConstants::friendlyGoalBoxBottomLeft.y &&
        y <= FieldConstants::friendlyGoalBoxTopRight.y
    );
}

bool Strategy::isPastOpponentGoalBox() {
    return robot.odometry.getY() > FieldConstants::opponentGoalBoxTopLeft.y;
}

bool Strategy::hasFreshBallReading() const {
    return robot.irSensor.ballFound() && robot.irSensor.getSignalStrength() > 0.0f &&
           millis() - robot.irSensor.getLastUpdateMillis() <=
               ScoreConfigs::IR_READING_TIMEOUT_MS;
}
