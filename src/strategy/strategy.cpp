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
        millis() - robot.robotCommunication.getLastUpdateMillis() <= 500;
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
            setRole(teammate.role == static_cast<uint8_t>(Role::ATTACK) ? Role::DEFENCE : Role::ATTACK);
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
    orbitDebounceTime = 0;
    alignedTime = 0;
    capturedGoalTargetLocked = false;
    if (role == Role::ATTACK) {
        transitionToTrackingStage(
            hasFreshBallReading() ? TrackingStage::APPROACH
                                  : TrackingStage::SEARCH);
    } else {
        defenceStage = DefenceStage::RETURN;
        shuffleRight = true;
        shuffleSwitchTime = 0;
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
    maneuverAroundBall(dt, 0);
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
    }
}

void Strategy::checkDefenceStage() {
    if (!isInGoalBox()) {
        defenceStage = DefenceStage::RETURN;
    } else {
        if (defenceStage != DefenceStage::SHUFFLE) shuffleSwitchTime = 0;
        defenceStage = DefenceStage::SHUFFLE;
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
    // const Position2D targetPosition = FieldConstants::friendlyGoalPosition;
    robot.drive.moveToPoint(dt, DefenceConfig::RETURN_SPD, targetPosition, robot.odometry, robot.imu.getRelativeYaw());
}

void Strategy::goalBallTrack(const float dt) {
    // Check bounds on every command, including direct calls.
    checkDefenceStage();
    if (defenceStage == DefenceStage::RETURN) {
        returnToHome(dt);
        return;
    }
    const float x = robot.odometry.getX();
    const float y = robot.odometry.getY();
    const float left = FieldConstants::friendlyGoalBoxBottomLeft.x + DefenceConfig::BOX_INSET_MM;
    const float right = FieldConstants::friendlyGoalBoxTopRight.x - DefenceConfig::BOX_INSET_MM;
    const float back = FieldConstants::friendlyGoalBoxBottomLeft.y + DefenceConfig::BOX_INSET_MM;
    const float front = FieldConstants::friendlyGoalBoxTopRight.y - DefenceConfig::BOX_INSET_MM;

    float velocityX = 0.0f;
    bool trackingBall = false;
    if (hasFreshBallReading()) {
        const float bearing = util::wrapAngle180(
            robot.irSensor.getDirectionDegrees() + robot.imu.getRelativeYaw());
        if (abs(bearing) <= 90.0f &&
            abs(bearing) > DefenceConfig::ALIGNMENT_DEADBAND_DEG) {
            velocityX = shuffleBearingPID.adjustmentValue(dt, bearing);
            trackingBall = true;
        }
    }

    if (trackingBall) {
        // Begin a fresh jitter cycle once the ball is aligned.
        shuffleSwitchTime = 0;
    } else {
        if (shuffleSwitchTime >= DefenceConfig::SHUFFLE_JITTER_MS) {
            shuffleRight = !shuffleRight;
            shuffleSwitchTime = 0;
        }
        velocityX = shuffleRight ? DefenceConfig::SHUFFLE_JITTER_SPD
                                 : -DefenceConfig::SHUFFLE_JITTER_SPD;
    }

    // At an inset edge, turn inward regardless of the ball bearing.
    if (x <= left) {
        shuffleRight = true;
        shuffleSwitchTime = 0;
        velocityX = DefenceConfig::SHUFFLE_JITTER_SPD;
    }
    if (x >= right) {
        shuffleRight = false;
        shuffleSwitchTime = 0;
        velocityX = -DefenceConfig::SHUFFLE_JITTER_SPD;
    }
    velocityX = constrain(velocityX, -DefenceConfig::SHUFFLE_MAX_SPD,
        DefenceConfig::SHUFFLE_MAX_SPD);
    float velocityY = 0.0f;
    if (y < back) velocityY = DefenceConfig::SHUFFLE_JITTER_SPD;
    if (y > front) velocityY = -DefenceConfig::SHUFFLE_JITTER_SPD;
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
        case TrackingStage::TRANSITION: {
            robot.drive.moveInDirection(
                dt, robot.irSensor.getDirectionDegrees(), AttackConfig::TRANSITION_SPD);
            break;
        }
        default: {
            robot.drive.stop();
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
            break;
        case TrackingStage::APPROACH:
            alignedTime = 0;
            orbitDebounceTime = 0;
            break;
        case TrackingStage::ORBIT:
            alignedTime = 0;
            if (previousStage == TrackingStage::APPROACH) {
                orbitDebounceTime = 0;
            }
            break;
        case TrackingStage::TRANSITION:
            alignedTime = 0;
            break;
        case TrackingStage::CAPTURED: {
            alignedTime = 0;
            const Position2D robotPosition = robot.odometry.getPosition();
            capturedGoalTarget =
                robotPosition.distanceTo(AttackConfig::GOAL_AIM_LEFT) <=
                        robotPosition.distanceTo(AttackConfig::GOAL_AIM_RIGHT)
                    ? AttackConfig::GOAL_AIM_LEFT
                    : AttackConfig::GOAL_AIM_RIGHT;
            capturedGoalTargetLocked = true;
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

    const float signalStrength = robot.irSensor.getSignalStrength();
    const float distanceError = AttackConfig::ORBIT_DISTANCE - signalStrength;
    const float targetRobotBearing = util::wrapAngle180(
        targetBallHeading - robot.imu.getRelativeYaw());
    const float headingError = abs(util::wrapAngle180(
        targetRobotBearing - robot.irSensor.getDirectionDegrees()));

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
            } else if (headingError <= AttackConfig::ENTER_ALIGNMENT_TOLERANCE) {
                transitionToTrackingStage(TrackingStage::TRANSITION);
            }
            return;

        case TrackingStage::TRANSITION:
            if (distanceError > AttackConfig::ORBIT_EXIT_TOLERANCE) {
                transitionToTrackingStage(TrackingStage::APPROACH);
            } else if (headingError > AttackConfig::EXIT_ALIGNMENT_TOLERANCE) {
                transitionToTrackingStage(TrackingStage::ORBIT);
            } else if (headingError > AttackConfig::ENTER_ALIGNMENT_TOLERANCE) {
                alignedTime = 0;
            } else if (alignedTime >= AttackConfig::ALIGNED_DEBOUNCE_MS) {
                transitionToTrackingStage(TrackingStage::CAPTURED);
            }
            return;

        case TrackingStage::CAPTURED:
            if (headingError > AttackConfig::EXIT_ALIGNMENT_TOLERANCE) {
                transitionToTrackingStage(TrackingStage::ORBIT);
            }
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
    robot.targetHeading = constrain(calculateAngleToGoal(), 
        -AttackConfig::MAX_GOAL_TILT, AttackConfig::MAX_GOAL_TILT);
    float ballDirection = robot.irSensor.getDirectionDegrees();
    float ballStrength = robot.irSensor.getSignalStrength();
    const float targetRobotBearing = util::wrapAngle180(
        targetBallHeading - robot.imu.getRelativeYaw());
    orbitAroundPoint(dt, targetRobotBearing, ballDirection, ballStrength);
}

void Strategy::orbitAroundPoint(const float dt, const float targetHeading,
                                const float currentHeading, const float currentDistance) {
    float headingError = util::wrapAngle180(
        currentHeading - targetHeading);
    float distanceError = AttackConfig::ORBIT_DISTANCE - currentDistance;

    float approach = orbitDistancePID.adjustmentValue(dt, distanceError);
    float tangent = -orbitTangentPID.adjustmentValue(dt, headingError);

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
    const float speed = AttackConfig::CAPTURED_MAX_SPD;

    const float ballDirection =
        abs(robot.irSensor.getDirectionDegrees()) <=
                AttackConfig::HEADING_DEADBAND ? 0.0f : 
                robot.irSensor.getDirectionDegrees();
    robot.drive.moveInDirection(dt, ballDirection, speed);
}

void Strategy::returnToNeutralPoint(const float dt) {
    robot.drive.moveToPoint(dt, AttackConfig::SEARCH_SPD, returnTargetPosition,
        robot.odometry, robot.imu.getRelativeYaw());
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
           millis() - robot.irSensor.getLastUpdateMillis() <= 20;
}
