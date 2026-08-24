#include <Robot.hpp>
#include <util/util.hpp>

Button::Button(const int &pin): buttonPin(pin) {}

void Button::setup() {
    pinMode(buttonPin, INPUT_PULLUP);
}

bool Button::isPressed() {
    return !digitalRead(buttonPin);
}

Robot::Robot()
    : button(41),
      irSensor(Serial4),
      imu(Wire2),
      colourSensor(22),
      logger(Serial, LOG_INTERVAL_MS) {
    if (WITH_ODOMETRY) {
        odometry = std::make_unique<OpticalOdometry>(Wire2);
    }
}

void Robot::setup() {
    button.setup();
    colourSensor.setup();
    drive.setup();
    irSensor.setup();
    imu.setup(); imu.resetYawOrigin();
    if (odometry != nullptr) {
        odometry-> setup();
    }
}

void Robot::run() {
    colourSensor.update(elapsedLastUpdateTime);
    irSensor.update();
    imu.update();
    if (odometry != nullptr) {
        odometry-> update();
    }

    if (button.isPressed()) {
        float updateDt = elapsedLastUpdateTime / 1000000.0f;
        handleHeadingCorrection(updateDt, targetHeading);
        conditionallyBreakLoop(handleEdgeDetection(updateDt));

        if (elapsedLastLoopTime >= LOOP_TIME_MS) {
            float dt = elapsedLastLoopTime / 1000.0f;
            elapsedLastLoopTime = 0;

            maneuverAroundBall(dt, 0);
            // drive.moveInDirection(dt, irSensor.getDirectionDegrees(), 100);
            // drive.motor1.setMotorRPM(100, dt);
        }
    } else {
        drive.stop();
        imu.resetYawOrigin();
        if (odometry != nullptr) {
            odometry-> resetPosition();
        }
    }

    elapsedLastUpdateTime = 0;
    logger.update([this](Logger &log) {
        log.log("dir", irSensor.getDirectionDegrees());
        log.log("str", irSensor.getSignalStrength());
        log.log("movementDir", drive.lastDirection);
        log.log("state", static_cast<int>(robotState));

        // log.log("heading", imu.getRelativeYaw());
        // log.log("colour", colourSensor.sensorState());
        // log.log("odometryX", odometry.getX());
        // log.log("odometryY", odometry.getY());
        // log.log("odometryH", odometry.getHeading());
        // log.log("rpm", drive.motor1.angularVelocityRPM);
        // log.log("ballFound", irSensor.ballFound());
    });
}

bool Robot::handleEdgeDetection(float dt) {
    if (colourSensor.detectedEdge()) {
        const Vector edgeVector = colourSensor.getVector();

        if (edgeVector.magnitude > 0.1f) {
            escapeDirection = degrees(edgeVector.angle) + 180.0f;
        }
        elapsedEscapeTime = 0;
    }

    if ((elapsedEscapeTime - ESCAPE_DURATION) <= ESCAPE_BUFFER &&
        (elapsedEscapeTime - ESCAPE_DURATION) >= 0) {
        drive.stop();
    }
    if (elapsedEscapeTime >= ESCAPE_DURATION) {
        return false;
    }
    
    drive.moveInDirection(dt, escapeDirection, BOUNDARY_ESCAPE_SPD);
    return true;
}

void Robot::handleHeadingCorrection(const float dt, const float targetHeading) {
    float headingError = util::wrapAngle180(imu.getRelativeYaw() - targetHeading);
    float adjustmentRate = -headingPID.adjustmentValue(dt, headingError) * TURN_SPD;
    drive.rotationRpm = adjustmentRate;
    // Logger::queue("headingErr", headingError);
    // Logger::queue("headingAdj", adjustmentRate);
}

void Robot::handleTargetHeading() {
    targetHeading = 0;
    if (abs(irSensor.getDirectionDegrees()) <= BALL_TILT_RANGE) {
        targetHeading = constrain(-irSensor.getDirectionDegrees(), -BALL_TILT_MAX, BALL_TILT_MAX);
    }
}


void Robot::maneuverAroundBall(const float dt, const float targetBallHeading) {
    checkRobotState(dt, targetBallHeading);
    switch (robotState) {
        case SEARCH: {
            if (odometry != nullptr) {
                drive.moveToPoint(dt, SEARCH_SPD, 0, 0, *odometry);
            } else {
                drive.moveInDirection(dt, 180, SEARCH_SPD);
            }
            break;
        }
        case APPROACH: {
            float speed = approachPID.adjustmentValue(dt, ORBIT_DISTANCE, irSensor.getSignalStrength()) * APPROACH_SPD;
            drive.moveInDirection(dt, irSensor.getDirectionDegrees(), speed);
            break;
        }
        case ORBIT: {
            handleTargetHeading();
            float headingError = util::wrapAngle180(irSensor.getDirectionDegrees() - targetBallHeading - targetHeading);
            float distanceError = ORBIT_DISTANCE - irSensor.getSignalStrength();

            float approach = orbitDistancePID.adjustmentValue(dt, distanceError);
            float tangent = -orbitTangentPID.adjustmentValue(dt, headingError);

            float orbitFactor = 1.0f - min(max(0.0, distanceError) / ORBIT_DISTANCE, 1.0f);
            tangent *= orbitFactor;

            float approachSpeed = approach * ORBIT_APPROACH_SPD;
            float tangentSpeed = tangent * ORBIT_SPD;
            Vector approachVector = Vector(Vector::AngMag {}, irSensor.getDirectionRadians(), approachSpeed);
            Vector tangentVector = Vector(Vector::Position {}, sin(irSensor.getDirectionRadians()), -cos(irSensor.getDirectionRadians())) * tangentSpeed;
            Vector finalVector = tangentVector + approachVector;

            float movementAngle = degrees(finalVector.angle);
            float movementSpeed = min(finalVector.magnitude, ORBIT_SPD);
            drive.moveInDirection(dt, movementAngle, movementSpeed);
            logger.queue("headingErr", headingError);
            logger.queue("approachspd", approachSpeed);
            logger.queue("tangentspd", tangentSpeed);
            break;
        }
        case CAPTURED: {
            targetHeading = 0;
            float alignedTime = (accumulatedAlignedTime - ALIGNED_DEBOUNCE_MS);
            float speed = CAPTURED_MIN_SPD + min(alignedTime + 100 / SPEED_RAMP_MAX_MS, 1.0f) * (CAPTURED_MAX_SPD - CAPTURED_MIN_SPD);
            float direction = (abs(irSensor.getDirectionDegrees()) <= HEADING_DEADBAND) ? 0 : irSensor.getDirectionDegrees();

            drive.moveInDirection(dt, direction, speed);
            break;
        }
    }
}

void Robot::checkRobotState(const float dt, const float targetBallHeading) {
    if (!irSensor.ballFound()) {
        robotState = State::SEARCH;
        accumulatedAlignedTime = 0;
        accumulatedOrbitTime = 0;
        return;
    }

    const float signalStrength = irSensor.getSignalStrength();
    const float headingError = abs(util::wrapAngle180(targetBallHeading - irSensor.getDirectionDegrees()));

    switch (robotState) {
        case State::SEARCH:
        case State::APPROACH:
            if (ORBIT_DISTANCE - signalStrength < ORBIT_ENTRY_TOLERANCE) {
                accumulatedOrbitTime += static_cast<unsigned long>(dt * 1000);

                if (accumulatedOrbitTime >= ORBIT_DEBOUNCE_MS) {
                    robotState = State::ORBIT;
                    accumulatedOrbitTime = 0;
                }
            } else {
                accumulatedOrbitTime = 0;
            }
            break;

        case State::ORBIT:
            if (ORBIT_DISTANCE - signalStrength > ORBIT_EXIT_TOLERANCE) {
                robotState = State::APPROACH;
                accumulatedAlignedTime = 0;
                return;
            }

            if (headingError > ENTER_ALIGNMENT_TOLERANCE) {
                accumulatedAlignedTime = 0;
                return;
            }

            accumulatedAlignedTime += static_cast<unsigned long>(dt * 1000);

            if (accumulatedAlignedTime >= ALIGNED_DEBOUNCE_MS && signalStrength > ORBIT_DISTANCE) {
                robotState = State::CAPTURED;
            }
            break;

        case State::CAPTURED:
            if (headingError > EXIT_ALIGNMENT_TOLERANCE) {
                robotState = State::ORBIT;
                accumulatedAlignedTime = 0;
            }
            break;
    }
}