// Top-level setup, control-loop scheduling, and ball strategy implementation.
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
      uartTransport(Serial4),
      irSensor(uartTransport),
      robotCommunication(uartTransport),
      imu(Wire2),
      odometry(Wire),
      colourSensor(22),
      logger(Serial, LOG_INTERVAL_MS),
      strategy(*this) {}

void Robot::setup() {
    button.setup();
    colourSensor.setup();
    drive.setup();
    uartTransport.setup();
    irSensor.setup();
    robotCommunication.setup();
    imu.setup(); imu.resetYawOrigin();
    odometry.setup();
}

void Robot::run() {
    // Fast path: keep sensor and transport state current on every iteration.
    colourSensor.update(elapsedLastUpdateTime);
    uartTransport.update();
    imu.update();
    odometry.update();

    strategy.update();

    if (button.isPressed()) {
        // Active path: correct heading continuously and enact strategy periodically.
        float updateDt = elapsedLastUpdateTime / 1000000.0f;
        handleHeadingCorrection(updateDt, targetHeading);
        conditionallyBreakLoop(handleEdgeDetection(updateDt));

        if (elapsedLastLoopTime >= LOOP_TIME_MS) {
            float dt = elapsedLastLoopTime / 1000.0f;
            elapsedLastLoopTime = 0;

            enforceDefinedRoleBehaviour(dt);
            
        }
        sendBluetoothUpdate();
    } else {
        // Safe idle path: stop motion and redefine the starting pose.
        drive.stop();
        imu.resetYawOrigin();
        odometry.resetPosition();
    }

    elapsedLastUpdateTime = 0;
    // Periodic telemetry; uncomment only the fields needed during tuning.
    logger.update([this](Logger &log) {
        log.log("t", static_cast<int>(millis() / 1000.0f));
        
        // log.log("dir", irSensor.getDirectionDegrees());
        // log.log("str", irSensor.getSignalStrength());
        // log.log("ballFound", irSensor.ballFound());
        // log.log("colour", colourSensor.sensorState());

        log.log("heading", imu.getRelativeYaw());
        log.log("odometryX", odometry.getX());
        log.log("odometryY", odometry.getY());
        log.log("odometryH", odometry.getHeading());

        // log.log("rpm", drive.motor1.angularVelocityRPM);
        // log.log("movementDir", drive.lastDirection);
        
        // log.log("bltX", robotCommunication.getReceivedPacket().x);
        // log.log("bltY", robotCommunication.getReceivedPacket().y);
        // log.log("bltH", robotCommunication.getReceivedPacket().heading);
        // log.log("bltDir", robotCommunication.getReceivedPacket().ballBearing);
        // log.log("bltStr", robotCommunication.getReceivedPacket().ballStrength);
        // log.log("bltScore", robotCommunication.getReceivedPacket().attackScore);
        // log.log("bltState", robotCommunication.getReceivedPacket().state);
        // log.log("bltRole", robotCommunication.getReceivedPacket().role);
        // log.log("bltFlags", robotCommunication.getReceivedPacket().flags);
        // log.log("bltSeq", robotCommunication.getReceivedPacket().sequence);
    });
}

void Robot::enforceDefinedRoleBehaviour(const float dt) {
    if (strategy.getRole() == Strategy::Role::ATTACK) {
        strategy.attack(dt);
    } else {
        strategy.defend(dt);
    }
}

bool Robot::handleEdgeDetection(const float dt) {
    // Refresh the escape direction whenever any boundary sensor sees white.
    if (colourSensor.detectedEdge()) {
        const Vector edgeVector = colourSensor.getVector();

        if (edgeVector.magnitude > 0.1f) {
            escapeDirection = degrees(edgeVector.angle) + 180.0f;
        }
        elapsedEscapeTime = 0;
        odometry.boundaryAlignOdometry(edgeVector, imu.getRelativeYaw());
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

void Robot::sendBluetoothUpdate() {
    uint8_t stage = (strategy.getRole() == Strategy::Role::ATTACK) ? 
        (uint8_t) strategy.getTrackingStage() : 
        (uint8_t) strategy.getDefenceStage();
    RobotPacket packet = {
        static_cast<int16_t>(odometry.getX()),
        static_cast<int16_t>(odometry.getY()),
        static_cast<int16_t>(imu.getRelativeYaw()),
        static_cast<int16_t>(irSensor.getDirectionDegrees()),
        static_cast<uint8_t>(irSensor.getSignalStrength()),
        static_cast<uint8_t>(strategy.calculateAttackScore()), // attack score
        static_cast<uint8_t>(stage), // stage, depends on attack/defend
        static_cast<uint8_t>(strategy.getRole()), // role (attack/defend)
        static_cast<uint8_t>(0), // flags
        static_cast<uint8_t>(packetSequence)
    };
    robotCommunication.sendPacket(packet);
    if (packetSequence < 256) packetSequence++; else packetSequence = 0;
}
