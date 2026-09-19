// Top-level setup, control-loop scheduling, and ball strategy implementation.
#include <Robot.hpp>
#include <util/util.hpp>

Button::Button(const int &pin): buttonPin(pin) {}

void Button::setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    rawPressed = stablePressed = !digitalRead(buttonPin);
    changedAt = millis();
}

bool Button::isPressed() {
    const bool pressed = !digitalRead(buttonPin);
    if (pressed != rawPressed) {
        rawPressed = pressed;
        changedAt = millis();
    }
    if (millis() - changedAt >= StartingPreset::DEBOUNCE_MS) stablePressed = rawPressed;
    return stablePressed;
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
    startingPreset.setup();
    colourSensor.setup();
    drive.setup();
    uartTransport.setup();
    irSensor.setup();
    robotCommunication.setup();
    imu.setup(); imu.resetYawOrigin();
    odometry.setup();
}

void Robot::run() {
    updateSensors();

    const bool running = updateRunState();
    updateStrategy(running);
    updateMovement(running);

    elapsedLastUpdateTime = 0;
    logTelemetry();
}

void Robot::updateSensors() {
    colourSensor.update(elapsedLastUpdateTime);
    uartTransport.update();
    imu.update();
    odometry.update();
}

bool Robot::updateRunState() {
    const bool startRequested = button.isPressed();
    const bool presetChanged = startingPreset.update(startRequested);

    // Wait for a pending single or double press before moving.
    const bool running = startRequested && !startingPreset.hasPendingPress();

    if (!running) {
        stopForIdle();
    }

    if (presetChanged || running != wasRunning) {
        applyStartingPosition();
    }

    wasRunning = running;
    return running;
}

void Robot::stopForIdle() {
    drive.stop();
    imu.resetYawOrigin();
    targetHeading = 0;
}

void Robot::applyStartingPosition() {
    const Position2D position = startingPreset.hasPosition()
        ? startingPreset.getPosition()
        : Position2D{};

    odometry.setFieldPosition(position, 0.0f);
    elapsedLastLoopTime = 0;
    elapsedLastUpdateTime = 0;
}

void Robot::updateStrategy(bool running) {
    strategy.configureGame(running, startingPreset.hasPosition(), startingPreset.bothAttack());
    strategy.update();

    // Keep sharing state while idle or escaping a boundary.
    sendBluetoothUpdate();
}

void Robot::updateMovement(bool running) {
    boundaryEscaping = false;
    if (!running) {
        return;
    }

    const float updateDt = max(
        static_cast<float>(elapsedLastUpdateTime) / 1000000.0f,
        0.000001f);
    handleHeadingCorrection(updateDt, targetHeading);
    boundaryEscaping = handleEdgeDetection(updateDt);

    const bool strategyUpdateDue = elapsedLastLoopTime >= LOOP_TIME_MS;
    if (boundaryEscaping || !strategyUpdateDue) {
        return;
    }

    const float strategyDt = elapsedLastLoopTime / 1000.0f;
    elapsedLastLoopTime = 0;
    enforceDefinedRoleBehaviour(strategyDt);
}

void Robot::logTelemetry() {
    logger.update([this](Logger &log) {
        log.log("t", static_cast<int>(millis() / 1000.0f));
        log.log("preset", startingPreset.hasPosition());
        log.log("bothAttack", (strategy.getCommunicationFlags() & 0x08) != 0);

        // Ball
        log.log("ballDeg", irSensor.getDirectionDegrees());
        log.log("ballStr", irSensor.getSignalStrength());
        log.log("ballAgeMs", static_cast<uint32_t>(millis() - irSensor.getLastUpdateMillis()));
        // log.log("colour", colourSensor.sensorState());

        // Position
        log.log("heading", imu.getRelativeYaw());
        log.log("odometryX", odometry.getX());
        log.log("odometryY", odometry.getY());
        log.log("odometryH", odometry.getHeading());

        // Movement
        log.log("role", strategy.getRole() == Strategy::Role::ATTACK ? "ATTACK" : "DEFENCE");
        log.log("stage", strategy.getRole() == Strategy::Role::ATTACK
            ? static_cast<uint8_t>(strategy.getTrackingStage())
            : static_cast<uint8_t>(strategy.getDefenceStage()));
        log.log("driveRPM", drive.lastTranslationRpm);
        log.log("moveDeg", drive.lastDirection);
        log.log("m1RPM", drive.motor1.angularVelocityRPM);
        log.log("edgeEscape", boundaryEscaping);

        // Teammate
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
    const uint8_t flags = strategy.getCommunicationFlags();
    // Normal telemetry at 50 Hz; role handoffs bypass the rate limit.
    if (elapsedLastBluetoothUpdate < 20 && flags == lastCommunicationFlags) return;
    uint8_t stage = (strategy.getRole() == Strategy::Role::ATTACK) ? 
        (uint8_t) strategy.getTrackingStage() : 
        (uint8_t) strategy.getDefenceStage();
    RobotPacket packet = {
        static_cast<int16_t>(odometry.getX()),
        static_cast<int16_t>(odometry.getY()),
        static_cast<int16_t>(imu.getRelativeYaw()),
        static_cast<int16_t>(irSensor.getDirectionDegrees()),
        static_cast<uint8_t>(constrain(irSensor.getSignalStrength(), 0.0f, 255.0f)),
        static_cast<uint8_t>(strategy.calculateAttackScore()), // attack score
        static_cast<uint8_t>(stage), // stage, depends on attack/defend
        static_cast<uint8_t>(strategy.getRole()), // role (attack/defend)
        flags,
        static_cast<uint8_t>(packetSequence)
    };
    if (robotCommunication.sendPacket(packet)) {
        elapsedLastBluetoothUpdate = 0;
        lastCommunicationFlags = flags;
        ++packetSequence;
    }
}
