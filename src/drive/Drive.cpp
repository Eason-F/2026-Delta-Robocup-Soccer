// Holonomic wheel mixing and field-position drive commands.
#include <Arduino.h>
#include <drive/Drive.hpp>
#include <util/Logger.hpp>
#include <util/util.hpp>

Drive::Drive() :
    motor1("1", DIRECTION_PIN1_1, DIRECTION_PIN2_1, ENCODER_PIN1_1, ENCODER_PIN2_1, motorPID1),
    motor2("2", DIRECTION_PIN1_2, DIRECTION_PIN2_2, ENCODER_PIN1_2, ENCODER_PIN2_2, motorPID2),
    motor3("3", DIRECTION_PIN1_3, DIRECTION_PIN2_3, ENCODER_PIN1_3, ENCODER_PIN2_3, motorPID3),
    motor4("4", DIRECTION_PIN1_4, DIRECTION_PIN2_4, ENCODER_PIN1_4, ENCODER_PIN2_4, motorPID4) {}

void Drive::setup() {
    analogWriteResolution(8);
    motor1.setup();
    motor2.setup();
    motor3.setup();
    motor4.setup();
}

void Drive::moveInDirection(const float &dt, int directionDegrees, int rpm) {
    // Project translation onto wheels mounted at 45-degree intervals.
    lastDirection = directionDegrees;
    motor1.setMotorRPM(cos(radians(directionDegrees + 315)) * rpm + rotationRpm, dt);
    motor2.setMotorRPM(cos(radians(directionDegrees + 225)) * rpm + rotationRpm, dt);
    motor3.setMotorRPM(cos(radians(directionDegrees + 45 )) * rpm + rotationRpm, dt);
    motor4.setMotorRPM(cos(radians(directionDegrees + 135)) * rpm + rotationRpm, dt);
}

void Drive::moveToPoint(const float &dt, const int &rpm, const float &targetX, const float &targetY, OpticalOdometry &odometry) {
    float velocityX = positionPIDX.adjustmentValue(dt, targetX, odometry.getX()) * rpm;
    float velocityY = positionPIDY.adjustmentValue(dt, targetY, odometry.getY()) * rpm;
    float direction = degrees(atan2(velocityX, velocityY));
    float speed = min(hypot(velocityY, velocityX), rpm);
    Logger::queue("vx", velocityX);
    Logger::queue("vy", velocityY);
    Logger::queue("dir", direction);
    // Logger::queue("spd", speed);

    if (hypot(targetX - odometry.getX(), targetY - odometry.getY()) < 15.0f) {
        stop(); return;
    }
    moveInDirection(dt, direction, max(speed, 40));
}

void Drive::moveToPoint(const float &dt, const int &rpm, const Position2D &target, OpticalOdometry &odometry) {
    moveToPoint(dt, rpm, target.x, target.y, odometry);
}

void Drive::turnInDirection(const float &dt, int rpm) {
    motor1.setMotorRPM(rpm, dt);
    motor2.setMotorRPM(rpm, dt);
    motor3.setMotorRPM(rpm, dt);
    motor4.setMotorRPM(rpm, dt);
}

void Drive::stop() {
    motor1.brake();
    motor2.brake();
    motor3.brake();
    motor4.brake();
}
