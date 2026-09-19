#pragma once
#include <strategy/strategy.hpp>
#include <util/FieldConstants.hpp>
struct RobotPacket {
    int16_t x = 0, y = 0, heading = 0, ballBearing = 0;
    uint8_t ballStrength = 0, attackScore = 0, state = 0, role = 0, flags = 0, sequence = 0;
};
class Robot {
public:
    struct Odometry {
        float x = 0, y = 0;
        float getX() const { return x; }
        float getY() const { return y; }
    } odometry;
    struct IR {
        float bearing = 0, strength = 0;
        bool valid = false;
        uint32_t updated = 0;
        float getDirectionDegrees() const { return bearing; }
        float getDirectionRadians() const { return radians(bearing); }
        float getSignalStrength() const { return strength; }
        bool ballFound() const { return valid; }
        uint32_t getLastUpdateMillis() const { return updated; }
    } irSensor;
    struct IMU {
        float yaw = 0;
        float getRelativeYaw() const { return yaw; }
    } imu;
    struct Communication {
        RobotPacket packet;
        bool valid = false;
        uint32_t updated = 0;
        bool hasReceivedPacket() const { return valid; }
        uint32_t getLastUpdateMillis() const { return updated; }
        const RobotPacket &getReceivedPacket() const { return packet; }
    } robotCommunication;
    struct Drive {
        int direction = 0, speed = 0;
        float targetX = 0, targetY = 0;
        bool stopped = false;
        void stop() { stopped = true; speed = 0; }
        void moveInDirection(float, int angle, int rpm) {
            direction = angle; speed = rpm; stopped = false;
        }
        void moveToPoint(float, int rpm, float x, float y, Odometry &) { speed = rpm; targetX = x; targetY = y; }
    } drive;
    float targetHeading = 0;
    void handleTargetHeading() { targetHeading = 0; }
};
