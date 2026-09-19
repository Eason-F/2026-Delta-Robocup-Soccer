#pragma once
#include <Wire.h>
struct sfe_otos_pose2d_t { float x, y, h; };
constexpr int kSfeOtosLinearUnitMeters = 0;
constexpr int kSfeOtosAngularUnitDegrees = 0;
class QwiicOTOS {
public:
    inline static sfe_otos_pose2d_t sensorPose = {};
    inline static unsigned calibrations = 0;
    bool begin(TwoWire &) { return true; }
    void calibrateImu() { ++calibrations; }
    void setLinearUnit(int) {}
    void setAngularUnit(int) {}
    void setOffset(sfe_otos_pose2d_t) {}
    void resetTracking() { sensorPose = {}; }
    void getPosition(sfe_otos_pose2d_t &p) { p = sensorPose; }
    void setPosition(sfe_otos_pose2d_t p) { sensorPose = p; }
};
