// Optical odometry setup, calibration, and pose accessors.
#include <odometry/Odometry.hpp>
#include <util/util.hpp>

sfe_otos_pose2d_t OpticalOdometry::SENSOR_OFFSET = {0.0f, 0.0f, -135.0f};

OpticalOdometry::OpticalOdometry(TwoWire &wirePort) : wirePort(wirePort) {}

void OpticalOdometry::setup() {
    wirePort.begin();
    wirePort.setClock(1000000);
    while (!odometrySensor.begin(wirePort)) {
        LOG_PRINT("Disconnected odometry"); LOG_NEXT;
    };
    odometrySensor.calibrateImu();
    
    odometrySensor.setLinearUnit(kSfeOtosLinearUnitMeters);
    odometrySensor.setAngularUnit(kSfeOtosAngularUnitDegrees);
    odometrySensor.setOffset(SENSOR_OFFSET);

    odometrySensor.resetTracking();
}

float OpticalOdometry::getX() {
    return position.x * LINEAR_MULTIPLIER;
}

float OpticalOdometry::getY() {
    return position.y * LINEAR_MULTIPLIER;
}

float OpticalOdometry::getHeading() {
    return position.h;
}

void OpticalOdometry::update() {
    odometrySensor.getPosition(position);
}

void OpticalOdometry::setPosition(sfe_otos_pose2d_t &pose) {
    odometrySensor.setPosition(pose);
}

void OpticalOdometry::resetPosition() {
    odometrySensor.calibrateImu(255, true);
    odometrySensor.resetTracking();
}

void OpticalOdometry::boundaryAlignOdometry(const Vector &boundaryVector, const float &heading) {
    float x = boundaryVector.x;
    float y = boundaryVector.y;
    float boundaryX = 
        FieldConstants::fieldWidth / 2 - 
        FieldConstants::boundaryInset - 
        FieldConstants::boundaryLineWidth - 
        BOUNDARY_CORRECTION_OFFSET;
    float boundaryY = 
        FieldConstants::fieldLength / 2 - 
        FieldConstants::boundaryInset - 
        FieldConstants::boundaryLineWidth -
        BOUNDARY_CORRECTION_OFFSET;

    sfe_otos_pose2d_t position = {
        x * boundaryX,
        y * boundaryY,
        heading
    };
    setPosition(position);
}
