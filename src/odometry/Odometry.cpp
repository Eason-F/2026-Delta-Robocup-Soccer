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

float OpticalOdometry::getX() const {
    return position.x * LINEAR_MULTIPLIER;
}

float OpticalOdometry::getY() const {
    return position.y * LINEAR_MULTIPLIER;
}

float OpticalOdometry::getHeading() const {
    return position.h;
}

Position2D OpticalOdometry::getPosition() const {
    return {getX(), getY()};
}

void OpticalOdometry::update() {
    odometrySensor.getPosition(position);
}

void OpticalOdometry::setPosition(sfe_otos_pose2d_t &pose) {
    odometrySensor.setPosition(pose);
    position = pose;
}

void OpticalOdometry::setFieldPosition(const Position2D &fieldPosition, float heading) {
    // Reverse the calibrated scale used by getX()/getY(); OTOS uses metres.
    sfe_otos_pose2d_t pose = {
        fieldPosition.x / LINEAR_MULTIPLIER,
        fieldPosition.y / LINEAR_MULTIPLIER,
        heading
    };
    setPosition(pose);
}

void OpticalOdometry::resetPosition() {
    odometrySensor.resetTracking();
    position = {};
}

void OpticalOdometry::boundaryAlignOdometry(const Vector &boundaryVector, const float &heading) {
    if (boundaryVector.magnitude <= 0.1f) return;
    const float fieldAngle = boundaryVector.angle + radians(heading);
    const float normalX = sin(fieldAngle);
    const float normalY = cos(fieldAngle);

    if (max(abs(normalX), abs(normalY)) < BOUNDARY_AXIS_ALIGNMENT_MIN) return;
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

    Position2D corrected = getPosition();
    if (abs(normalX) > abs(normalY)) {
        corrected.x = normalX > 0.0f ? boundaryX : -boundaryX;
    } else {
        corrected.y = normalY > 0.0f ? boundaryY : -boundaryY;
    }
    if (corrected.distanceTo(getPosition()) > BOUNDARY_CORRECTION_TOLERANCE_MAX) return;

    setFieldPosition(corrected, heading);
}
