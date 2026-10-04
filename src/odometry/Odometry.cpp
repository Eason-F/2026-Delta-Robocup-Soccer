// Optical odometry setup, calibration, and pose accessors.
#include <odometry/Odometry.hpp>
#include <util/util.hpp>

sfe_otos_pose2d_t OpticalOdometry::SENSOR_OFFSET = {0.0f, 0.0f, -45.0f};

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
    // Only snap position when the robot faces near a field axis.
    if (abs(std::remainder(heading, 90.0f)) > ROBOT_AXIS_ALIGNMENT_TOLERANCE_DEG) return;

    // The sensor angle starts at robot-forward and increases toward robot-right.
    const float boundaryBearing = boundaryVector.angle + radians(heading);
    const float boundaryDirectionX = sin(boundaryBearing);
    const float boundaryDirectionY = cos(boundaryBearing);
    if (max(abs(boundaryDirectionX), abs(boundaryDirectionY)) < BOUNDARY_AXIS_ALIGNMENT_MIN) return;

    const float boundaryX = FieldConstants::fieldWidth / 2 -
        FieldConstants::boundaryInset - FieldConstants::boundaryLineWidth -
        BOUNDARY_CORRECTION_OFFSET;
    const float boundaryY = FieldConstants::fieldLength / 2 -
        FieldConstants::boundaryInset - FieldConstants::boundaryLineWidth -
        BOUNDARY_CORRECTION_OFFSET;

    Position2D correctedPosition = getPosition();
    float correction;
    if (abs(boundaryDirectionX) > abs(boundaryDirectionY)) {
        const float alignedX = boundaryDirectionX > 0.0f ? boundaryX : -boundaryX;
        correction = alignedX - correctedPosition.x;
        correctedPosition.x = alignedX;
    } else {
        const float alignedY = boundaryDirectionY > 0.0f ? boundaryY : -boundaryY;
        correction = alignedY - correctedPosition.y;
        correctedPosition.y = alignedY;
    }

    // if (abs(correction) > BOUNDARY_CORRECTION_TOLERANCE_MAX) return;
    setFieldPosition(correctedPosition, heading);
}
