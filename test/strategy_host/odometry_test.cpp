#include <odometry/Odometry.hpp>
#include <cassert>
#include <iostream>
uint32_t testMillis = 1000;
static bool near(float a,float b) { return abs(a-b)<0.001f; }
int main() {
    TwoWire wire; OpticalOdometry odometry(wire); odometry.setup();
    assert(QwiicOTOS::calibrations==1);
    for (auto p : {Position2D{0,-150},Position2D{0,-615},Position2D{0,-815},
                   FieldConstants::friendlyGoalBoxTopLeft,FieldConstants::friendlyGoalBoxTopRight}) {
        odometry.setFieldPosition(p,17);
        assert(near(odometry.getX(),p.x) && near(odometry.getY(),p.y));
        assert(near(QwiicOTOS::sensorPose.x,p.x/1380));
        assert(near(QwiicOTOS::sensorPose.y,p.y/1380));
        assert(near(odometry.getHeading(),17));
        odometry.update();
        assert(near(odometry.getPosition().x,p.x) && near(odometry.getPosition().y,p.y));
    }
    struct BoundaryCase { float angle, heading, x, y; };
    for (auto c : {BoundaryCase{0,0,120,815},BoundaryCase{180,0,120,-815},
                   BoundaryCase{90,0,510,-765},BoundaryCase{-90,0,-510,-765},
                   BoundaryCase{0,90,510,-765},BoundaryCase{180,90,-510,-765},
                   BoundaryCase{0,30,120,815}}) {
        odometry.setFieldPosition({120,-765});
        odometry.boundaryAlignOdometry(Vector(Vector::AngMag{},radians(c.angle),1),c.heading);
        assert(near(odometry.getX(),c.x) && near(odometry.getY(),c.y));
        assert(near(odometry.getHeading(),c.heading));
    }
    odometry.setFieldPosition({120,-765});
    odometry.boundaryAlignOdometry(Vector(Vector::AngMag{},radians(45),1),0);
    assert(near(odometry.getX(),120) && near(odometry.getY(),-765));
    odometry.boundaryAlignOdometry(Vector(Vector::Position{},0,0),0);
    assert(near(odometry.getX(),120) && near(odometry.getY(),-765));
    odometry.resetPosition();
    assert(near(odometry.getX(),0) && near(odometry.getY(),0));
    assert(QwiicOTOS::calibrations==1);
    std::cout << "Odometry tests passed: preset field units, sensor readback, boundary alignment and reset.\n";
}
