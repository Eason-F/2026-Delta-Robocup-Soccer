#include <Robot.hpp>
#include <colour/colour.hpp>
#include <odometry/Odometry.hpp>
#include <cassert>
#include <iostream>
uint32_t testMillis = 1000;
int main() {
    TwoWire wire;
    OpticalOdometry odometry(wire);
    odometry.setup();
    odometry.setFieldPosition({120,-815});
    ColourSensor colours;
    colours.setup();
    for (int &pin : testPins) pin = LOW;
    testPins[26]=HIGH; // Physical rear colour module sees the friendly white line.
    colours.update(1);
    assert(colours.detectedEdge());
    odometry.boundaryAlignOdometry(colours.getVector(),0);
    assert(abs(odometry.getX()-120)<0.001f);
    assert(abs(odometry.getY()+815)<0.001f);

    Robot robot;
    Strategy strategy(robot);
    robot.odometry={odometry.getX(),odometry.getY()};
    strategy.configureGame(true,true,false);
    RobotPacket teammate;
    teammate.flags=5; // Initialized attacker with a valid preset.
    robot.robotCommunication={teammate,true,millis()};
    strategy.update();
    assert(strategy.getRole()==Strategy::Role::DEFENCE);
    // After boundary escape, the next defence command must shuffle, not return
    // backward again. Use a ball below the explicit role-switch threshold.
    for (float bearing : {-40.0f,40.0f}) {
        robot.irSensor={bearing,20,true,millis()};
        strategy.update();
        strategy.defend(.015f);
        assert(strategy.getRole()==Strategy::Role::DEFENCE);
        assert(strategy.getDefenceStage()==Strategy::DefenceStage::SHUFFLE);
        assert(robot.drive.direction==(bearing<0 ? -90 : 90));
        assert(robot.drive.speed>0);
    }
    std::cout << "Boundary tracking regression passed: rear line preserves goal-box pose and left/right tracking resumes.\n";
}
