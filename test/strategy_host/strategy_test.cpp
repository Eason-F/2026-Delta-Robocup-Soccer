#include <Robot.hpp>
#include <cassert>
#include <iostream>
uint32_t testMillis = 1000;
static void ball(Robot &r, float bearing, float strength) {
    r.irSensor = {bearing, strength, true, millis()};
}
static RobotPacket packet(const Robot &r, const Strategy &s) {
    RobotPacket p;
    p.x = r.odometry.x; p.y = r.odometry.y;
    p.heading = r.imu.yaw; p.ballBearing = r.irSensor.bearing;
    p.ballStrength = constrain(r.irSensor.strength, 0.0f, 255.0f);
    p.role = static_cast<uint8_t>(s.getRole()); p.flags = s.getCommunicationFlags();
    return p;
}
static void receive(Robot &r, RobotPacket p) { r.robotCommunication = {p, true, millis()}; }
static void initializeDefender(Robot &r, Strategy &s) {
    r.odometry.y = -765;
    s.configureGame(true, true, false);
    RobotPacket p; p.flags = 5;
    receive(r,p); s.update();
    assert(s.getRole() == Strategy::Role::DEFENCE);
}
static void movement() {
    Robot r; Strategy s(r); initializeDefender(r,s);
    s.defend(.015f);
    assert(s.getDefenceStage() == Strategy::DefenceStage::PASSIVE && r.drive.speed == 0);
    for (float bearing : {-40.0f, 40.0f}) {
        ball(r,bearing,80); s.defend(.015f);
        assert(r.drive.direction == (bearing < 0 ? -90 : 90) && r.drive.speed > 0);
    }
    for (float bearing : {-19.0f, 19.0f, 160.0f}) {
        ball(r,bearing,80); s.defend(.015f); assert(r.drive.speed == 0);
    }
    for (float sign : {-1.0f, 1.0f}) {
        ball(r,sign*40,80); r.odometry.x = sign*390; s.defend(.015f);
        assert(r.drive.speed > 0 && r.drive.speed <= 6);
        r.odometry.x = sign*400; s.defend(.015f); assert(r.drive.speed == 0);
        ball(r,-sign*40,80); s.defend(.015f); assert(r.drive.speed > 0);
    }
    struct Case { float x,y; int angle; };
    for (auto c : {Case{500,-765,-90},Case{-500,-765,90},Case{0,-500,180},Case{0,-1000,0}}) {
        r.odometry={c.x,c.y}; s.defend(.015f);
        assert(s.getDefenceStage()==Strategy::DefenceStage::RETURN);
        assert(abs(util::wrapAngle180(r.drive.direction-c.angle)) <= 1);
        assert(r.drive.speed > 0 && r.drive.speed <= 270);
    }
    r.odometry={0,-765}; r.imu.yaw=30; ball(r,10,80); s.defend(.015f);
    assert(r.drive.direction==60 && r.drive.speed>0);
    testMillis+=251; s.defend(.015f);
    assert(s.getDefenceStage()==Strategy::DefenceStage::PASSIVE && r.drive.speed==0);
    s.defend(0); assert(r.drive.stopped);
}
static void defensiveDriftCorrection() {
    Robot r; Strategy s(r); initializeDefender(r,s);
    ball(r,40,80);
    // No preferred Y line: movement stays lateral anywhere in the safe band.
    for (float y : {-865.0f,-815.0f,-700.0f,-665.0f}) {
        r.odometry={0,y}; s.defend(.015f);
        assert(r.drive.direction==90 && r.drive.speed>0);
    }
    // Track right while correcting back from the front margin, and vice versa.
    r.odometry={0,-640}; s.defend(.015f);
    assert(s.getDefenceStage()==Strategy::DefenceStage::SHUFFLE);
    assert(r.drive.direction>90 && r.drive.direction<180);
    assert(r.drive.speed>0 && r.drive.speed<=100);
    r.odometry={0,-890}; s.defend(.015f);
    assert(r.drive.direction>0 && r.drive.direction<90);
    // At a corner, boundary correction overrides an outward ball command.
    r.odometry={425,-640}; s.defend(.015f);
    assert(r.drive.direction < -90 && r.drive.direction > -180);
    // A lost or centred ball must not disable forward/backward correction.
    r.odometry={0,-890}; r.irSensor.valid=false; s.defend(.015f);
    assert(s.getDefenceStage()==Strategy::DefenceStage::SHUFFLE);
    assert(r.drive.direction==0 && r.drive.speed>0);
    r.odometry={0,-640}; ball(r,0,80); s.defend(.015f);
    assert(abs(r.drive.direction)==180 && r.drive.speed>0);
    // Far outside: full faster return speed; slow down as the box approaches.
    r.odometry={0,-150}; s.defend(.015f);
    assert(s.getDefenceStage()==Strategy::DefenceStage::RETURN);
    assert(r.drive.speed==270 && abs(r.drive.direction)==180);
    r.odometry={0,-600}; s.defend(.015f);
    assert(r.drive.speed>0 && r.drive.speed<270);
}
static void gates() {
    Robot r; Strategy s(r); initializeDefender(r,s); ball(r,0,150);
    for (auto p : {Position2D{451,-765},Position2D{-451,-765},Position2D{0,-614},Position2D{0,-916}}) {
        r.odometry={p.x,p.y}; s.update(); assert(s.getRole()==Strategy::Role::DEFENCE);
    }
    r.odometry={0,-765};
    ball(r,61,150); s.update(); assert(s.getRole()==Strategy::Role::DEFENCE);
    ball(r,60,29); s.update(); assert(s.getRole()==Strategy::Role::DEFENCE);
    receive(r,r.robotCommunication.packet); r.irSensor.updated=millis()-251;
    s.update(); assert(s.getRole()==Strategy::Role::DEFENCE);
    ball(r,60,30); s.update(); assert(s.getRole()==Strategy::Role::ATTACK);
    for(int i=0;i<10;++i) s.update();
    assert((s.getCommunicationFlags()>>5)==1);
}
static void overshot() {
    Robot r; Strategy s(r); initializeDefender(r,s);
    auto p=r.robotCommunication.packet; p.ballBearing=170; p.ballStrength=30;
    receive(r,p); s.update(); assert(s.getRole()==Strategy::Role::DEFENCE);
    p.flags|=2; p.ballStrength=45;
    receive(r,p); s.update(); assert(s.getRole()==Strategy::Role::DEFENCE);
    p.ballStrength=30; p.ballBearing=90;
    receive(r,p); s.update(); assert(s.getRole()==Strategy::Role::DEFENCE);
    p.ballBearing=170; receive(r,p); s.update(); assert(s.getRole()==Strategy::Role::ATTACK);
}
static void switchThresholdBoundaries() {
    struct ResponseCase { float bearing, strength; bool swap; };
    for (auto c : {ResponseCase{60,30,true},ResponseCase{-60,30,true},
                   ResponseCase{61,30,false},ResponseCase{-61,30,false},
                   ResponseCase{0,29,false}}) {
        Robot r; Strategy s(r); initializeDefender(r,s);
        ball(r,c.bearing,c.strength); s.update();
        assert((s.getRole()==Strategy::Role::ATTACK)==c.swap);
    }
    struct OvershotCase { int16_t bearing, heading; uint8_t strength; bool swap; };
    for (auto c : {OvershotCase{131,-90,44,true},OvershotCase{-131,90,44,true},
                   OvershotCase{130,90,44,false},OvershotCase{-130,-90,44,false},
                   OvershotCase{170,0,45,false},OvershotCase{90,90,44,false}}) {
        Robot r; Strategy s(r); initializeDefender(r,s);
        auto p=r.robotCommunication.packet;
        p.flags|=2; p.ballBearing=c.bearing; p.heading=c.heading; p.ballStrength=c.strength;
        receive(r,p); s.update();
        assert((s.getRole()==Strategy::Role::ATTACK)==c.swap);
    }
}
static void handoffs() {
    Robot front,back; Strategy a(front),b(back);
    a.configureGame(true,true,false); b.configureGame(true,true,false);
    a.attack(.015f); assert(front.drive.stopped);
    receive(front,packet(back,b)); a.update(); assert(!(a.getCommunicationFlags()&1));
    front.odometry.y=-300; back.odometry.y=-765;
    auto pf=packet(front,a),pb=packet(back,b);
    receive(front,pb); receive(back,pf); a.update(); b.update();
    assert(a.getRole()==Strategy::Role::ATTACK && b.getRole()==Strategy::Role::DEFENCE);
    for(int i=0;i<70;++i) {
        bool backDefends=b.getRole()==Strategy::Role::DEFENCE;
        Robot &dr=backDefends?back:front, &ar=backDefends?front:back;
        Strategy &ds=backDefends?b:a, &as=backDefends?a:b;
        dr.odometry={0,-765}; ar.odometry={0,-300}; ball(dr,0,150); ball(ar,0,80);
        auto old=packet(dr,ds);
        receive(dr,packet(ar,as)); ds.update(); assert(ds.getRole()==Strategy::Role::ATTACK);
        receive(ar,packet(dr,ds)); as.update(); assert(as.getRole()==Strategy::Role::DEFENCE);
        ball(ar,0,150); as.update(); assert(as.getRole()==Strategy::Role::DEFENCE);
        receive(ar,old); as.update(); assert(as.getRole()==Strategy::Role::DEFENCE);
        assert((a.getCommunicationFlags()>>5)==((i+1)&7));
        assert((b.getCommunicationFlags()>>5)==((i+1)&7));
    }
}
static void communicationFallback() {
    Robot solo;
    Strategy s(solo);
    s.update(); // No packet has ever arrived: attack must actually be enabled.
    assert(s.getRole() == Strategy::Role::ATTACK);
    s.attack(.015f);
    assert(solo.drive.targetX == 0 && solo.drive.targetY == -765);
    assert(!solo.drive.stopped && solo.drive.speed == 270); // SEARCH return also uses the faster target.
    assert(!(s.getCommunicationFlags() & 1));
    solo.targetHeading = 12;
    s.update();
    assert(solo.targetHeading == 12); // Do not reset attack on every offline update.

    Robot front, back;
    Strategy a(front), b(back);
    a.configureGame(true,true,false); b.configureGame(true,true,false);
    front.odometry.y = -300;
    initializeDefender(back, b);
    receive(front, packet(back, b)); a.update();
    assert(a.getRole() == Strategy::Role::ATTACK);
    back.odometry.y = -500; // Even a returning defender defaults to attack.
    testMillis += 500;
    b.update(); assert(b.getRole() == Strategy::Role::DEFENCE);
    ++testMillis;
    b.update(); assert(b.getRole() == Strategy::Role::ATTACK);
    b.attack(.015f); assert(!back.drive.stopped && back.drive.speed > 0);
    receive(back, packet(front, a)); b.update();
    assert(b.getRole() == Strategy::Role::DEFENCE); // One-sided loss recovers.

    testMillis += 501;
    a.update(); b.update();
    assert(a.getRole() == Strategy::Role::ATTACK && b.getRole() == Strategy::Role::ATTACK);
    const auto frontPacket = packet(front, a), backPacket = packet(back, b);
    receive(front, backPacket); receive(back, frontPacket);
    a.update(); b.update();
    assert(a.getRole() == Strategy::Role::ATTACK && b.getRole() == Strategy::Role::DEFENCE);
}
static void presetGameModes() {
    Robot front, back;
    Strategy a(front), b(back);
    front.odometry.y = -150; back.odometry.y = -815;
    a.configureGame(true, false, false); // Null at start latches both attack.
    b.configureGame(true, true, false);
    receive(front, packet(back,b)); a.update();
    receive(back, packet(front,a)); b.update();
    assert(a.getRole()==Strategy::Role::ATTACK && b.getRole()==Strategy::Role::ATTACK);
    assert((a.getCommunicationFlags()&8) && (b.getCommunicationFlags()&8));
    assert((a.getCommunicationFlags()&4)==0 && (b.getCommunicationFlags()&4));
    // Peer resets or the link fails: the running robot retains the game latch.
    a.configureGame(false,false,false);
    receive(back,packet(front,a)); b.update();
    assert(b.getCommunicationFlags()&8);
    testMillis+=501; b.update(); assert(b.getCommunicationFlags()&8);
    // Both reset and select positions: normal roles are available next game.
    b.configureGame(false,false,false);
    a.configureGame(false,true,false); b.configureGame(false,true,false);
    auto pa=packet(front,a), pb=packet(back,b);
    receive(front,pb); receive(back,pa); a.update(); b.update();
    assert(!(a.getCommunicationFlags()&8) && !(b.getCommunicationFlags()&8));
    a.configureGame(true,true,false); b.configureGame(true,true,false);
    pa=packet(front,a); pb=packet(back,b);
    receive(front,pb); receive(back,pa); a.update(); b.update();
    assert(a.getRole()==Strategy::Role::ATTACK && b.getRole()==Strategy::Role::DEFENCE);
    // Explicit double-12 mode is shared even with two valid positions.
    a.configureGame(false,true,true); b.configureGame(false,true,false);
    receive(back,packet(front,a)); b.update();
    assert(b.getRole()==Strategy::Role::ATTACK && !(b.getCommunicationFlags()&8));
    a.configureGame(true,true,true); b.configureGame(true,true,false);
    receive(front,packet(back,b)); a.update();
    receive(back,packet(front,a)); b.update();
    assert(a.getCommunicationFlags()&8); assert(b.getCommunicationFlags()&8);

    // Re-select positions while idle: previous front/back assignment is discarded.
    a.configureGame(false,true,false); b.configureGame(false,true,false);
    front.odometry.y=-815; back.odometry.y=-150;
    pa=packet(front,a); pb=packet(back,b);
    receive(front,pb); receive(back,pa); a.update(); b.update();
    assert(!(a.getCommunicationFlags()&1) && !(b.getCommunicationFlags()&1));
    a.configureGame(true,true,false); b.configureGame(true,true,false);
    pa=packet(front,a); pb=packet(back,b);
    receive(front,pb); receive(back,pa); a.update(); b.update();
    assert(a.getRole()==Strategy::Role::DEFENCE && b.getRole()==Strategy::Role::ATTACK);
}
int main() {
    movement(); defensiveDriftCorrection(); gates(); overshot(); switchThresholdBoundaries(); handoffs(); communicationFallback(); presetGameModes();
    std::cout << "Strategy tests passed: defence, 70 handoffs, communication fallback, preset roles and both-attack run latching.\n";
}
