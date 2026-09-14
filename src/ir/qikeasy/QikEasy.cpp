// QikEasy register reads and vector-based ball direction estimation.
#include <ir/qikeasy/QikEasy.hpp>
#include <util/Logger.hpp>

QikEasy::QikEasy(TwoWire &wirePort) : wirePort(wirePort) {}

void QikEasy::setup() {
    wirePort.begin();
}

Vector QikEasy::readMeasuredSignal(const uint8_t &moduleIndex) {
    const uint8_t moduleAddress = MODULE_ADDRESSES[moduleIndex % 2];
    uint8_t direction = 0; uint16_t strength = 0;

    wirePort.beginTransmission(moduleAddress);
    wirePort.write(DIRECTION_REGISTER);
    wirePort.endTransmission(false);

    wirePort.requestFrom(moduleAddress, (uint8_t) 1);
    if (wirePort.available() >= 1) {
        direction = wirePort.read();
    }
    strength = signalStrength(STRENGTH_REGISTER);
    return Vector(Vector::AngMag {}, direction, strength);
}

uint16_t QikEasy::signalStrength(const int &receiverID) {
    const uint8_t moduleAddress = MODULE_ADDRESSES[(receiverID / 5) % 2];

    wirePort.beginTransmission(moduleAddress);
    wirePort.write(RECEIVER_REGISTERS[receiverID % 5]);
    wirePort.endTransmission(false);
    
    uint16_t strength = 0;
    wirePort.requestFrom(moduleAddress, (uint8_t) 2);
    if (wirePort.available() >= 2) {
        strength = wirePort.read() << 8;
        strength |= wirePort.read();
    }
    // Logger::queue("receiverID", receiverID - 2);
    // Logger::queue("receiverStrength", strength);
    return strength;
}

void QikEasy::updateReadings() {
    if (AVERAGED_VECTOR_MAX > 0) {
        signalVec = vectorReadings();
    } else {
        signalVec = qikeasyReadings();
    }
}

Vector QikEasy::vectorReadings() {
    // Sample ten receivers distributed evenly around the robot.
    for (int i = 2; i < 12; i++) {
        uint16_t strength = signalStrength(i);
        float direction = ((i - 2) * PI / 5);
        allVecs[i - 2] = Vector(Vector::AngMag {}, direction, strength);
    }

    // Combine only the strongest receivers to suppress ambient IR noise.
    Vector strongestVectors[AVERAGED_VECTOR_MAX];
    for (int i = 0; i < AVERAGED_VECTOR_MAX; i++) {
        for (int j = 0; j < 10; j++) {
            if (allVecs[j].magnitude > strongestVectors[i].magnitude) {
                strongestVectors[i] = allVecs[j];
                allVecs[j] = Vector();
            }
        }
    }

    Vector signal = Vector();
    for (int i = 0; i < AVERAGED_VECTOR_MAX; i++) {
        signal = signal + strongestVectors[i];
    }
    signal.magnitude = strongestVectors[0].magnitude;
    return signal;
}

Vector QikEasy::qikeasyReadings() {
    Vector module1 = readMeasuredSignal(0);
    Vector module2 = readMeasuredSignal(1);

    if (module1.magnitude > module2.magnitude) {
        if (module1.magnitude > 5) {
            module1.angle -= 5;
        } else {
            module1.angle += 13;
        }
    } else {
        module2.angle += 4;
    }
    Vector signal = (module1.magnitude > module2.magnitude) ? module1 : module2;
    signal.angle *= PI/9;
    return signal;
}


float QikEasy::strengthToDistance(const uint16_t &strength) {
    // Reserved for an empirically calibrated strength-to-distance curve.
    return 0.0f;
}

float QikEasy::getDirectionDegrees() {
    return degrees(signalVec.angle);
}

float QikEasy::getSignalStrength() {
    return signalVec.magnitude;
}

bool QikEasy::ballFound() {
    return signalVec.magnitude > 0;
}
