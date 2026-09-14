// UART IR packet validation, decoding, and latest-reading accessors.
#include <ir/uart/UartIRSensor.hpp>

UartIRSensor::UartIRSensor(UartPacketTransport &transport)
    : transport(transport) {}

void UartIRSensor::setup() {
    transport.setPacketHandler(IR_MEASUREMENT_TYPE, handlePacket, this);
}

void UartIRSensor::processPacket(const uint8_t *data, uint8_t length) {
    if (length != IR_PAYLOAD_LENGTH) {
        return;
    }

    const int16_t bearingCentidegrees =
        static_cast<int16_t>(readU16(data));
    directionDegrees = static_cast<float>(bearingCentidegrees) / 100.0f;
    signalStrength = static_cast<float>(readU16(data + sizeof(uint16_t)));
    readingValid = true;
    lastUpdateMillis = millis();
}

void UartIRSensor::handlePacket(void *context, const uint8_t *data,
                                uint8_t length) {
    static_cast<UartIRSensor *>(context)->processPacket(data, length);
}

uint16_t UartIRSensor::readU16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

float UartIRSensor::getDirectionDegrees() const {
    return directionDegrees;
}

float UartIRSensor::getDirectionRadians() const {
    return radians(directionDegrees);
}

float UartIRSensor::getSignalStrength() const {
    return signalStrength;
}

bool UartIRSensor::ballFound() const {
    return readingValid;
}

uint32_t UartIRSensor::getLastUpdateMillis() const {
    return lastUpdateMillis;
}

float UartIRSensor::strengthToDistance(const uint16_t &strength) {
    // Reserved for an empirically calibrated strength-to-distance curve.
    return 0.0f;
}
