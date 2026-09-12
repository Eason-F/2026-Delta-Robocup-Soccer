#include <communication/RobotCommunication.hpp>

#include <cstring>

RobotCommunication::RobotCommunication(UartPacketTransport &transport)
    : transport(transport) {}

void RobotCommunication::setup() {
    transport.setPacketHandler(ESP_NOW_TO_TEENSY_TYPE, handlePacket, this);
}

bool RobotCommunication::sendPacket(const RobotPacket &packet) {
    return transport.sendPacket(
        TEENSY_TO_ESP_NOW_TYPE,
        reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

bool RobotCommunication::hasReceivedPacket() const {
    return receivedPacketValid;
}

const RobotPacket &RobotCommunication::getReceivedPacket() const {
    return receivedPacket;
}

uint32_t RobotCommunication::getLastUpdateMillis() const {
    return lastUpdateMillis;
}

void RobotCommunication::processPacket(const uint8_t *data, uint8_t length) {
    if (length != sizeof(RobotPacket)) {
        return;
    }

    memcpy(&receivedPacket, data, sizeof(receivedPacket));
    receivedPacketValid = true;
    lastUpdateMillis = millis();
}

void RobotCommunication::handlePacket(void *context, const uint8_t *data,
                                      uint8_t length) {
    static_cast<RobotCommunication *>(context)->processPacket(data, length);
}
