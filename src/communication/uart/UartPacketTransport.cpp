// UART frame encoding, byte-wise parsing, dispatch, and CRC calculation.
#include <communication/uart/UartPacketTransport.hpp>

#include <cstring>

UartPacketTransport::UartPacketTransport(HardwareSerial &serialPort,
                                         uint32_t baudRate)
    : serialPort(serialPort), baudRate(baudRate) {}

void UartPacketTransport::setup() {
    serialPort.begin(baudRate);
    resetReceiver();
}

void UartPacketTransport::update() {
    while (serialPort.available() > 0) {
        processByte(static_cast<uint8_t>(serialPort.read()));
    }
}

bool UartPacketTransport::sendPacket(uint8_t type, const uint8_t *data,
                                     uint8_t length) {
    if (length > MAX_PAYLOAD_LENGTH || (length != 0 && data == nullptr)) {
        return false;
    }

    // CRC covers type through payload; marker bytes and CRC are excluded.
    uint8_t frame[5 + MAX_PAYLOAD_LENGTH + sizeof(uint16_t)];
    frame[0] = MARKER_0;
    frame[1] = MARKER_1;
    frame[2] = type;
    frame[3] = length;
    frame[4] = transmitSequence;

    if (length != 0) {
        memcpy(frame + 5, data, length);
    }

    uint16_t crc = 0xFFFF;
    for (uint8_t index = 2; index < 5 + length; ++index) {
        crc = updateCrc(crc, frame[index]);
    }

    frame[5 + length] = static_cast<uint8_t>(crc);
    frame[6 + length] = static_cast<uint8_t>(crc >> 8);

    const size_t frameLength = 7 + length;
    const bool sent = serialPort.write(frame, frameLength) == frameLength;
    if (sent) {
        ++transmitSequence;
    }
    return sent;
}

bool UartPacketTransport::setPacketHandler(uint8_t type,
                                           PacketHandler handler,
                                           void *context) {
    for (HandlerEntry &entry : handlers) {
        if (entry.handler == nullptr || entry.type == type) {
            entry.type = type;
            entry.handler = handler;
            entry.context = context;
            return true;
        }
    }
    return false;
}

void UartPacketTransport::processByte(uint8_t value) {
    switch (receiveState) {
        case ReceiveState::WAITING_FOR_MARKER_0:
            if (value == MARKER_0) {
                receiveState = ReceiveState::WAITING_FOR_MARKER_1;
            }
            break;
        case ReceiveState::WAITING_FOR_MARKER_1:
            if (value == MARKER_1) {
                receiveState = ReceiveState::READING_TYPE;
            } else if (value != MARKER_0) {
                receiveState = ReceiveState::WAITING_FOR_MARKER_0;
            }
            break;
        case ReceiveState::READING_TYPE:
            packetType = value;
            calculatedCrc = updateCrc(0xFFFF, value);
            receiveState = ReceiveState::READING_LENGTH;
            break;
        case ReceiveState::READING_LENGTH:
            payloadLength = value;
            calculatedCrc = updateCrc(calculatedCrc, value);
            if (payloadLength > MAX_PAYLOAD_LENGTH) {
                resetReceiver(value);
            } else {
                receiveState = ReceiveState::READING_SEQUENCE;
            }
            break;
        case ReceiveState::READING_SEQUENCE:
            sequence = value;
            calculatedCrc = updateCrc(calculatedCrc, value);
            payloadPosition = 0;
            receiveState = payloadLength == 0
                               ? ReceiveState::READING_CRC_LOW
                               : ReceiveState::READING_PAYLOAD;
            break;
        case ReceiveState::READING_PAYLOAD:
            payload[payloadPosition++] = value;
            calculatedCrc = updateCrc(calculatedCrc, value);
            if (payloadPosition >= payloadLength) {
                receiveState = ReceiveState::READING_CRC_LOW;
            }
            break;
        case ReceiveState::READING_CRC_LOW:
            receivedCrc = value;
            receiveState = ReceiveState::READING_CRC_HIGH;
            break;
        case ReceiveState::READING_CRC_HIGH:
            receivedCrc |= static_cast<uint16_t>(value) << 8;
            if (receivedCrc == calculatedCrc) {
                processPacket();
            }
            resetReceiver(value);
            break;
    }
}

void UartPacketTransport::processPacket() {
    for (const HandlerEntry &entry : handlers) {
        if (entry.handler != nullptr && entry.type == packetType) {
            entry.handler(entry.context, payload, payloadLength);
            return;
        }
    }
}

void UartPacketTransport::resetReceiver(uint8_t currentByte) {
    // Reuse a trailing marker byte as the possible start of the next frame.
    receiveState = currentByte == MARKER_0
                       ? ReceiveState::WAITING_FOR_MARKER_1
                       : ReceiveState::WAITING_FOR_MARKER_0;
    payloadPosition = 0;
    calculatedCrc = 0xFFFF;
    receivedCrc = 0;
}

uint16_t UartPacketTransport::updateCrc(uint16_t crc, uint8_t value) {
    crc ^= static_cast<uint16_t>(value) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000U)
                  ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                  : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}
