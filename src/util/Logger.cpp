// Buffered telemetry assembly and non-blocking output flushing.
#include <util/Logger.hpp>

Logger *Logger::activeLogger = nullptr;

Logger::Logger(Print &output, uint32_t intervalMs)
    : output(output),
      intervalMs(intervalMs),
      lastLogTime(millis()) {

    activeLogger = this;
}

Logger::~Logger() {
    if (activeLogger == this) {
        activeLogger = nullptr;
    }
}

void Logger::rebuildQueuedBuffer() {
    // Reconstruct the flat output after a queued value is added or replaced.
    queuedFields.clear();

    for (size_t i = 0; i < queuedFieldCount; ++i) {
        if (i > 0) {
            queuedFields.print("  | ");
        }

        queuedFields.print(queuedFieldNames[i]);
        queuedFields.print(": ");

        const BufferPrint &value = queuedFieldValues[i];

        const size_t remaining =
            queuedFields.getRemainingCapacity();

        const size_t copyLength =
            min(
                value.getLength(),
                remaining
            );

        for (size_t j = 0; j < copyLength; ++j) {
            queuedFields.write(value.getData()[j]);
        }
    }
}

void Logger::appendQueuedFields() {
    if (queuedFieldCount == 0) {
        return;
    }

    if (!firstField) {
        lineBuffer.print("  | ");
    }

    const size_t available =
        lineBuffer.getRemainingCapacity();

    const size_t queuedLength =
        queuedFields.getLength();

    const size_t copyLength =
        min(
            queuedLength,
            available
        );

    for (size_t i = 0; i < copyLength; ++i) {
        lineBuffer.write(
            queuedFields.getData()[i]
        );
    }

    firstField = false;

    clearQueuedFields();
}

void Logger::clearQueuedFields() {
    queuedFields.clear();

    for (size_t i = 0; i < queuedFieldCount; ++i) {
        queuedFieldValues[i].clear();
    }

    queuedFieldCount = 0;
}

void Logger::flush() {
    // Write only what the destination currently accepts to avoid blocking control.
    if (!linePending) {
        return;
    }

    const size_t totalLength =
        lineBuffer.getLength();

    if (sentLength >= totalLength) {
        sentLength = 0;
        linePending = false;
        return;
    }

    const size_t remaining =
        totalLength - sentLength;

    const int available =
        output.availableForWrite();

    if (available <= 0) {
        return;
    }

    const size_t writeLength =
        min(
            remaining,
            static_cast<size_t>(available)
        );

    const size_t written =
        output.write(
            lineBuffer.getData() + sentLength,
            writeLength
        );

    sentLength += written;

    if (sentLength >= totalLength) {
        sentLength = 0;
        linePending = false;
    }
}

void Logger::setInterval(uint32_t newIntervalMs) {
    intervalMs = newIntervalMs;
}

uint32_t Logger::getInterval() const {
    return intervalMs;
}
