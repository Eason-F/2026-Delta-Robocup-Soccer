#pragma once

// Rate-limited, non-blocking serial telemetry with replaceable queued fields.

#include <Arduino.h>
#include <cstring>

class Logger {
public:
    // Fixed capacities avoid heap allocation in the control loop.
    static constexpr size_t BUFFER_SIZE = 512;
    static constexpr size_t MAX_QUEUED_FIELDS = 32;
    static constexpr size_t MAX_FIELD_NAME_LENGTH = 31;

    Logger(Print &output, uint32_t intervalMs);
    ~Logger();

    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

    template <typename Callback>
    void update(Callback callback) {
        // Flush phase: continue transmitting an existing line.
        flush();

        // Backpressure guard: do not overwrite a partially transmitted line.
        if (linePending) {
            return;
        }

        const uint32_t now = millis();

        if (intervalMs != 0 &&
            static_cast<uint32_t>(now - lastLogTime) < intervalMs) {
            return;
        }

        lastLogTime = now;

        lineBuffer.clear();
        firstField = true;

        // Assembly phase: callback fields first, then queued telemetry.
        callback(*this);

        // Append queued telemetry.
        appendQueuedFields();

        lineBuffer.println();

        linePending = true;
        sentLength = 0;

        flush();
    }

    template <typename Value>
    void log(const char *name, const Value &value) {
        if (name == nullptr) {
            return;
        }

        if (!firstField) {
            lineBuffer.print("  | ");
        }

        lineBuffer.print(name);
        lineBuffer.print(": ");
        lineBuffer.print(value);

        firstField = false;
    }

    /**
     * Queue a telemetry field.
     *
     * If the field already exists in the queue, its value is replaced
     * with the newest value.
     */
    template <typename Value>
    static void queue(const char *name, const Value &value) {
        if (activeLogger == nullptr || name == nullptr) {
            return;
        }

        activeLogger->queueValue(name, value);
    }

    void setInterval(uint32_t intervalMs);
    uint32_t getInterval() const;

private:
    class BufferPrint : public Print {
    public:
        size_t write(uint8_t value) override {
            if (length >= BUFFER_SIZE) {
                return 0;
            }

            data[length++] = value;
            return 1;
        }

        void clear() {
            length = 0;
        }

        size_t getLength() const {
            return length;
        }

        const uint8_t *getData() const {
            return data;
        }

        size_t getRemainingCapacity() const {
            return BUFFER_SIZE - length;
        }

    private:
        // Print-compatible fixed buffer used for formatting arbitrary values.
        uint8_t data[BUFFER_SIZE];
        size_t length = 0;
    };

    template <typename Value>
    void queueValue(const char *name, const Value &value) {
        // Replacement path: update an existing field instead of duplicating it.
        for (size_t i = 0; i < queuedFieldCount; ++i) {
            if (strcmp(queuedFieldNames[i], name) == 0) {
                /*
                 * The field already exists.
                 *
                 * We don't append another copy. Instead, rebuild
                 * the queued buffer using the new value.
                 */
                rebuildQueuedField(i, name, value);
                return;
            }
        }

        // Capacity guard: silently discard new fields when the queue is full.
        if (queuedFieldCount >= MAX_QUEUED_FIELDS) {
            return;
        }

        // Storage phase: retain a bounded field name and its formatted value.
        strncpy(
            queuedFieldNames[queuedFieldCount],
            name,
            MAX_FIELD_NAME_LENGTH
        );

        queuedFieldNames[queuedFieldCount][MAX_FIELD_NAME_LENGTH] = '\0';

        ++queuedFieldCount;

        rebuildQueuedBuffer(
            queuedFieldCount - 1,
            value
        );
    }

    template <typename Value>
    void rebuildQueuedField(
        size_t fieldIndex,
        const char *name,
        const Value &value
    ) {
        /*
         * Save the new value by rebuilding the entire queued buffer.
         *
         * Since BufferPrint only stores formatted text, the easiest
         * way to replace a value of arbitrary type is to reconstruct
         * all queued fields.
         */
        queuedFieldValues[fieldIndex].clear();

        queuedFieldValues[fieldIndex].print(value);

        rebuildQueuedBuffer();
    }

    template <typename Value>
    void rebuildQueuedBuffer(
        size_t newlyAddedIndex,
        const Value &newValue
    ) {
        /*
         * Store the new value in its own BufferPrint.
         */
        queuedFieldValues[newlyAddedIndex].clear();
        queuedFieldValues[newlyAddedIndex].print(newValue);

        rebuildQueuedBuffer();
    }

    void rebuildQueuedBuffer();

    void appendQueuedFields();
    void clearQueuedFields();
    void flush();

    static Logger *activeLogger;

    Print &output;

    BufferPrint lineBuffer;
    BufferPrint queuedFields;

    /*
     * Each queued field has its own formatted value.
     *
     * This allows queue("RPM", 1200), followed by
     * queue("RPM", 1250), to replace the old value.
     */
    BufferPrint queuedFieldValues[MAX_QUEUED_FIELDS];

    char queuedFieldNames[
        MAX_QUEUED_FIELDS
    ][MAX_FIELD_NAME_LENGTH + 1];

    uint32_t intervalMs;
    uint32_t lastLogTime;

    size_t sentLength = 0;
    size_t queuedFieldCount = 0;

    bool linePending = false;
    bool firstField = true;
};
