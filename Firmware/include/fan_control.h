#ifndef FAN_CONTROL_H
#define FAN_CONTROL_H


#include <Arduino.h>
#include <atomic>

#define MIN_FAN_VOLTAGE 0.0f // Minimum starting voltage to turn fan blades
#define MAX_FAN_VOLTAGE 12.0f// Maximum allowable fan voltage

void fan_serial_test() {
    extern std::atomic<float> target_voltage;
    extern std::atomic<uint16_t> global_output_val;

    static float requested_v = MIN_FAN_VOLTAGE;
    static float sent_rail = 0;
    static uint16_t out_counts = 0;

    bool got_new = false;
    if (Serial.available()) {
        requested_v = Serial.parseFloat();
        got_new = true;
    }

    float volts = min(max(requested_v, MIN_FAN_VOLTAGE), MAX_FAN_VOLTAGE);

    if (volts != target_voltage.load(std::memory_order_acquire)) {
        target_voltage.store(volts, std::memory_order_release);
    }

    if (got_new) {
        Serial.printf("Initiating change to %.2f V\n",
                      volts);
    }
}

#endif