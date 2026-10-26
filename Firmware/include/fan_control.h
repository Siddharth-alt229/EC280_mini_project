#ifndef FAN_CONTROL_H
#define FAN_CONTROL_H


#include <Arduino.h>
#include <atomic>
#include <driver/mcpwm.h>

// --- PID Parameters & Constraints ---
#define K_P 0.15f            // Proportional gain (adjusts voltage based on current temp error)
#define K_I 0.01f            // Integral gain (eliminates steady-state offset over time)
#define K_D 0.05f            // Derivative gain (reacts to how quickly temp is rising)

#define TEMP_SETPOINT 30.0f  // Target ideal temperature in °C
#define MIN_FAN_VOLTAGE 3.3f // Minimum starting voltage to turn fan blades
#define MAX_FAN_VOLTAGE 12.0f// Maximum allowable fan voltage
#define BASE_VOLTAGE 5.0f    // Baseline operating voltage at setpoint
#define DEADTIME_TICKS 1

/**
 * Calculates output voltage for the fan based on temperature using PID control.
 * 
 * @param current_temp Temperature reading from ESP32 sensor (°C)
 * @param dt Time elapsed since last calculation in seconds (e.g., 0.1 for 100ms)
 * @return float Target voltage needed for the fan
 */

void set_fan_pins (uint8_t fan_pin_1, uint8_t fan_pin_2) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, fan_pin_1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, fan_pin_2);

    mcpwm_config_t fan_controller;
    fan_controller.frequency = 100000;
    fan_controller.cmpr_a = 0.0;
    fan_controller.cmpr_b = 0.0;
    fan_controller.counter_mode = MCPWM_UP_COUNTER;
    fan_controller.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &fan_controller);

    mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_0, 20000000);

    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 100000);

    mcpwm_deadtime_enable(
        MCPWM_UNIT_0,
        MCPWM_TIMER_0,
        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
        DEADTIME_TICKS,
        DEADTIME_TICKS
    );
}


float compute_pid_fan_voltage(float current_temp, float dt, float rail_turnover_volt) {
    extern uint8_t target_voltage;
    extern uint16_t global_output_val;
    extern std::atomic<uint8_t> target_ready_flag;
    extern std::atomic<uint8_t> global_out_flag;
    static float integral_error = 0.0f;
    static float previous_temp = TEMP_SETPOINT;
    static float previous_rail_voltage = 0.0f;
    static uint8_t aimed_rail_volt = 0;
    static uint16_t local_output_val = 0;

    if (global_out_flag.load(std::memory_order_acquire) == 1){
        global_out_flag.store(2, std::memory_order_seq_cst);
        local_output_val = global_output_val;
        global_out_flag.store(1, std::memory_order_seq_cst);
    }

    // 1. Calculate temperature difference (Error)
    float error = current_temp - TEMP_SETPOINT;

    // 2. Proportional term
    float p_term = K_P * error;

    // 3. Integral term with anti-windup clamp
    integral_error += error * dt;
    integral_error = min(max(integral_error, -50.0f), 50.0f);
    float i_term = K_I * integral_error;

    // 4. Derivative term
    float derivative = (current_temp - previous_temp) / dt;
    float d_term = K_D * derivative;

    previous_temp = current_temp;

    // 5. Total PID voltage sum
    float raw_voltage = BASE_VOLTAGE + p_term + i_term + d_term;
    float time_averaged_voltage = min(max(raw_voltage, MIN_FAN_VOLTAGE), MAX_FAN_VOLTAGE);

    if(time_averaged_voltage < rail_turnover_volt) {
        aimed_rail_volt = 0;
    } else {
       aimed_rail_volt = 1;
    }

    if(((target_ready_flag).load(std::memory_order_acquire) == 1) && previous_rail_voltage != aimed_rail_volt) {
        (target_ready_flag).store(0, std::memory_order_seq_cst);
        target_voltage = aimed_rail_volt;
        (target_ready_flag).store(1, std::memory_order_release);
        previous_rail_voltage = aimed_rail_volt;
    }

    float duty_cycle = max(min(100.0f, time_averaged_voltage*15920/local_output_val), 0.0f);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);

    // 6. Shutdown threshold & voltage output limits
    if (current_temp < TEMP_SETPOINT - 2.0f) {
        integral_error = 0.0f; // Reset integral term when cool
        return 0.0f;
    }

    return min(max(raw_voltage, MIN_FAN_VOLTAGE), MAX_FAN_VOLTAGE);
}

void fan_serial_test(float rail_turnover_volt) {
    extern uint8_t target_voltage;
    extern uint16_t global_output_val;
    extern std::atomic<uint8_t> target_ready_flag;
    extern std::atomic<uint8_t> global_out_flag;

    static float requested_v = MIN_FAN_VOLTAGE;
    static uint8_t sent_rail = 0;
    static uint16_t out_counts = 0;

    bool got_new = false;
    if (Serial.available()) {
        requested_v = Serial.parseFloat();
        got_new = true;
    }

    if (global_out_flag.load(std::memory_order_acquire) == 1) {
        global_out_flag.store(2, std::memory_order_seq_cst);
        out_counts = global_output_val;
        global_out_flag.store(1, std::memory_order_seq_cst);
    }

    float volts = min(max(requested_v, MIN_FAN_VOLTAGE), MAX_FAN_VOLTAGE);
    uint8_t aimed = (volts < rail_turnover_volt) ? 0 : 1;

    if (aimed != sent_rail && target_ready_flag.load(std::memory_order_acquire) == 1) {
        target_ready_flag.store(0, std::memory_order_seq_cst);
        target_voltage = aimed;
        target_ready_flag.store(1, std::memory_order_release);
        sent_rail = aimed;
    }

    float duty = max(min(100.0f, volts * 15920.0f / out_counts), 0.0f);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);

    if (got_new) {
        Serial.printf("req %.2f V -> %.2f V | rail %s | ADC %u | duty %.1f %%\n",
                      requested_v, volts, sent_rail ? "12V" : "5V", out_counts, duty);
    }
}

#endif