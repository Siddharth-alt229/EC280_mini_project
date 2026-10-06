#ifndef ROUTE_LDO_H
#define ROUTE_LDO_H

#define LDO_PINS_LEN 2
#define LDO_DEADTIME_TICKS 1
#include <Arduino.h>
#include <driver/mcpwm.h>

uint8_t LDO_pins[LDO_PINS_LEN] = {0};

void set_ldo_routing_pins(uint8_t en_12, uint8_t en_5, uint32_t ldo_switcher_res, uint32_t ldo_switcher_freq) {
    LDO_pins[0] = en_5;
    LDO_pins[1] = en_12;

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, en_12);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, en_5);

    mcpwm_config_t LDO_switcher;

    LDO_switcher.frequency = 100000;
    LDO_switcher.cmpr_a = 0.0;
    LDO_switcher.cmpr_b = 0.0;
    LDO_switcher.counter_mode = MCPWM_UP_COUNTER;
    LDO_switcher.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &LDO_switcher);

    mcpwm_timer_set_resolution(MCPWM_UNIT_1, MCPWM_TIMER_1, ldo_switcher_res);

    mcpwm_set_frequency(MCPWM_UNIT_1, MCPWM_TIMER_1, ldo_switcher_freq);

    mcpwm_deadtime_enable(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, LDO_DEADTIME_TICKS, LDO_DEADTIME_TICKS);

}

uint8_t LDO_volts[LDO_PINS_LEN] = {5, 12};

void route_ldo(float* global_output_val, float turnover_volt, float ldo_anti_hystersis_delay) {
    static uint32_t last_switching_time = 0;
    float sw_output = *global_output_val;
    static float out = 0;
    float out_prev = out;
    uint32_t anti_hys_cycle_delay = ldo_anti_hystersis_delay*(float)240;
    uint32_t cycle_delay = esp_cpu_get_cycle_count()-last_switching_time;
    float err = sw_output - turnover_volt;
    if(cycle_delay > anti_hys_cycle_delay && (err > 0.05f || err < -0.05f)){
        if (err > 0) {
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 100.0f);
            out = 1;
        } else{
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0f);
            out = 0;
        }
        if(out != out_prev) {
            last_switching_time = esp_cpu_get_cycle_count();
        }
    }
}

#endif