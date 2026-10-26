#ifndef ROUTE_LINEAR_H
#define ROUTE_LINEAR_H

#define LDO_PINS_LEN 2
#define LDO_DEADTIME_TICKS 1
#include <Arduino.h>
#include <driver/mcpwm.h>

uint8_t LDO_pins[LDO_PINS_LEN] = {0};

void set_linear_routing_pins(uint8_t en_12, uint8_t en_5, uint32_t ldo_switcher_res, uint32_t ldo_switcher_freq) {
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

uint8_t route_linear(uint8_t* desired_out_val, uint32_t anti_hys_cycle_delay) {
    static uint32_t last_switching_time = 0;
    uint8_t returnable = 0;
    uint8_t sw_output = *desired_out_val;
    static uint8_t out = 0;
    uint8_t out_prev = out;
    uint32_t cycle_delay = esp_cpu_get_cycle_count()-last_switching_time;
    if(sw_output != out_prev){
        if(cycle_delay > anti_hys_cycle_delay){
            mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, sw_output*100.0f);
            out = sw_output;
            last_switching_time = esp_cpu_get_cycle_count();
        } else {
            returnable = 1;
        }
    }
    return returnable;
}

#endif