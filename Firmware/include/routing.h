#ifndef ROUTING_H
#define ROUTING_H
#include <Arduino.h>
#include <stdlib.h>
#include <driver/mcpwm.h>

#define DEADTIME_TICKS 1 //1 tick = 500ns

void set_routing_pins(uint8_t buck_switch_pin, 
    uint8_t boost_switch_pin, 
    uint32_t enable_timer_res, 
    uint32_t enable_switching_freq
){

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, buck_switch_pin);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, boost_switch_pin);

    mcpwm_config_t buck_boost_switcher;
    buck_boost_switcher.frequency = 100000;
    buck_boost_switcher.cmpr_a = 0.0;
    buck_boost_switcher.cmpr_b = 0.0;
    buck_boost_switcher.counter_mode = MCPWM_UP_COUNTER;
    buck_boost_switcher.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &buck_boost_switcher);

    mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_0, enable_timer_res);

    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, enable_switching_freq);

    mcpwm_deadtime_enable(
        MCPWM_UNIT_0,
        MCPWM_TIMER_0,
        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
        DEADTIME_TICKS,
        DEADTIME_TICKS
    );
}

struct converterState {
  uint8_t buck_state;
  uint8_t boost_state;
};

converterState route(float out_desired, 
    long int anti_hysteresis_delay, 
    float* global_input_val
){
    
    converterState converter_state;
    static uint32_t last_switching_time = 0;
    static uint8_t buck_switch = 0;
    static uint8_t boost_switch = 0;
    uint8_t last_buck_state = 0;
    uint8_t last_boost_state = 0;
    float input_voltage = 0;
    uint8_t en = 0;
    uint32_t now = 0;
    uint32_t cycle_delay = 0;
    float time_delay = 0;

    
    input_voltage = *global_input_val;

    last_buck_state = buck_switch;
    last_boost_state = boost_switch;

    
    now = esp_cpu_get_cycle_count();
    cycle_delay = now-last_switching_time;
    time_delay = cycle_delay * (float)0.004166;
    

    en = (time_delay > anti_hysteresis_delay) ? 0b11111111: 0;

    buck_switch = en ? ((input_voltage > out_desired) ? 1 : 0): last_buck_state;
    boost_switch = (~buck_switch) & 0b00000001;
    converter_state.buck_state = buck_switch;
    converter_state.boost_state = boost_switch;

    if(buck_switch != last_buck_state){
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (buck_switch*100));
    }


    last_switching_time = (buck_switch == last_buck_state && boost_switch == last_boost_state) ? last_switching_time : esp_cpu_get_cycle_count();

    return converter_state;
}

#endif