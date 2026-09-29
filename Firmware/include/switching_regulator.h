#ifndef switching_regulator_h
#define switching_regulator_h

#include <Arduino.h>
#include <routing.h>
#include <converter_output_regulator.h>

struct converter {
    uint8_t type; // Buck = 0, Boost = 1
    uint8_t switch_pin;
    uint8_t control_pin;
};

struct switching_state {
    converterState con_state;
    uint8_t commanded_regulator_output;
    uint8_t actual_regulator_output;
    float duty_cycle;
    uint32_t timestamp;
};

struct packet {
    uint16_t start_marker = 0xABCD;
    switching_state data;
};

void switching_output_setup(uint8_t input_line, uint8_t buck_routing_pin, uint8_t boost_routing_pin, uint8_t switching_output_probe, uint8_t converter_control_pin) {
    set_routing_pins(input_line, buck_routing_pin, boost_routing_pin);
    set_converter_pins(input_line, switching_output_probe, converter_control_pin);
}

packet switching_output_control () {}

#endif