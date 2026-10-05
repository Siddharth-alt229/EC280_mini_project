#ifndef converter_output_regulator_h
#define converter_output_regulator_h

#include <Arduino.h>
#include <driver/ledc.h>
#include <adc_setup.h>

void set_converter_pins(uint8_t input_probe_channel_ADC1, uint8_t output_probe_channel_ADC1, uint8_t control_pin){

    ledc_timer_config_t ledc_timer {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_9_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 100000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num = control_pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&ledc_channel);

}



struct controlSig {
    float control_sig;
    float drive_sig;
};

controlSig converter_out_regulator(float out_desired, int buck_state, float* global_input_val, float* global_output_val, float anti_windup_clamp, uint8_t rst){
    float input_voltage = 0;
    //float integral_feedback = 0.0001;
    float integral_feedback = 0;
    //float proportional_feedback = 0.001;
    float proportional_feedback = 0;
    float previous_output = 0;
    static float duty_cycle = 0;
    static float integral_term = 0;
    float integral_correction = 0;
    float proportional_correction = 0;
    uint8_t en = 0;
    float differential = 0;
    float differential_abs = 0;
    int differential_int = 0;
    static float control_signal = 0;
    static uint16_t drive_signal = 0;
    controlSig final_drive;
    float previous_control_signal = 0;
    float control_signal_int = 0;
    static uint8_t last_buck_state = 0;
    float correction_factor = 0;

    input_voltage = *global_input_val;

    previous_control_signal = control_signal;
    previous_output = *global_output_val;

    differential = (out_desired - previous_output);
    differential_abs = (differential>0) ? differential : -differential;
    differential_int = (int)(differential_abs*8.0f);

    en = ((differential_int < 5) && previous_output >= 12) ? 0 : 1;

    if(en){
        integral_term = ((buck_state == last_buck_state) && (differential_abs<2.0f) && (!rst)) ? (integral_term + differential): 0;
        //integral_term = min(max(integral_term, -anti_windup_clamp), anti_windup_clamp);
        integral_correction = integral_term  * integral_feedback;
        proportional_correction = (differential * proportional_feedback);
        correction_factor = integral_correction + proportional_correction;
        if(buck_state){
            duty_cycle = (input_voltage>0) ? min((max(out_desired, float(0)) / input_voltage), (float)1): 0;
        } else{
            duty_cycle = (out_desired>0) ? min(max((float)1-(input_voltage/out_desired), (float)0), (float)1.0): 0;
        }
        control_signal_int = max((float)0, min((float)1, duty_cycle + correction_factor));
        if(!(((previous_control_signal - control_signal_int) < (float)0.01) && (previous_control_signal-control_signal_int) > (float)(-0.01))){
            control_signal = control_signal_int;
            drive_signal = (uint16_t)(control_signal*511);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, drive_signal);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        }
    }

    last_buck_state = buck_state;

    final_drive.control_sig = control_signal;
    final_drive.drive_sig = drive_signal;

    return final_drive;
}

#endif