#ifndef adc_setup_h
#define adc_setup_h

#include <Arduino.h>
#include <esp_adc/adc_continuous.h>

adc_continuous_handle_t handle = NULL;

void set_adc_pins(uint8_t input_probe_channel_ADC1, uint8_t output_probe_channel_ADC1) {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 8,
        .conv_frame_size = 4,
        .flags = {.flush_pool = 1}
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
    adc_digi_pattern_config_t inp_settings = {
        .atten = ADC_ATTEN_DB_12,
        .channel = input_probe_channel_ADC1,
        .unit = ADC_UNIT_1,
        .bit_width = 12
    };
    adc_digi_pattern_config_t out_settings = {
        .atten = ADC_ATTEN_DB_12,
        .channel = output_probe_channel_ADC1,
        .unit = ADC_UNIT_1,
        .bit_width = 12
    };

    adc_digi_pattern_config_t configs[2] = {inp_settings, out_settings};

    adc_continuous_config_t adc1_settings = {
        .pattern_num = 2,
        .adc_pattern = configs,
        .sample_freq_hz = 80000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1
    };
    ESP_ERROR_CHECK(adc_continuous_config(handle, &adc1_settings));
    adc_continuous_start(handle);
}

void read_adc(adc_continuous_handle_t handle, uint8_t* buf, uint8_t buf_size, uint32_t* copied_out_len, uint32_t timeout_ms, uint16_t* global_input_val, uint16_t* global_output_val, uint8_t input_probe_channel_ADC1, uint8_t output_probe_channel_ADC1) {
    ESP_ERROR_CHECK(adc_continuous_read (handle, buf, buf_size, copied_out_len, timeout_ms));
    uint16_t val_raw;
    uint8_t channel = 0;
    uint16_t val = 0;
    for(int i=0; i<(int)(*copied_out_len/2); i++){
        channel = (buf[1] & 0b11110000)>>4;
        val_raw = ((uint16_t)(buf[0])) | (uint16_t)((buf[1]&0b00001111)<<8);
        val = val_raw;
        if(channel == input_probe_channel_ADC1){
           *global_input_val = val;
        } else if(channel == output_probe_channel_ADC1) {
            *global_output_val = val;
        }
        buf += 2;
    }
}

#endif