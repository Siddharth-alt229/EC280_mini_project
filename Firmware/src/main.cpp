#include <Arduino.h>
#include <driver/dac_oneshot.h>
#include "adc_setup.h"
#include "routing.h"
#include "converter_output_regulator.h"

#define BUF_SIZE 128
#define INPUT_PROBE_CHANNEL_ADC1 0
#define OUTPUT_PROBE_CHANNEL_ADC1 1
#define ADC_TIMEOUT_MS 5
#define BUCK_ENABLE_PIN 14
#define BOOST_ENABLE_PIN 27
#define CONVERTER_CONTROL_PIN 12
#define ANTI_HYSTERISIS_DELAY 50
#define ANTI_WINDUP_CLAMP 2

float global_input_val = 0;
float global_output_val = 0;

uint8_t global_adc_buf[BUF_SIZE];
uint32_t copied_out_len = 0;

float out_desired;

dac_oneshot_handle_t dac_handle;

void setup() {
  // put your setup code here, to run once:
  dac_oneshot_config_t cfg = { .chan_id = DAC_CHAN_0 };
  dac_oneshot_new_channel(&cfg, &dac_handle);
  set_adc_pins(INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1);
  set_routing_pins(BUCK_ENABLE_PIN, BOOST_ENABLE_PIN);
  set_converter_pins(INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1, CONVERTER_CONTROL_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  read_adc(handle, global_adc_buf, BUF_SIZE, &copied_out_len, ADC_TIMEOUT_MS, &global_input_val, &global_output_val, INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1);
  converterState routing_state;
  routing_state = route(out_desired, ANTI_HYSTERISIS_DELAY, &global_input_val);
  controlSig out_sig;
  out_sig = converter_out_regulator(out_desired, routing_state.buck_state, &global_input_val, &global_output_val, ANTI_WINDUP_CLAMP);

  dac_oneshot_output_voltage(dac_handle, (uint8_t)(out_sig.control_sig*255));

}