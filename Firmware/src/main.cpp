#include <Arduino.h>
#include <driver/dac_oneshot.h>
#include "adc_setup.h"
#include "routing.h"
#include "converter_output_regulator.h"
#include "esp_random.h"
#include <atomic>

#define BUF_SIZE 128
#define INPUT_PROBE_CHANNEL_ADC1 0 //GPIO 36 (VP)
#define OUTPUT_PROBE_CHANNEL_ADC1 1 //GPIO 37
#define ADC_TIMEOUT_MS 5
#define BUCK_ENABLE_PIN 14
#define BOOST_ENABLE_PIN 27
#define CONVERTER_CONTROL_PIN 12
#define ANTI_HYSTERISIS_DELAY 20
#define ANTI_WINDUP_CLAMP 2
#define MAXIMUM_STEP_MAGNITUDE 1.0f
#define MINIMUM_STEP_MAGNITUDE 0.1f

std::atomic<uint8_t> target_ready_flag(0);
std::atomic<uint8_t> step_ready_flag(0);

float target_voltage = 5;
float requested_step = 0;

dac_oneshot_handle_t dac_handle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dac_oneshot_config_t cfg = { .chan_id = DAC_CHAN_0 }; // GPIO 25
  dac_oneshot_new_channel(&cfg, &dac_handle);
  set_adc_pins(INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1);
  set_routing_pins(BUCK_ENABLE_PIN, BOOST_ENABLE_PIN);
  set_converter_pins(INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1, CONVERTER_CONTROL_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  static float global_input_val = 0;
  static float global_output_val = 0;
  static uint8_t global_adc_buf[BUF_SIZE];
  static uint32_t copied_out_len = 0;
  float random_float = (float)esp_random()/(float)UINT32_MAX;
  float second_random_float = (float)esp_random()/(float)UINT32_MAX;
  static float out_desired = 0;
  static float local_target_voltage = 0;
  static float local_requested_step = 0;
  static uint8_t change = 0;

  if(target_ready_flag.load(std::memory_order_acquire) == 1){
    local_target_voltage = target_voltage;
  }

  if(step_ready_flag.load(std::memory_order_acquire) == 1){
    local_requested_step = requested_step;
  }

  float err = out_desired-local_target_voltage;

  change = (err < -0.05f || err > 0.05f) ? 1 : 0;

  if(change) {
    float differential_pre = local_target_voltage-out_desired;
    float step = (local_requested_step>0) ? min(max(local_requested_step, MINIMUM_STEP_MAGNITUDE), MAXIMUM_STEP_MAGNITUDE): (max(min(local_requested_step, -MINIMUM_STEP_MAGNITUDE), -MAXIMUM_STEP_MAGNITUDE));
    step = (differential_pre*step > 0) ? step: (step>0 ? -MINIMUM_STEP_MAGNITUDE: MINIMUM_STEP_MAGNITUDE);
    step = (differential_pre == 0) ? 0: step;
    float differential_post = (differential_pre-step);
    if((differential_post<0.05f && differential_post>(-0.05f))){
      change = 0;
      out_desired = local_target_voltage;
    } else{
      out_desired += step;
    }
  }
  random_float = max(random_float, (float)0.90);
  read_adc(handle, global_adc_buf, BUF_SIZE, &copied_out_len, ADC_TIMEOUT_MS, &global_input_val, &global_output_val, INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1);
  converterState routing_state;
  float input = global_input_val;
  routing_state = route(out_desired, ANTI_HYSTERISIS_DELAY, &global_input_val);
  controlSig out_sig = {0.0f};
  out_sig = converter_out_regulator(out_desired, routing_state.buck_state, &global_input_val, &global_output_val, ANTI_WINDUP_CLAMP, 0);
  dac_oneshot_output_voltage(dac_handle, (uint8_t)(out_sig.control_sig*255));
}