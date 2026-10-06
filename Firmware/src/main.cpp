#include <Arduino.h>
#include <driver/dac_oneshot.h>
#include "adc_setup.h"
#include "routing.h"
#include "converter_output_regulator.h"
#include "route_ldo.h"
#include <atomic>

#define BUF_SIZE 128
#define INPUT_PROBE_CHANNEL_ADC1 0 //GPIO 36 (VP)
#define OUTPUT_PROBE_CHANNEL_ADC1 1 //GPIO 37
#define ADC_TIMEOUT_MS 5
#define BUCK_ENABLE_PIN 14
#define BOOST_ENABLE_PIN 27
#define CONVERTER_CONTROL_PIN 13
#define ANTI_HYSTERISIS_DELAY 20
#define ANTI_WINDUP_CLAMP 2
#define MAXIMUM_STEP_MAGNITUDE 1.0f
#define MINIMUM_STEP_MAGNITUDE 0.1f
#define INTEGRAL_FEEDBACK_FACTOR 0.0f
#define PROPORTIONAL_FEEDBACK_FACTOR 0.0f
#define MOS_SWITCHING_FREQ 100000
#define ENABLE_TIMER_RES 2000000
#define ENABLE_SWITCHING_FREQ 100000
#define V12_LDO_INPUT_PIN 10
#define V5_LDO_INPUT_PIN 11
#define TURNOVER_VOLTAGE 5.3f
#define LDO_SWITCHER_TIMER_RES 2000000
#define LDO_SWITCHER_FREQ 100000
#define LDO_ANTI_HYSTERSIS_DELAY 0.1f


std::atomic<uint8_t> target_ready_flag(0); // This variable signals exists as a signal telling the core whether target_voltage is safe to read or if its currently being updated
std::atomic<uint8_t> step_ready_flag(0); //This variable signals exists as a signal telling the core whether requested_step is safe to read or if its currently being updated

float target_voltage = 0;
float requested_step = 0;

dac_oneshot_handle_t dac_handle;

void setup() {
  // put your setup code here, to run once:
  pinMode(V12_LDO_INPUT_PIN, OUTPUT);
  pinMode(V5_LDO_INPUT_PIN, OUTPUT);
  dac_oneshot_config_t cfg = { .chan_id = DAC_CHAN_0 }; // GPIO 25
  dac_oneshot_new_channel(&cfg, &dac_handle);
  set_adc_pins(INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1);
  set_routing_pins(BUCK_ENABLE_PIN, BOOST_ENABLE_PIN, ENABLE_TIMER_RES, ENABLE_SWITCHING_FREQ);
  set_converter_pins(INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1, CONVERTER_CONTROL_PIN, MOS_SWITCHING_FREQ);
  set_ldo_routing_pins(V12_LDO_INPUT_PIN, V5_LDO_INPUT_PIN, LDO_SWITCHER_TIMER_RES, LDO_SWITCHER_FREQ);
}

void loop() {
  // put your main code here, to run repeatedly:
  static float global_input_val = 0;
  static float global_output_val = 0;
  static uint8_t global_adc_buf[BUF_SIZE];
  static uint32_t copied_out_len = 0;
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
    float step = 
    (local_requested_step>0) ? min(max(local_requested_step, MINIMUM_STEP_MAGNITUDE), MAXIMUM_STEP_MAGNITUDE): 
    (max(min(local_requested_step, -MINIMUM_STEP_MAGNITUDE), -MAXIMUM_STEP_MAGNITUDE));
    //The above line exists as a protection to clamp the step sizes between a minimum and maximum defined above (at the very top of the program).
    step = (differential_pre*step > 0) ? step: (step>0 ? -MINIMUM_STEP_MAGNITUDE: MINIMUM_STEP_MAGNITUDE);
    step = (differential_pre == 0) ? 0: step;
    //These lines ensure the step moves the converter's output towards the target. In case it doesn't the system fixes a default rate of MINIMUM_STEP_MAGNITUDE in whichever direction the target is.
    float differential_post = (differential_pre-step);
    if((differential_post<0.05f && differential_post>(-0.05f))){
      change = 0;
      out_desired = local_target_voltage;
    } else{
      out_desired += step;
    }
  }
  read_adc(handle, global_adc_buf, BUF_SIZE, &copied_out_len, ADC_TIMEOUT_MS, &global_input_val, &global_output_val, INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1);
  converterState routing_state;
  routing_state = route(out_desired, ANTI_HYSTERISIS_DELAY, &global_input_val);
  controlSig out_sig = {0.0f};
  out_sig = converter_out_regulator(out_desired, routing_state.buck_state, &global_input_val, &global_output_val, ANTI_WINDUP_CLAMP, 0, INTEGRAL_FEEDBACK_FACTOR, PROPORTIONAL_FEEDBACK_FACTOR);
  route_ldo(&global_output_val, TURNOVER_VOLTAGE, LDO_ANTI_HYSTERSIS_DELAY);
  dac_oneshot_output_voltage(dac_handle, (uint8_t)(out_sig.control_sig*255));
  //The control signal is reflected on GPIO 25 through the inbuilt DAC.
}