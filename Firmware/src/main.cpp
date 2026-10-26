#include <Arduino.h>
#include <driver/dac_oneshot.h>
#include "adc_setup.h"
#include "converter_output_regulator.h"
#include "route_linear.h"
#include "fan_control.h"
#include <atomic>

#define BUF_SIZE 128
#define INPUT_PROBE_CHANNEL_ADC1 0 //GPIO 36 (VP)
#define OUTPUT_PROBE_CHANNEL_ADC1 1 //GPIO 37
#define ADC_TIMEOUT_MS 5
#define FAN_PIN_1 14
#define FAN_PIN_2 27
#define CONVERTER_CONTROL_PIN 13
#define ANTI_HYSTERISIS_DELAY 20
#define MOS_SWITCHING_FREQ 1000
#define V12_LDO_INPUT_PIN 32
#define V5_LDO_INPUT_PIN 33
#define TURNOVER_VOLTAGE 5.3f
#define LDO_SWITCHER_TIMER_RES 20000000
#define LDO_SWITCHER_FREQ 100000
#define LDO_ANTI_HYSTERSIS_DELAY_CYCLES 100000


std::atomic<uint8_t> target_ready_flag(1); // This variable signals exists as a signal telling the core whether target_voltage is safe to read or if its currently being updated
std::atomic<uint8_t> global_out_flag(1);

uint8_t target_voltage = 0;
uint16_t global_output_val = 0;

dac_oneshot_handle_t dac_handle;

void get_voltage(void* pvParams) {
  Serial.begin(115200);
  Serial.setTimeout(50);
  set_fan_pins(FAN_PIN_1, FAN_PIN_2);
  for(;;) {
    fan_serial_test(TURNOVER_VOLTAGE);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  // put your setup code here, to run once:
  xTaskCreatePinnedToCore(
    get_voltage,
    "getVoltage",
    6144,
    NULL,
    1,
    NULL,
    0
  );
  dac_oneshot_config_t cfg = { .chan_id = DAC_CHAN_0 }; // GPIO 25
  dac_oneshot_new_channel(&cfg, &dac_handle);
  set_adc_pins(INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1);
  set_converter_pins(INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1, CONVERTER_CONTROL_PIN, MOS_SWITCHING_FREQ);
  set_linear_routing_pins(V12_LDO_INPUT_PIN, V5_LDO_INPUT_PIN, LDO_SWITCHER_TIMER_RES, LDO_SWITCHER_FREQ);
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint16_t global_input_val = 0;
  static uint16_t local_output_val = 0;
  static uint8_t global_adc_buf[BUF_SIZE];
  static uint32_t copied_out_len = 0;
  static uint8_t local_target_voltage = 0;
  static uint16_t out_sig;

  uint8_t prev_target_voltage = local_target_voltage;

  if(target_ready_flag.load(std::memory_order_acquire) == 1){
    target_ready_flag.store(2, std::memory_order_seq_cst);
    local_target_voltage = target_voltage;
    target_ready_flag.store(1, std::memory_order_release);
  }
  read_adc(handle, global_adc_buf, BUF_SIZE, &copied_out_len, ADC_TIMEOUT_MS, &global_input_val, &local_output_val, INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1);
  if(global_out_flag.load(std::memory_order_acquire) == 1){
    global_out_flag.store(0, std::memory_order_seq_cst);
    global_output_val = local_output_val;
    global_out_flag.store(1, std::memory_order_seq_cst);
  }
  if(prev_target_voltage >= local_target_voltage){
    uint8_t routed = route_linear(&local_target_voltage, LDO_ANTI_HYSTERSIS_DELAY_CYCLES);
    if(!routed) {
      out_sig = converter_out_regulator(local_target_voltage, &global_input_val);
    }
  } else {
    out_sig = converter_out_regulator(local_target_voltage, &global_input_val);
    route_linear(&local_target_voltage, LDO_ANTI_HYSTERSIS_DELAY_CYCLES);
  }
  dac_oneshot_output_voltage(dac_handle, (uint8_t)(out_sig*0.4990f));
  //The control signal is reflected on GPIO 25 through the inbuilt DAC.
}