#include <Arduino.h>
#include <driver/dac_oneshot.h>
#include "adc_setup.h"
#include "converter_output_regulator.h"
#include "fan_control.h"
#include <atomic>

#define BUF_SIZE 128
#define INPUT_PROBE_CHANNEL_ADC1 0 //GPIO 36 (VP)
#define OUTPUT_PROBE_CHANNEL_ADC1 1 //GPIO 37
#define ADC_TIMEOUT_MS 5
#define CONVERTER_CONTROL_PIN 13
#define MOS_SWITCHING_FREQ 10000

std::atomic<float> target_voltage = 0;
std::atomic<uint16_t> global_output_val = 0;

dac_oneshot_handle_t dac_handle;

void get_voltage(void* pvParams) {
  Serial.begin(115200);
  Serial.setTimeout(50);
  for(;;) {
    fan_serial_test();
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
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint16_t global_input_val = 0;
  static uint16_t local_output_val = 0;
  static uint8_t global_adc_buf[BUF_SIZE];
  static uint32_t copied_out_len = 0;
  static float local_target_voltage = 0;
  static uint16_t out_sig;

  float prev_target_voltage = local_target_voltage;

  local_target_voltage = target_voltage.load(std::memory_order_acquire);
  read_adc(handle, global_adc_buf, BUF_SIZE, &copied_out_len, ADC_TIMEOUT_MS, &global_input_val, &local_output_val, INPUT_PROBE_CHANNEL_ADC1, OUTPUT_PROBE_CHANNEL_ADC1);
  global_output_val.store(local_output_val, std::memory_order_release);
  out_sig = converter_out_regulator(local_target_voltage, &global_input_val);
  
  dac_oneshot_output_voltage(dac_handle, (uint8_t)(out_sig*0.062f));
  //The control signal is reflected on GPIO 25 through the inbuilt DAC.
}