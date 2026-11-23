# ESP32 Auto-Routing Buck/Boost Converter Controller

Firmware for an ESP32 that regulates the output of a DC-DC converter by automatically choosing between **buck** and **boost** operation based on the live input voltage, then driving a single PWM-controlled power stage with a feedforward duty cycle (plus optional PI trim). Setpoint changes are slewed so the output ramps rather than jumps.

Built on the Arduino framework using the ESP-IDF peripheral drivers directly: continuous-mode ADC, MCPWM, LEDC, and the one-shot DAC.

---

## Features

- **Dual-channel continuous ADC sampling** of the input and output rails (ADC1, 12-bit, 80 kHz).
- **Feedforward duty-cycle calculation** from the ideal buck/boost transfer functions, with optional PI correction and anti-windup.
- **Analog monitor output** of the control signal on the ESP32 DAC.

---

## How it works

```
 Vin probe ──┐                                  
             ├─► ADC (continuous)
 Vout probe ─┘            │                       
                          │
                          └────► converter_out_regulator() ─► PWM gate drive (GPIO13, LEDC, 100 kHz)
                                          │
                                          └─► DAC (GPIO25): control signal monitor

 target_voltage ─► slew limiter (main loop) ─► out_desired ─► route() / regulator
```

Each pass through `loop()`:

1. **`read_adc()`**: drains the ADC buffer and updates the latest input and output voltages.
2. **`converter_out_regulator()`**: computes the duty cycle and writes it to the LEDC PWM channel.
3. The resulting control signal (0 to 1) is mirrored to the DAC as an 8-bit analog level.


### Duty-cycle control (`converter_output_regulator.h`)

Feedforward duty from the ideal transfer functions:

| Mode  | Duty cycle              |
|-------|-------------------------|
| Buck  | `D = Vout* / Vin`       |
| Boost | `D = 1 - Vin / Vout*`   |


- **Resolution**: 12-bit LEDC duty (0 to 4095) at `MOS_SWITCHING_FREQ` (10kHz).

### ADC scaling (`adc_setup.h`)

The ADC reading from 0-4095 is assumed to signify a voltage between 0-14V, and correspondingly it is assumed that the input voltage lies within this range.

---

## Hardware

Target: **classic ESP32** (the pin map relies on ESP32 ADC1 channels and the on-chip DAC).

| Function                     | GPIO | Peripheral          |
|------------------------------|------|---------------------|
| Input voltage probe          | 36 (VP) | ADC1 channel 0   |
| Output voltage probe         | 37   | ADC1 channel 1      |
| Converter PWM (gate drive)   | 13   | LEDC, high-speed ch 0 |
| Control-signal monitor       | 25   | DAC channel 0       |

**Notes**

- Both probe inputs must be divided down to stay within the ADC range at 12 dB attenuation.
- GPIO14 can toggle briefly during boot. Add pull-downs on gate-driver enable lines so the power stage stays off until the firmware is running.
- Verify dead-time and gate-drive behavior on a scope before connecting a real load.

---

## Configuration

All tunables are `#define`s at the top of `main.cpp`.

| Macro                          | Default   | Description |
|--------------------------------|-----------|-------------|
| `INPUT_PROBE_CHANNEL_ADC1`     | `0`       | ADC1 channel for Vin |
| `OUTPUT_PROBE_CHANNEL_ADC1`    | `1`       | ADC1 channel for Vout |
| `ADC_TIMEOUT_MS`               | `5`       | Timeout for each ADC read |
| `BUF_SIZE`                     | `128`     | ADC read buffer size (bytes) |
| `BUCK_ENABLE_PIN`              | `14`      | Buck enable GPIO |
| `BOOST_ENABLE_PIN`             | `27`      | Boost enable GPIO |
| `CONVERTER_CONTROL_PIN`        | `13`      | PWM gate-drive GPIO |
| `MOS_SWITCHING_FREQ`           | `10000`  | Converter PWM frequency (Hz) |

---

## Usage

Set the desired output voltage by writing `target_voltage`, which is an atomic float.

---

## Project structure

```
src/
├── main.cpp                      # Setup, setpoint slew limiter, main control loop
├── adc_setup.h                   # Continuous ADC init and sample decoding
├── routing.h                     # Buck/boost mode selection and MCPWM enable outputs
└── converter_output_regulator.h  # Feedforward + PI duty-cycle controller, LEDC PWM
```

`converter_output_regulator.h` includes `<adc_setup.h>` with angle brackets, so the headers need to be on the include path (for example in PlatformIO's `include/` or `lib/` directory).

---