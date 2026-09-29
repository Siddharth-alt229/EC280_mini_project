# Buck/Boost Voltage Regulation System

A real-time ESP32-based buck/boost converter control system that dynamically selects between **buck** and **boost** operation and regulates the converter output toward a configurable target voltage.

The system combines:

* Continuous ADC-based voltage measurement
* Automatic buck/boost mode selection
* Anti-hysteresis switching logic
* Feed-forward duty-cycle calculation
* Proportional + integral output correction
* Integral anti-windup
* 100 kHz PWM generation
* Hardware dead-time control
* Binary telemetry over UART
* ESP32 CPU cycle-counter timing for low-overhead switching decisions

The architecture is intended for applications where the converter input voltage may move above or below the desired output voltage and the controller must automatically select the appropriate power-conversion mode.

---

## Architecture

```text
                     ┌─────────────────────┐
                     │      ADC System     │
                     │                     │
Input Voltage ──────►│ ADC1 continuous     │
Output Voltage ─────►│ sampling @ 80 kHz  │
                     └──────────┬──────────┘
                                │
                                ▼
                     ┌─────────────────────┐
                     │ Global Measurements │
                     │                     │
                     │ global_input_val    │
                     │ global_output_val   │
                     └───────┬─────┬───────┘
                             │     │
                  Input      │     │ Output
                  voltage    │     │ voltage
                             │     │
              ┌──────────────┘     └──────────────┐
              ▼                                   ▼
   ┌─────────────────────┐             ┌─────────────────────┐
   │   Routing Controller│             │ Output Regulator    │
   │                     │             │                     │
   │ Input > Target      │             │ Feed-forward duty   │
   │       → BUCK         │             │ + P correction      │
   │                     │             │ + I correction      │
   │ Input < Target      │             │ + anti-windup       │
   │       → BOOST       │             │                     │
   └──────────┬──────────┘             └──────────┬──────────┘
              │                                   │
              │ Buck/Boost state                  │ PWM duty
              ▼                                   ▼
       ┌──────────────┐                    ┌──────────────┐
       │ MCPWM        │                    │ LEDC         │
       │ 100 kHz      │                    │ 100 kHz      │
       │ + dead-time  │                    │ 9-bit PWM    │
       └──────┬───────┘                    └──────┬───────┘
              │                                   │
              └────────────────┬──────────────────┘
                               ▼
                     ┌───────────────────┐
                     │ Power Converter   │
                     │                   │
                     │ BUCK / BOOST      │
                     └───────────────────┘
```

---

# Control Strategy

The controller operates around a desired output voltage:

```cpp
out_desired
```

The controller continuously measures:

```cpp
global_input_val
global_output_val
```

and performs two separate control tasks.

### 1. Topology selection

The routing controller determines whether the converter should operate as a buck or boost converter.

```text
Vin > Vout_desired  → BUCK
Vin < Vout_desired  → BOOST
```

A switching deadband / anti-hysteresis delay prevents rapid transitions between the two modes when the input voltage is close to the target.

### 2. Output regulation

Once the topology is selected, the output regulator determines the required PWM duty cycle.

The regulator uses:

* Feed-forward duty calculation
* Proportional correction
* Integral correction
* Integral clamping
* Output-change thresholding

This allows the controller to start from an approximate theoretically correct duty cycle and then correct the remaining regulation error.

---

# File Structure

```text
.
├── main.cpp
├── adc_setup.h
├── routing.h
└── converter_output_regulator.h
```

## `adc_setup.h`

Responsible for configuring the ESP32 ADC and continuously acquiring the input and output voltage measurements.

### `set_adc_pins()`

```cpp
void set_adc_pins(
    uint8_t input_probe_channel_ADC1,
    uint8_t output_probe_channel_ADC1
);
```

Initializes the ESP32 continuous ADC driver.

Current configuration:

| Parameter          |           Value |
| ------------------ | --------------: |
| ADC unit           |            ADC1 |
| Resolution         |          12-bit |
| Attenuation        |           11 dB |
| Sampling frequency |          80 kHz |
| Conversion mode    | Single ADC unit |
| Buffer size        |         8 bytes |
| Frame size         |         4 bytes |

Two ADC channels are configured:

* Input-voltage probe
* Output-voltage probe

The ADC is started continuously during initialization.

---

### `read_adc()`

```cpp
void read_adc(
    adc_continuous_handle_t handle,
    uint8_t* buf,
    uint8_t buf_size,
    uint32_t* copied_out_len,
    uint32_t timeout_ms,
    float* global_input_val,
    float* global_output_val,
    uint8_t input_probe_channel_ADC1,
    uint8_t output_probe_channel_ADC1
);
```

Reads the most recent ADC conversion data and updates:

```cpp
global_input_val
global_output_val
```

The ADC's raw 12-bit value is converted into a voltage using:

```cpp
val = 0.00059 * val_raw;
```

Therefore, the current software assumes a fixed conversion factor between the ADC reading and the actual monitored voltage.

The physical voltage-divider / sensing circuitry must therefore be designed such that this scaling factor corresponds to the desired measurement range.

---

# `routing.h`

This module determines whether the converter operates in buck or boost mode.

## Hardware PWM initialization

```cpp
set_routing_pins(
    uint8_t buck_switch_pin,
    uint8_t boost_switch_pin
);
```

The routing controller uses ESP32 MCPWM.

Current configuration:

```text
MCPWM Unit 0
Timer 0
Frequency: 100 kHz
Counter: UP
Output A: Buck
Output B: Boost
```

Dead-time is enabled for the complementary outputs.

```cpp
mcpwm_deadtime_enable(...)
```

This is important because the buck and boost switching devices must not be driven simultaneously.

---

# Routing Algorithm

The central function is:

```cpp
converterState route(
    float out_desired,
    long int anti_hysteresis_delay,
    float* global_input_val
);
```

It returns:

```cpp
struct converterState {
    uint8_t buck_state;
    uint8_t boost_state;
};
```

The controller first measures the elapsed time since the last topology transition.

Instead of using `micros()`, the implementation uses:

```cpp
esp_cpu_get_cycle_count();
```

This provides a very low-overhead timing mechanism based directly on the ESP32 CPU cycle counter.

The elapsed cycle count is converted into a time estimate using:

```cpp
time_delay = cycle_delay * 0.004166;
```

The routing state is then updated only when the anti-hysteresis delay has expired.

Conceptually:

```text
                     ┌─────────────┐
                     │ Read Vin    │
                     └──────┬──────┘
                            │
                            ▼
                    Vin > Vtarget?
                     /          \
                   YES           NO
                    │             │
                    ▼             ▼
                  BUCK           BOOST
```

The boost state is generated as the complement of the buck state:

```cpp
boost_switch = (~buck_switch) & 0b00000001;
```

Thus the intended operating states are:

```text
Buck   = 1, Boost = 0
Buck   = 0, Boost = 1
```

---

# Anti-Hysteresis Switching

Rapid buck/boost transitions are undesirable when:

```text
Vin ≈ Vtarget
```

because measurement noise, converter ripple, and control corrections could repeatedly cause:

```text
BUCK → BOOST → BUCK → BOOST → ...
```

The routing controller therefore retains the previous topology until the configured delay has elapsed.

The delay is configured in `main.cpp`:

```cpp
#define ANTI_HYSTERISIS_DELAY 50
```

This parameter should be interpreted according to the time conversion used by the current cycle-counter implementation.

---

# `converter_output_regulator.h`

This module performs the actual output-voltage regulation.

The main function is:

```cpp
controlSig converter_out_regulator(
    float out_desired,
    int buck_state,
    float* global_input_val,
    float* global_output_val,
    float anti_windup_clamp
);
```

It returns:

```cpp
struct controlSig {
    float control_sig;
    float drive_sig;
};
```

where:

* `control_sig` is the normalized control signal
* `drive_sig` is the corresponding PWM value

---

# PWM Generation

The output regulator uses the ESP32 LEDC peripheral.

Current configuration:

```text
Frequency:       100 kHz
Resolution:      9 bit
Maximum duty:    511
Output:          CONVERTER_CONTROL_PIN
```

The normalized control signal is:

```text
0.0 ───────────────► 1.0
```

which is converted into:

```cpp
drive_signal = control_signal * 511;
```

The resulting value is written to the LEDC peripheral.

---

# Feed-Forward Control

The regulator does not begin from an arbitrary PWM duty cycle.

Instead, it calculates an approximate ideal duty cycle from the measured input voltage and desired output voltage.

## Buck mode

For an ideal buck converter:

$$
V_{out} = D V_{in}
$$

therefore:

$$
D \approx \frac{V_{out}}{V_{in}}
$$

The implementation uses:

```cpp
duty_cycle = out_desired / input_voltage;
```

with bounds applied between 0 and 1.

---

## Boost mode

For an ideal boost converter:

$$
V_{out} = \frac{V_{in}}{1-D}
$$

Rearranging:

$$
D \approx 1-\frac{V_{in}}{V_{out}}
$$

The implementation therefore uses:

```cpp
duty_cycle = 1 - (input_voltage / out_desired);
```

again constrained to:

```text
0 ≤ D ≤ 1
```

This feed-forward component provides the controller with an initial estimate of the duty cycle required to produce the desired output.

---

# Feedback Correction

The feed-forward estimate is corrected using the output-voltage error:

```cpp
differential = out_desired - previous_output;
```

Therefore:

```text
Output too low  → positive correction
Output too high → negative correction
```

The controller contains proportional and integral terms.

Current gains:

```cpp
integral_feedback = 0.0001;
proportional_feedback = 0.001;
```

The correction is:

$$
C = K_i I + K_p e
$$

where:

* `e` is output-voltage error
* `I` is the accumulated integral error
* `Kp` is the proportional gain
* `Ki` is the integral gain

The final command is approximately:

$$
D_{command}=D_{feedforward}+C
$$

and is constrained to:

```text
0 ≤ Dcommand ≤ 1
```

---

# Integral Control

The integral term accumulates output-voltage error:

```cpp
integral_term = integral_term + differential;
```

However, integration is deliberately restricted.

Integration occurs only when:

```text
buck state has not changed
AND
absolute output error < 2 V
```

This prevents the integrator from aggressively accumulating error during topology transitions or large transient errors.

---

# Anti-Windup

The integral term is clamped:

```cpp
integral_term = min(
    max(integral_term, -anti_windup_clamp),
    anti_windup_clamp
);
```

The current configuration is:

```cpp
#define ANTI_WINDUP_CLAMP 2
```

Therefore:

```text
-2 ≤ integral_term ≤ +2
```

This prevents the integral controller from accumulating indefinitely when the requested output cannot immediately be achieved.

---

# Small-Change Suppression

The regulator does not update the PWM peripheral for extremely small changes in the calculated control signal.

The current threshold is:

```cpp
0.01
```

Therefore, if:

```text
|new_control_signal - previous_control_signal| < 0.01
```

the PWM register is left unchanged.

This reduces unnecessary peripheral writes for insignificant corrections.

---

# Main Control Loop

`main.cpp` ties the complete system together.

The control sequence is:

```text
             ┌────────────────┐
             │ Continuous ADC │
             └───────┬────────┘
                     │
                     ▼
             ┌────────────────┐
             │ Read Vin/Vout  │
             └───────┬────────┘
                     │
                     ▼
             ┌────────────────┐
             │ Select topology│
             │ Buck / Boost    │
             └───────┬────────┘
                     │
                     ▼
             ┌────────────────┐
             │ Calculate duty │
             │ Feed-forward   │
             │ + PI correction│
             └───────┬────────┘
                     │
                     ▼
             ┌────────────────┐
             │ Update PWM      │
             └───────┬────────┘
                     │
                     ▼
             ┌────────────────┐
             │ Power converter │
             └───────┬────────┘
                     │
                     └──────► ADC feedback
```

The loop executes:

```cpp
read_adc(...);

routing_state = route(
    out_desired,
    ANTI_HYSTERISIS_DELAY,
    &global_input_val
);

out_sig = converter_out_regulator(
    out_desired,
    routing_state.buck_state,
    &global_input_val,
    &global_output_val,
    ANTI_WINDUP_CLAMP
);
```

This creates a closed-loop system:

```text
        ┌──────────────────────────────────────┐
        │                                      │
        │             Controller               │
        │                                      │
        │  Vin ──► Buck/Boost selection        │
        │  Vout ─► PI + feed-forward regulator │
        │                                      │
        └───────────────┬──────────────────────┘
                        │
                        ▼
                 PWM / topology
                        │
                        ▼
                 ┌─────────────┐
                 │   POWER     │
                 │  CONVERTER  │
                 └──────┬──────┘
                        │
                        ▼
                       Vout
                        │
                        └──────── feedback
```

---

# Telemetry

Drive Signal Output

The controller exposes the calculated drive signal through the ESP32's built-in DAC for simple real-time diagnostics without adding communication overhead to the control loop.

The normalized control_sig value (0–1) is mapped to the DAC's 8-bit output range (0–255):

dac_oneshot_output_voltage(dac_handle, out_sig.control_sig * 255);

This produces an approximately 0–3.3 V analog output proportional to the commanded converter duty cycle. For example, a control_sig of 0.5 produces approximately 1.65 V at the DAC output.

On the classic ESP32, DAC_CHAN_0 outputs on GPIO25. This output is intended as a diagnostic signal and does not participate in the converter's control path.

---

# Pin Configuration

The current `main.cpp` configuration is:

| Function              |    GPIO |
| --------------------- | ------: |
| Buck switch           | GPIO 14 |
| Boost switch          | GPIO 27 |
| Converter PWM control | GPIO 12 |

ADC channels:

| Measurement    |    ADC channel |
| -------------- | -------------: |
| Input voltage  | ADC1 channel 0 |
| Output voltage | ADC1 channel 1 |

These values can be changed using the macros at the beginning of `main.cpp`.

```cpp
#define INPUT_PROBE_CHANNEL_ADC1 0
#define OUTPUT_PROBE_CHANNEL_ADC1 1

#define BUCK_ENABLE_PIN 14
#define BOOST_ENABLE_PIN 27

#define CONVERTER_CONTROL_PIN 12
```

---

# Configuration Parameters

The main control parameters are:

```cpp
#define ADC_TIMEOUT_MS 5
#define ANTI_HYSTERISIS_DELAY 50
#define ANTI_WINDUP_CLAMP 2
```

The controller's internal gains are currently:

```cpp
integral_feedback = 0.0001;
proportional_feedback = 0.001;
```

These parameters determine the dynamic behavior of the regulation loop and should be tuned according to the actual converter power stage, switching frequency, load range, sensing circuitry, and desired transient response.

---

# Operating Example

Assume:

```text
Vin = 18 V
Vtarget = 12 V
```

Since:

```text
18 V > 12 V
```

the routing controller selects:

```text
BUCK
```

The feed-forward duty estimate is:

$$
D \approx \frac{12}{18}=0.667
$$

so the controller starts near:

```text
66.7% duty
```

The measured output is then compared against the target.

For example, if:

```text
Vout = 11.7 V
Vtarget = 12 V
```

the error is:

```text
+0.3 V
```

The proportional and integral terms provide a positive correction to the feed-forward duty.

---

Now assume:

```text
Vin = 8 V
Vtarget = 12 V
```

Since:

```text
8 V < 12 V
```

the routing controller selects:

```text
BOOST
```

The ideal boost duty estimate is:

$$
D \approx 1-\frac{8}{12}
$$

$$
D \approx 0.333
$$

so the regulator starts around:

```text
33.3% duty
```

and feedback subsequently corrects the remaining error.

---

# Timing Architecture

A notable part of this implementation is its use of the ESP32 CPU cycle counter:

```cpp
esp_cpu_get_cycle_count()
```

instead of a higher-level timing function such as:

```cpp
micros()
```

The cycle counter provides direct access to the processor's cycle count and is therefore suitable for very low-overhead timing measurements inside the control path.

The routing controller computes:

```cpp
now = esp_cpu_get_cycle_count();

cycle_delay = now - last_switching_time;
```

and subsequently converts the cycle difference into a time estimate.

This is particularly useful because the buck/boost routing decision is part of a latency-sensitive control path.

---

# Peripheral Allocation

The system currently makes use of three ESP32 hardware peripherals:

| Peripheral            | Purpose                            |
| --------------------- | ---------------------------------- |
| ADC continuous driver | Input/output voltage measurement   |
| MCPWM                 | Buck/boost switching and dead-time |
| LEDC                  | Converter PWM duty control         |

This separates the topology-selection switching from the converter duty-cycle generation.

Conceptually:

```text
MCPWM
  │
  ├── Buck switching
  └── Boost switching

LEDC
  │
  └── Converter duty-cycle control

ADC
  │
  ├── Vin measurement
  └── Vout measurement
```

---

# Current Assumptions

The software currently assumes:

1. `out_desired` contains a valid target output voltage.
2. The ADC inputs are already scaled to safe ESP32 ADC voltages through appropriate sensing circuitry.
3. The ADC scaling factor `0.00059` corresponds to the actual sensing network.
4. The power stage itself provides the required inductors, capacitors, MOSFETs, gate drivers, diodes/synchronous switches, current limiting, and protection.
5. Buck and boost switching are electrically complementary.
6. The selected topology is safe for the instantaneous input/output voltage relationship.
7. The converter operates at approximately 100 kHz.
8. The ESP32 CPU frequency remains consistent with the cycle-to-time conversion used by the routing controller.
9. The control loop executes frequently enough relative to the converter dynamics.
10. `out_desired` is provided by the higher-level application/control layer.

---

# Safety Considerations

This software is a controller, not a complete power-stage protection system.

A real hardware implementation should independently address conditions such as:

* Over-voltage
* Under-voltage
* Over-current
* Short circuit
* MOSFET shoot-through
* Excessive inductor current
* Thermal overload
* ADC/sensor failure
* Loss of feedback
* Startup conditions
* Input transients
* Output capacitor discharge
* Buck/boost transition transients

Software PWM control should not be considered the sole protection mechanism for a high-power converter.

Hardware-level protections should be used wherever required by the power-stage design.

---

# Summary

This project implements a digitally controlled buck/boost converter around an ESP32.

The controller can be summarized as:

```text
                DESIRED OUTPUT
                      │
                      ▼
              ┌───────────────┐
              │ Routing Logic │
              └───────┬───────┘
                      │
              ┌───────┴───────┐
              │               │
           Vin>Vtarget    Vin<Vtarget
              │               │
              ▼               ▼
             BUCK            BOOST
              │               │
              └───────┬───────┘
                      │
                      ▼
             Feed-Forward Duty
                      │
                      ▼
                PI Correction
                      │
                      ▼
                 PWM Output
                      │
                      ▼
              POWER CONVERTER
                      │
                      ▼
                    Vout
                      │
                      └──────────► ADC feedback
```

The resulting architecture combines **feed-forward control for fast initial response** with **closed-loop correction for regulation accuracy**, while the separate routing controller automatically selects the appropriate converter topology based on the relationship between input voltage and the requested output voltage.
