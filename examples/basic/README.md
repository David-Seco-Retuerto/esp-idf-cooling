# Basic Cooling Example

This example demonstrates the basic use of the ESP-IDF cooling component.

It configures:

- Two temperature-monitored devices.
- One PWM-controlled cooling actuator.
- A 3 °C hysteresis band.
- A minimum active duty cycle of 25%.
- A linear duty increase of 10 percentage points per degree Celsius.

The inverter temperature is simulated so that actuator activation,
proportional duty control, full-duty operation, and hysteresis can be observed.

## Configuration

Update the PWM GPIO in `main/main.c` for your board:

```c
#define FAN_PWM_GPIO 21
```

## Build

From the `examples/basic` directory:

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```
