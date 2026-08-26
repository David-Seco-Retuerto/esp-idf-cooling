#include <stdint.h>

#include "cooling.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FAN_PWM_GPIO 21

static const char *TAG = "cooling_example";


static cooled_device_t inverter = {
    .name = "Inverter",
    .temperature_mc = 40000,
    .threshold_temp_mc = 50000,
    .full_duty_temp_mc = 70000,
};


static cooled_device_t motor = {
    .name = "Motor",
    .temperature_mc = 45000,
    .threshold_temp_mc = 55000,
    .full_duty_temp_mc = 75000,
};


void app_main(void)
{
    ESP_ERROR_CHECK(cooling_pwm_global_init());

    cooling_system_t cooling_system;

    ESP_ERROR_CHECK(
        cooling_system_init(&cooling_system, 2)
    );

    ESP_ERROR_CHECK(
        cooling_system_add_device(&cooling_system, &inverter)
    );

    ESP_ERROR_CHECK(
        cooling_system_add_device(&cooling_system, &motor)
    );


    cooling_actuator_t fan;

    ESP_ERROR_CHECK(
        cooling_actuator_init(
            &fan,
            "Main fan",
            FAN_PWM_GPIO,
            3000,   // 3 °C hysteresis
            10,     // +10 percentage points / °C
            25,     // minimum 25% duty while active
            false   // active-high PWM
        )
    );

    ESP_ERROR_CHECK(cooling_actuator_set_system(&fan, &cooling_system));


    int32_t temperature_step_mc = 2000;

    while (1) {
        inverter.temperature_mc += temperature_step_mc;

        if (inverter.temperature_mc >= 76000) {
            temperature_step_mc = -2000;
        } else if (inverter.temperature_mc <= 40000) {
            temperature_step_mc = 2000;
        }

        ESP_ERROR_CHECK(cooling_actuator_update(&fan));

        ESP_LOGI(
            TAG,
            "Inverter: %.1f °C | Fan duty: %u%% | Active: %s",
            inverter.temperature_mc / 1000.0,
            (unsigned)fan.duty_cycle,
            fan.active ? "yes" : "no"
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}