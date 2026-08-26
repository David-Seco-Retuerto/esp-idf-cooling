#include "cooling.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "soc/soc_caps.h"


#define COOLING_TAG "COOLING_DRV"

#define COOLING_PWM_INVALID_CH      UINT8_MAX
#define COOLING_LEDC_RESOLUTION     LEDC_TIMER_8_BIT
#define COOLING_LEDC_FULL_DUTY      (1U << COOLING_LEDC_RESOLUTION)


#if SOC_LEDC_SUPPORT_HS_MODE
#define COOLING_LEDC_MODE LEDC_HIGH_SPEED_MODE
#else
#define COOLING_LEDC_MODE LEDC_LOW_SPEED_MODE
#endif

static bool s_pwm_channel_used[SOC_LEDC_CHANNEL_NUM] = {0};
static bool s_pwm_global_initialized = false;



/* PWM helpers */

esp_err_t cooling_pwm_global_init(void)
{
    if (s_pwm_global_initialized) {
        return ESP_OK;
    }

    ledc_timer_config_t timer_config = {
        .speed_mode = COOLING_LEDC_MODE,
        .timer_num = COOLING_LEDC_TIMER,
        .freq_hz = COOLING_LEDC_FREQ_HZ,
        .duty_resolution = COOLING_LEDC_RESOLUTION,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        ESP_LOGE(COOLING_TAG, "Failed to initialize global PWM timer");
        return err;
    }

    s_pwm_global_initialized = true;
    ESP_LOGD(COOLING_TAG, "Global PWM initialized successfully");
    return ESP_OK;
}

/**
 * @brief Configure the LEDC channel used by a cooling actuator.
 *
 * Configures the actuator GPIO, internally allocated LEDC channel, timer,
 * initial 0% duty cycle, and optional output inversion.
 *
 * The global LEDC timer must have been initialized successfully before calling
 * this function.
 *
 * @param[in] act Pointer to the actuator whose PWM channel is to be configured.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the global PWM timer is not initialized
 *      - ESP-IDF LEDC error code if channel configuration fails
 */
static esp_err_t cooling_actuator_pwm_init(const cooling_actuator_t *act)
{
    if (!s_pwm_global_initialized) {
        ESP_LOGE(COOLING_TAG, "PWM global init not done");
        return ESP_ERR_INVALID_STATE;
    }

    ledc_channel_config_t channel_config = {
        .gpio_num   = act->gpio,
        .speed_mode = COOLING_LEDC_MODE,
        .channel    = (ledc_channel_t)act->pwm_channel,
        .timer_sel  = COOLING_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .flags = {
            .output_invert = act->pwm_inverted
            }
    };

    return ledc_channel_config(&channel_config);
}

/**
 * @brief Set the logical PWM duty cycle of a cooling actuator.
 *
 * Converts a logical duty value in the range 0-100% to the raw LEDC duty value
 * and applies it to the actuator channel.
 *
 * PWM polarity inversion is handled by the LEDC channel configuration, so the
 * supplied duty always represents logical cooling demand:
 *
 * - 0% means cooling off.
 * - 100% means maximum cooling.
 *
 * @param[in] act Pointer to the cooling actuator.
 * @param[in] duty_percent Logical duty cycle from 0 to 100%.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if @p act is NULL or @p duty_percent exceeds 100
 *      - ESP_ERR_INVALID_STATE if PWM is not initialized or the actuator channel
 *        is invalid
 *      - ESP-IDF LEDC error code if the duty cannot be updated
 */
static esp_err_t cooling_actuator_pwm_set_duty(const cooling_actuator_t *act, uint8_t duty_percent)
{
    if (act == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_pwm_global_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (act->pwm_channel == COOLING_PWM_INVALID_CH ||
        act->pwm_channel >= SOC_LEDC_CHANNEL_NUM) {
        return ESP_ERR_INVALID_STATE;
    }

    if (duty_percent > 100U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t ledc_duty = ((uint32_t)duty_percent * COOLING_LEDC_FULL_DUTY) / 100U;

    return ledc_set_duty_and_update(
        COOLING_LEDC_MODE,
        (ledc_channel_t)act->pwm_channel,
        ledc_duty,
        0
    );
}

/**
 * @brief Set a cooling actuator output to 0% logical duty.
 *
 * @param[in] act Pointer to the cooling actuator.
 *
 * @return
 *      - ESP_OK on success
 *      - Error code returned by cooling_actuator_pwm_set_duty() on failure
 */
static esp_err_t cooling_actuator_pwm_stop(const cooling_actuator_t *act)
{
    return cooling_actuator_pwm_set_duty(act, 0U);
}



/* Validation helpers */

/**
 * @brief Validate a cooled-device configuration.
 *
 * Checks that the device pointer is valid, that the device name is non-empty
 * and null-terminated, and that the full-duty temperature is greater than the
 * cooling activation threshold.
 *
 * @param[in] dev Pointer to the cooled device to validate.
 *
 * @return
 *      - ESP_OK if the device configuration is valid
 *      - ESP_ERR_INVALID_ARG if the pointer, name, or temperature configuration
 *        is invalid
 */
static esp_err_t cooled_device_validate(const cooled_device_t *dev)
{


    if (dev == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid cooled device pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->name[0] == '\0' ||
        memchr(dev->name, '\0', sizeof(dev->name)) == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid cooled device name");
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->full_duty_temp_mc <= dev->threshold_temp_mc) {
        ESP_LOGE(
            COOLING_TAG,
            "Device '%s' has invalid temperature limits: full-duty temperature (%ld) must be greater than threshold (%ld)",
            dev->name,
            (long)dev->full_duty_temp_mc,
            (long)dev->threshold_temp_mc
        );
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Validate the internal state of a cooling system.
 *
 * Checks that the system exists, has initialized device storage, and contains
 * consistent device-count and capacity values.
 *
 * @param[in] sys Pointer to the cooling system to validate.
 *
 * @return
 *      - ESP_OK if the cooling system is valid
 *      - ESP_ERR_INVALID_ARG if @p sys is NULL
 *      - ESP_ERR_INVALID_STATE if the cooling system is not initialized or its
 *        internal state is inconsistent
 */
static esp_err_t cooling_system_validate(const cooling_system_t *sys)
{
    if (sys == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid cooling system pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (sys->devices == NULL) {
        ESP_LOGE(COOLING_TAG, "Cooling system not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sys->max_devices == 0U || sys->num_devices > sys->max_devices) {
        ESP_LOGE(COOLING_TAG, "Cooling system has invalid internal state");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

/**
 * @brief Validate the state of a cooling actuator.
 *
 * @param[in] actuator Pointer to the actuator to validate.
 *
 * @return
 *      - ESP_OK if the actuator is initialized
 *      - ESP_ERR_INVALID_ARG if @p actuator is NULL
 *      - ESP_ERR_INVALID_STATE if the actuator is not initialized
 */
static esp_err_t cooling_actuator_validate(const cooling_actuator_t *actuator)
{
    if (actuator == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid actuator pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (!actuator->initialized) {
        ESP_LOGE(COOLING_TAG, "Actuator not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}



/* PWM channel management */

/**
 * @brief Allocate an unused LEDC channel for a cooling actuator.
 *
 * Channels are tracked internally by the cooling driver. This allocator only
 * tracks channels allocated through this component.
 *
 * @return
 *      - Allocated LEDC channel index on success
 *      - COOLING_PWM_INVALID_CH if no driver-managed channel is available
 */
static uint8_t cooling_allocate_pwm_channel(void)
{
    for (uint8_t i = 0; i < SOC_LEDC_CHANNEL_NUM; i++) {
        if (!s_pwm_channel_used[i]) {
            s_pwm_channel_used[i] = true;
            return i;
        }
    }

    return COOLING_PWM_INVALID_CH;
}

/**
 * @brief Release an internally allocated LEDC channel.
 *
 * Marks the channel as available for reuse by another cooling actuator.
 * Invalid channel indices are ignored.
 *
 * @param[in] channel LEDC channel index to release.
 */
static void cooling_release_pwm_channel(uint8_t channel)
{
    if (channel < SOC_LEDC_CHANNEL_NUM) {
        s_pwm_channel_used[channel] = false;
    }
}



/* Cooling system management */

void cooling_system_destroy(cooling_system_t *system)
{
    if (system == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid cooling system pointer");
        return;
    }

    free(system->devices);

    *system = (cooling_system_t){0};

    ESP_LOGD(COOLING_TAG, "Cooling system destroyed");
}

esp_err_t cooling_system_init(cooling_system_t *system, size_t max_devices)
{
    if (system == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid cooling system pointer");
        return ESP_ERR_INVALID_ARG;
    }

    *system = (cooling_system_t){0};

    if (max_devices == 0U) {
        ESP_LOGE(COOLING_TAG, "max_devices cannot be 0");
        return ESP_ERR_INVALID_ARG;
    }

    if (max_devices > SIZE_MAX / sizeof(*system->devices)) {
        ESP_LOGE(COOLING_TAG, "Requested device capacity is too large");
        return ESP_ERR_INVALID_ARG;
    }

    system->devices = calloc(max_devices, sizeof(*system->devices));
    if (system->devices == NULL) {
        ESP_LOGE(COOLING_TAG, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }

    system->num_devices = 0;
    system->max_devices = max_devices;

    ESP_LOGD(
        COOLING_TAG,
        "Cooling system initialized with capacity for %u device(s)",
        (unsigned)max_devices
    );
    return ESP_OK;
}

esp_err_t cooling_system_add_device(cooling_system_t *system, const cooled_device_t *device)
{
    esp_err_t err = cooling_system_validate(system);
    if (err != ESP_OK) {
        return err;
    }

    err = cooled_device_validate(device);
    if (err != ESP_OK) {
        return err;
    }

    for (size_t i = 0; i < system->num_devices; i++) {
        if (system->devices[i] == device) {
            ESP_LOGE(
                COOLING_TAG,
                "Device '%s' is already registered in this cooling system",
                device->name
            );
            return ESP_ERR_INVALID_STATE;
        }
    }

    if (system->num_devices >= system->max_devices) {
        ESP_LOGE(COOLING_TAG, "Cooling system has reached its device capacity (%u)", (unsigned)system->max_devices);
        return ESP_ERR_NO_MEM;
    }

    system->devices[system->num_devices] = device;
    system->num_devices++;

    ESP_LOGD(
        COOLING_TAG,
        "Device '%s' added to cooling system",
        device->name
    );
    return ESP_OK;
}



/* Cooling actuator management */

void cooling_actuator_destroy(cooling_actuator_t *actuator)
{
    if (actuator == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid actuator pointer");
        return;
    }

    if (!actuator->initialized) {
        *actuator = (cooling_actuator_t){
            .gpio = -1,
            .pwm_channel = COOLING_PWM_INVALID_CH
        };
        return;
    }

    if (actuator->pwm_channel != COOLING_PWM_INVALID_CH) {
        if (s_pwm_global_initialized) {
            esp_err_t err = cooling_actuator_pwm_stop(actuator);

            if (err != ESP_OK) {
                ESP_LOGW(
                    COOLING_TAG,
                    "Failed to stop actuator '%s' PWM: %s",
                    actuator->name,
                    esp_err_to_name(err)
                );
            }
        }
        cooling_release_pwm_channel(actuator->pwm_channel);
    }

    *actuator = (cooling_actuator_t){
        .gpio = -1,
        .pwm_channel = COOLING_PWM_INVALID_CH
    };
    ESP_LOGD(COOLING_TAG, "Actuator was destroyed");
}

esp_err_t cooling_actuator_init(
    cooling_actuator_t *actuator,
    const char *name,
    int gpio,
    uint32_t hysteresis_mc,
    uint32_t slope_pct_per_degC,
    uint8_t min_duty_cycle,
    bool pwm_inverted)
{
    if (actuator == NULL) {
        ESP_LOGE(COOLING_TAG, "Invalid actuator pointer");
        return ESP_ERR_INVALID_ARG;
    }

    /* Leave the actuator in a safe state if initialization fails. */
    *actuator = (cooling_actuator_t){
        .gpio = -1,
        .pwm_channel = COOLING_PWM_INVALID_CH
    };

    if (name == NULL || name[0] == '\0') {
        ESP_LOGE(COOLING_TAG, "Invalid actuator name");
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(name) >= sizeof(actuator->name)) {
        ESP_LOGE(COOLING_TAG, "Actuator name is too long");
        return ESP_ERR_INVALID_ARG;
    }

    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        ESP_LOGE(COOLING_TAG, "GPIO %d is not a valid output GPIO", gpio);
        return ESP_ERR_INVALID_ARG;
    }

    if (min_duty_cycle > 100U) {
        ESP_LOGE(COOLING_TAG, "Minimum duty cycle must be between 0 and 100");
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_pwm_global_initialized) {
        ESP_LOGE(COOLING_TAG, "PWM global initialization not done");
        return ESP_ERR_INVALID_STATE;
    }

    actuator->gpio = gpio;
    actuator->hysteresis_mc = hysteresis_mc;
    actuator->slope_pct_per_degC = slope_pct_per_degC;
    actuator->min_duty_cycle = min_duty_cycle;
    actuator->pwm_inverted = pwm_inverted;

    strncpy(actuator->name, name, sizeof(actuator->name) - 1);
    actuator->name[sizeof(actuator->name) - 1] = '\0';

    actuator->pwm_channel = cooling_allocate_pwm_channel();
    if (actuator->pwm_channel == COOLING_PWM_INVALID_CH) {
        ESP_LOGE(COOLING_TAG, "No PWM channels available");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = cooling_actuator_pwm_init(actuator);
    if (err != ESP_OK) {
        cooling_release_pwm_channel(actuator->pwm_channel);
        actuator->pwm_channel = COOLING_PWM_INVALID_CH;

        ESP_LOGE(
            COOLING_TAG,
            "Failed to initialize PWM for actuator: %s",
            esp_err_to_name(err)
        );

        return err;
    }

    actuator->initialized = true;
    actuator->active = false;
    actuator->duty_cycle = 0;
    actuator->cooling_system = NULL;

    ESP_LOGD(
        COOLING_TAG,
        "Actuator '%s' initialized on GPIO %d, LEDC channel %u",
        actuator->name,
        actuator->gpio,
        actuator->pwm_channel
    );
    return ESP_OK;
}

esp_err_t cooling_actuator_set_system(cooling_actuator_t *actuator, cooling_system_t *system)
{
    esp_err_t err = cooling_actuator_validate(actuator);
    if (err != ESP_OK) {
        return err;
    }

    err = cooling_system_validate(system);
    if (err != ESP_OK) {
        return err;
    }

    err = cooling_actuator_pwm_stop(actuator);
    if (err != ESP_OK) {
        ESP_LOGE(
            COOLING_TAG,
            "Failed to stop actuator PWM before assigning system: %s",
            esp_err_to_name(err)
        );
        return err;
    }

    actuator->cooling_system = system;
    actuator->active = false;
    actuator->duty_cycle = 0;

    ESP_LOGD(
        COOLING_TAG,
        "Cooling system assigned to actuator '%s'",
        actuator->name
    );
    return ESP_OK;
}

esp_err_t cooling_actuator_detach_system(cooling_actuator_t *actuator)
{
    esp_err_t err = cooling_actuator_validate(actuator);
    if (err != ESP_OK) {
        return err;
    }

    err = cooling_actuator_pwm_stop(actuator);
    if (err != ESP_OK) {
        ESP_LOGE(
            COOLING_TAG,
            "Failed to stop actuator '%s' PWM: %s",
            actuator->name,
            esp_err_to_name(err)
        );
        return err;
    }

    actuator->cooling_system = NULL;
    actuator->active = false;
    actuator->duty_cycle = 0;

    ESP_LOGD(COOLING_TAG, "Cooling system detached from actuator '%s'", actuator->name);

    return ESP_OK;
}



/* Cooling control */

/**
 * @brief Update the actuator active state according to device temperatures.
 *
 * An inactive actuator becomes active when at least one registered device rises
 * above its threshold temperature.
 *
 * Once active, the actuator remains active until every registered device falls
 * below its threshold minus the configured actuator hysteresis.
 *
 * @param[in,out] actuator Pointer to an actuator with a valid assigned cooling
 *        system.
 */
static void cooling_actuator_apply_hysteresis(cooling_actuator_t *actuator)
{
    bool any_above_threshold = false;
    bool all_below_hysteresis = true;

    for (size_t i = 0; i < actuator->cooling_system->num_devices; i++) {

        const cooled_device_t *dev = actuator->cooling_system->devices[i];
        int64_t off_threshold_mc = (int64_t) dev->threshold_temp_mc - (int64_t)actuator->hysteresis_mc;

        if (dev->temperature_mc > dev->threshold_temp_mc) {
            any_above_threshold = true;
        }

        if (dev->temperature_mc >= off_threshold_mc) {
            all_below_hysteresis = false;
        }
    }

    if (!actuator->active && any_above_threshold) {
        actuator->active = true;
    }
    else if (actuator->active && all_below_hysteresis) {
        actuator->active = false;
    }
}

/**
 * @brief Compute the cooling duty requested by one cooled device.
 *
 * The requested duty is 0% at or below the device threshold, increases linearly
 * according to @p slope_pct_per_degC above the threshold, and becomes 100% at
 * or above the device full-duty temperature.
 *
 * The calculated duty is saturated at 100%.
 *
 * @param[in] device Pointer to the cooled device.
 * @param[in] slope_pct_per_degC Duty increase in percentage points per degree
 *        Celsius above the device threshold.
 *
 * @return Requested logical duty cycle from 0 to 100%.
 */
static uint8_t cooled_device_compute_duty(const cooled_device_t *device, uint32_t slope_pct_per_degC)
{
    if (device->temperature_mc <= device->threshold_temp_mc) {
        return 0;
    }

    if (device->temperature_mc >= device->full_duty_temp_mc) {
        return 100;
    }

    int64_t delta_mc = (int64_t)device->temperature_mc - (int64_t)device->threshold_temp_mc;

    uint64_t duty = ((uint64_t)delta_mc * slope_pct_per_degC) / 1000U;

    if (duty > 100U) {
        duty = 100U;
    }

    return (uint8_t)duty;
}

/**
 * @brief Compute the total cooling demand for an actuator.
 *
 * Computes the requested duty cycle of every device in the actuator's assigned
 * cooling system and returns the highest request.
 *
 * @param[in] actuator Pointer to an actuator with a valid assigned cooling
 *        system containing at least one device.
 *
 * @return Highest requested logical duty cycle from 0 to 100%.
 */
static uint8_t cooling_actuator_compute_duty(const cooling_actuator_t *actuator)
{
    uint8_t max_duty = 0;

    for (size_t i = 0; i < actuator->cooling_system->num_devices; i++) {
        uint8_t duty = cooled_device_compute_duty(
            actuator->cooling_system->devices[i],
            actuator->slope_pct_per_degC
        );

        if (duty > max_duty) {
            max_duty = duty;
        }
    }

    return max_duty;
}

esp_err_t cooling_actuator_update(cooling_actuator_t *actuator)
{
    esp_err_t err = cooling_actuator_validate(actuator);
    if (err != ESP_OK) {
        return err;
    }

    if (actuator->cooling_system == NULL) {
        ESP_LOGE(COOLING_TAG, "Actuator has no cooling system assigned");
        return ESP_ERR_INVALID_STATE;
    }

    err = cooling_system_validate(actuator->cooling_system);
    if (err != ESP_OK) {
        return err;
    }

    if (actuator->cooling_system->num_devices == 0U) {
        ESP_LOGE(COOLING_TAG, "Cooling system contains no devices");
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < actuator->cooling_system->num_devices; i++) {
        err = cooled_device_validate(actuator->cooling_system->devices[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    bool was_active = actuator->active;

    cooling_actuator_apply_hysteresis(actuator);

    if (actuator->active != was_active) {
        ESP_LOGD(
            COOLING_TAG,
            "Actuator '%s' %s",
            actuator->name,
            actuator->active ? "activated" : "deactivated"
        );
    }
    uint8_t new_duty = 0;

    if (actuator->active) {
        new_duty = cooling_actuator_compute_duty(actuator);

        if (new_duty < actuator->min_duty_cycle) {
            new_duty = actuator->min_duty_cycle;
        }
    }

    err = cooling_actuator_pwm_set_duty(actuator, new_duty);
    if (err != ESP_OK) {
        ESP_LOGE(
            COOLING_TAG,
            "Failed to update actuator '%s' PWM: %s",
            actuator->name,
            esp_err_to_name(err)
        );
        return err;
    }
    if (new_duty != actuator->duty_cycle) {
        ESP_LOGD(COOLING_TAG, "Updated actuator '%s' duty cycle from %u to %u", 
            actuator->name, 
            (unsigned)actuator->duty_cycle, 
            (unsigned)new_duty);
    }

    actuator->duty_cycle = new_duty;

    return ESP_OK;
}
