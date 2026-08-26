#pragma once


/* =========================================================================================================
 * Includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/ledc.h"

/* =========================================================================================================
 * CONFIGURATION
 */

#ifndef COOLING_LEDC_TIMER
    #define COOLING_LEDC_TIMER          LEDC_TIMER_0
#endif

#ifndef COOLING_LEDC_FREQ_HZ
#define COOLING_LEDC_FREQ_HZ 25000
#endif



/* =========================================================================================================
 * Data Types
 */

/**
 * @brief Temperature-monitored device cooled by a cooling actuator.
 *
 * A cooled device represents a component whose temperature is monitored by the
 * application, such as a motor, inverter, battery, or other heat-generating
 * device.
 *
 * The application is responsible for updating @ref temperature_mc before
 * calling cooling_actuator_update().
 *
 * Cooling begins when the device temperature rises above
 * @ref threshold_temp_mc. When the temperature reaches
 * @ref full_duty_temp_mc, the device requests 100% actuator duty.
 *
 * Temperatures are expressed in milli-degrees Celsius (m°C).
 */
typedef struct {
    char name[16];                  /**< Human-readable device name. */
    int32_t temperature_mc;         /**< Current device temperature in m°C. */
    int32_t threshold_temp_mc;      /**< Temperature above which cooling is requested, in m°C. */
    int32_t full_duty_temp_mc;      /**< Temperature at which 100% actuator duty is requested, in m°C. */
} cooled_device_t;


/**
 * @brief Group of cooled devices controlled by the same actuator.
 *
 * A cooling system stores pointers to one or more @ref cooled_device_t objects.
 * During each actuator update, the cooling demand of all registered devices is
 * evaluated and the highest requested duty cycle is applied to the actuator.
 *
 * The cooling system does not own the device objects. The application must
 * ensure that every registered @ref cooled_device_t remains valid for as long
 * as it is part of the system.
 *
 * The internal device-pointer array is allocated by cooling_system_init() and
 * released by cooling_system_destroy().
 */
typedef struct {
    const cooled_device_t **devices; /**< Registered cooled devices. */
    size_t num_devices;              /**< Number of currently registered devices. */
    size_t max_devices;              /**< Maximum number of devices that can be registered. */
} cooling_system_t;


/**
 * @brief PWM-controlled cooling actuator.
 *
 * A cooling actuator represents one PWM-controlled cooling output, such as a
 * fan or pump.
 *
 * The actuator is associated with a @ref cooling_system_t and determines its
 * required duty cycle from the cooling demand of all devices in that system.
 * The highest requested duty is applied.
 *
 * Hysteresis is used to prevent repeated activation and deactivation near a
 * device threshold. Once activated, the actuator remains active until all
 * registered devices fall below their respective threshold minus the configured
 * hysteresis.
 *
 * While active, the actuator output is never lower than
 * @ref min_duty_cycle. The logical duty cycle always uses the range 0-100%,
 * regardless of whether the physical PWM output is inverted.
 *
 * The PWM channel is allocated internally by the driver.
 */
typedef struct {
    char name[16];                      /**< Human-readable actuator name. */
    int gpio;                           /**< GPIO used for the PWM output. */
    uint8_t pwm_channel;                /**< Internally allocated LEDC channel. */
    bool initialized;                   /**< True after successful actuator initialization. */
    bool active;                        /**< Current hysteresis-controlled active state. */

    cooling_system_t *cooling_system;   /**< Cooling system currently assigned to the actuator. */

    uint32_t hysteresis_mc;             /**< Hysteresis band in m°C. */
    uint32_t slope_pct_per_degC;        /**< Duty increase in percentage points per °C above threshold. */

    uint8_t duty_cycle;                 /**< Current logical actuator duty cycle, from 0 to 100%. */
    uint8_t min_duty_cycle;             /**< Minimum duty cycle applied while the actuator is active. */
    bool pwm_inverted;                  /**< True to invert the physical PWM output polarity. */
} cooling_actuator_t;



/* =========================================================================================================
 * PWM Global Initialization
 */

/**
 * @brief Initialize the global LEDC timer used by the cooling driver.
 *
 * This function configures the LEDC timer shared by all cooling actuators.
 * It must be called successfully before cooling_actuator_init().
 *
 * The function is idempotent. Calling it again after successful initialization
 * returns ESP_OK without reconfiguring the timer.
 *
 * The LEDC timer and PWM frequency may be overridden at compile time using
 * COOLING_LEDC_TIMER and COOLING_LEDC_FREQ_HZ. The driver uses a fixed 8-bit
 * duty resolution.
 *
 * @return
 *      - ESP_OK on success, or if the PWM timer was already initialized
 *      - ESP-IDF LEDC error code if timer configuration fails
 */
esp_err_t cooling_pwm_global_init(void);

/* =========================================================================================================
    Cooling System Management
*/

/**
 * @brief Initialize a cooling system.
 *
 * Allocates storage for up to @p max_devices cooled-device pointers and
 * initializes the system state.
 *
 * The cooling system stores pointers to devices but does not allocate or own
 * the @ref cooled_device_t objects themselves.
 *
 * @param[out] system Pointer to the cooling system to initialize.
 * @param[in] max_devices Maximum number of devices that may be registered.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if @p system is NULL, @p max_devices is zero,
 *        or the requested capacity is invalid
 *      - ESP_ERR_NO_MEM if the internal device-pointer array cannot be allocated
 */
esp_err_t cooling_system_init(cooling_system_t *system, size_t max_devices);


/**
 * @brief Destroy a cooling system and release its internal storage.
 *
 * Frees the internal array used to store device pointers and resets the cooling
 * system structure.
 *
 * The registered @ref cooled_device_t objects are not freed.
 *
 * Any actuator still referencing this cooling system must be detached or
 * destroyed before the system itself is destroyed.
 *
 * @param[in,out] system Pointer to the cooling system to destroy.
 */
void cooling_system_destroy(cooling_system_t *system);


/**
 * @brief Add a cooled device to a cooling system.
 *
 * Registers a pointer to @p device in the cooling system.
 *
 * The cooling system does not take ownership of the device. The device must
 * therefore remain valid while registered.
 *
 * A device may only be registered once in the same cooling system.
 *
 * The device configuration must satisfy:
 *
 * @code
 * full_duty_temp_mc > threshold_temp_mc
 * @endcode
 *
 * @param[in,out] system Pointer to an initialized cooling system.
 * @param[in] device Pointer to the cooled device to register.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if @p system or @p device is NULL, or if the
 *        device temperature configuration is invalid
 *      - ESP_ERR_INVALID_STATE if the cooling system is not initialized,
 *        has an invalid state, or the device is already registered
 *      - ESP_ERR_NO_MEM if the cooling system has reached its configured
 *        device capacity
 */
esp_err_t cooling_system_add_device(cooling_system_t *system, const cooled_device_t *device);



/* =========================================================================================================
 * Cooling Actuator Management
 */

/**
 * @brief Initialize a PWM-controlled cooling actuator.
 *
 * Initializes an actuator, allocates an available LEDC channel, and configures
 * the selected GPIO as a PWM output.
 *
 * cooling_pwm_global_init() must be called successfully before this function.
 *
 * The actuator starts inactive with a logical duty cycle of 0%.
 *
 * @param[out] actuator Pointer to the actuator structure to initialize.
 * @param[in] name Human-readable actuator name. The name must fit in the
 *        actuator name buffer.
 * @param[in] gpio GPIO used for the PWM output.
 * @param[in] hysteresis_mc Hysteresis band in milli-degrees Celsius.
 * @param[in] slope_pct_per_degC Duty-cycle increase in percentage points per
 *        degree Celsius above a device threshold.
 * @param[in] min_duty_cycle Minimum duty cycle applied while the actuator is
 *        active, from 0 to 100%.
 * @param[in] pwm_inverted Set to true when the physical PWM output must be
 *        inverted.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if an argument is invalid
 *      - ESP_ERR_INVALID_STATE if the global PWM timer has not been initialized
 *      - ESP_ERR_NO_MEM if no LEDC channels are available
 *      - ESP-IDF LEDC error code if channel configuration fails
 */
esp_err_t cooling_actuator_init(
    cooling_actuator_t *actuator,
    const char *name,
    int gpio,
    uint32_t hysteresis_mc,
    uint32_t slope_pct_per_degC,
    uint8_t min_duty_cycle,
    bool pwm_inverted);

/**
 * @brief Assign a cooling system to an actuator.
 *
 * Associates @p system with @p actuator.
 *
 * Before the new system is assigned, the actuator PWM output is set to 0% and
 * its active state and stored duty cycle are reset.
 *
 * The assigned cooling system must remain valid until it is detached, replaced,
 * or the actuator is destroyed.
 *
 * @param[in,out] actuator Pointer to an initialized cooling actuator.
 * @param[in] system Pointer to an initialized cooling system.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if @p actuator or @p system is NULL
 *      - ESP_ERR_INVALID_STATE if the actuator or cooling system is not
 *        initialized or is otherwise invalid
 *      - ESP-IDF LEDC error code if the PWM output cannot be reset
 */
esp_err_t cooling_actuator_set_system(cooling_actuator_t *actuator, cooling_system_t *system);

/**
 * @brief Detach the cooling system from an actuator.
 *
 * Sets the actuator PWM output to 0%, clears the current cooling-system
 * reference, and resets the actuator active state and stored duty cycle.
 *
 * After detaching, cooling_actuator_update() cannot be called successfully
 * until another cooling system is assigned.
 *
 * @param[in,out] actuator Pointer to an initialized cooling actuator.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if @p actuator is NULL
 *      - ESP_ERR_INVALID_STATE if the actuator is not initialized
 *      - ESP-IDF LEDC error code if the PWM output cannot be reset
 */
esp_err_t cooling_actuator_detach_system(cooling_actuator_t *actuator);

/**
 * @brief Destroy a cooling actuator and release its PWM channel.
 *
 * Attempts to set the actuator PWM output to 0%, releases the internally
 * allocated LEDC channel, and resets the actuator structure.
 *
 * The associated cooling system and its cooled devices are not modified or
 * destroyed.
 *
 * Calling this function with a non-initialized actuator is safe and resets the
 * structure to its default invalid state.
 *
 * @param[in,out] actuator Pointer to the actuator to destroy.
 */
void cooling_actuator_destroy(cooling_actuator_t *actuator);



/* =========================================================================================================
 * Cooling Control
 */

/**
 * @brief Update the actuator state and PWM duty cycle.
 *
 * Evaluates all cooled devices registered in the actuator's cooling system,
 * updates the actuator active state using hysteresis, computes the cooling
 * demand of each device, and applies the highest requested duty cycle.
 *
 * For each device:
 *
 * - No proportional duty is requested at or below the device threshold.
 * - Above the threshold, duty increases according to
 *   @ref cooling_actuator_t::slope_pct_per_degC.
 * - At or above @ref cooled_device_t::full_duty_temp_mc, the device requests
 *   100% duty.
 *
 * Once the actuator becomes active, it remains active until all registered
 * devices fall below their respective threshold minus the configured
 * hysteresis. While active, the applied duty cycle is never lower than
 * @ref cooling_actuator_t::min_duty_cycle.
 *
 * The application must update each device's temperature before calling this
 * function.
 *
 * @param[in,out] actuator Pointer to an initialized actuator with an assigned
 *        cooling system containing at least one valid cooled device.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if @p actuator is NULL or a registered device has
 *        an invalid configuration
 *      - ESP_ERR_INVALID_STATE if the actuator is not initialized, no cooling
 *        system is assigned, the cooling system is invalid, or it contains no
 *        devices
 *      - ESP-IDF LEDC error code if the PWM duty cannot be updated
 */
esp_err_t cooling_actuator_update(cooling_actuator_t *actuator);

