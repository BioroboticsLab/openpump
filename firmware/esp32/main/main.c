#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/timers.h"

// Define constants for PWM (using LEDC peripheral)
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (18) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (7000) // Set duty to 7000/8191 (85%)
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

// Define constants for ADC
#define ADC_CHANNEL             ADC1_CHANNEL_6 // GPIO34 if ADC1
#define ADC_ATTEN               ADC_ATTEN_DB_11
#define ADC_WIDTH               ADC_WIDTH_BIT_12
#define DEFAULT_VREF            1100 // Use ADC calibration API to obtain a better estimate

// Define sensor threshold for capacitive sensor
#define SENSOR_THRESHOLD       2600 // tested with water and low concentration sugar solution

// Define digital output pin
#define DIGITAL_OUTPUT_IO       (19) // Define the output GPIO for digital output

// Define failsafe timeout in milliseconds
#define FAILSAFE_TIMEOUT        60000 // 1 minute

static esp_adc_cal_characteristics_t *adc_chars;

static TimerHandle_t failsafe_timer;

bool main_loop = true;

void init_pwm() {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0 initially
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    // Set duty to 50%
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void init_adc() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, adc_chars);
}

void init_gpio() {
    // Configure the digital output pin
    esp_rom_gpio_pad_select_gpio(DIGITAL_OUTPUT_IO);
    gpio_set_direction(DIGITAL_OUTPUT_IO, GPIO_MODE_OUTPUT);
}

void failsafe_timer_callback(TimerHandle_t xTimer) {
    // Stop the motor by setting digital output pin to LOW
    gpio_set_level(DIGITAL_OUTPUT_IO, 0);
    printf("Failsafe activated: Motor stopped due to timeout.\n");
    main_loop = false;
}

void app_main(void) {
    // Initialize PWM, ADC, and GPIO
    init_pwm();
    init_adc();
    init_gpio();

    // Create failsafe timer
    failsafe_timer = xTimerCreate("FailsafeTimer", pdMS_TO_TICKS(FAILSAFE_TIMEOUT), pdFALSE, (void *)0, failsafe_timer_callback);
    if (failsafe_timer == NULL) {
        printf("Failed to create failsafe timer\n");
        return;
    }

    while (main_loop) {
        // Read ADC and convert the raw value to voltage in mV
        uint32_t adc_reading = adc1_get_raw(ADC_CHANNEL);
        printf("\rSensor Reading: %d - ", (int) adc_reading);

        // Check if sensor reading is below the threshold (means capacitive sensor is wet)
        if (adc_reading < SENSOR_THRESHOLD) {
            // Set digital output pin to LOW
            gpio_set_level(DIGITAL_OUTPUT_IO, 0);
            printf("Sensor wet, stop motor.");

            // Stop the failsafe timer if it is running
            if (xTimerIsTimerActive(failsafe_timer) == pdTRUE) {
                xTimerStop(failsafe_timer, 0);
                printf("Failsafe timer stopped.\n");
            }

        } else {
            // Set digital output pin to HIGH
            gpio_set_level(DIGITAL_OUTPUT_IO, 1);
            printf("Sensor dry, start motor.");

            // Start the failsafe timer if it is not running
            if (xTimerIsTimerActive(failsafe_timer) == pdFALSE) {
                xTimerStart(failsafe_timer, 0);
                printf("Failsafe timer started.\n");
            }


        }

        vTaskDelay(pdMS_TO_TICKS(250)); // Delay for a second
    }
}
