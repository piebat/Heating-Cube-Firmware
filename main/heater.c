#include "heater.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "heater";

/* DC elements switched by MOSFET gates. 1 kHz is well above the thermal
 * response of the element and low enough that gate switching losses stay
 * negligible with a modest gate driver. */
#define HEATER_FREQ_HZ      1000
#define HEATER_RES          LEDC_TIMER_12_BIT
#define HEATER_MAX_DUTY     ((1 << 12) - 1)
#define HEATER_MODE         LEDC_LOW_SPEED_MODE
#define HEATER_TIMER        LEDC_TIMER_0

/* One gate pin and one LEDC channel per zone, all off the shared timer. */
static const int s_gpio[HEATER_ZONE_COUNT] = {
    GPIO_NUM_7,
    GPIO_NUM_8,
    GPIO_NUM_9,
};
static const ledc_channel_t s_channel[HEATER_ZONE_COUNT] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
};

static float s_duty[HEATER_ZONE_COUNT];

esp_err_t heater_init(void)
{
    /* Hold every gate down before the LEDC peripheral takes the pins, so no
     * element can pulse during boot. */
    uint64_t pins = 0;
    for (int z = 0; z < HEATER_ZONE_COUNT; z++) {
        pins |= 1ULL << s_gpio[z];
    }
    gpio_config_t io = {
        .pin_bit_mask = pins,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio config");
    for (int z = 0; z < HEATER_ZONE_COUNT; z++) {
        gpio_set_level(s_gpio[z], 0);
    }

    ledc_timer_config_t timer = {
        .speed_mode = HEATER_MODE,
        .timer_num = HEATER_TIMER,
        .duty_resolution = HEATER_RES,
        .freq_hz = HEATER_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "ledc timer");

    for (int z = 0; z < HEATER_ZONE_COUNT; z++) {
        ledc_channel_config_t channel = {
            .gpio_num = s_gpio[z],
            .speed_mode = HEATER_MODE,
            .channel = s_channel[z],
            .timer_sel = HEATER_TIMER,
            .duty = 0,
            .hpoint = 0,
            .intr_type = LEDC_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "ledc channel %d", z);

        s_duty[z] = 0.0f;
        ESP_LOGI(TAG, "heater zone %d ready on GPIO%d, %d Hz PWM",
                 z, s_gpio[z], HEATER_FREQ_HZ);
    }
    return ESP_OK;
}

void heater_set_duty(int zone, float duty)
{
    if (zone < 0 || zone >= HEATER_ZONE_COUNT) {
        return;
    }
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    s_duty[zone] = duty;

    uint32_t raw = (uint32_t)(duty * HEATER_MAX_DUTY + 0.5f);
    ledc_set_duty(HEATER_MODE, s_channel[zone], raw);
    ledc_update_duty(HEATER_MODE, s_channel[zone]);
}

float heater_get_duty(int zone)
{
    if (zone < 0 || zone >= HEATER_ZONE_COUNT) {
        return 0.0f;
    }
    return s_duty[zone];
}

void heater_off(int zone)
{
    if (zone < 0 || zone >= HEATER_ZONE_COUNT) {
        return;
    }
    s_duty[zone] = 0.0f;
    /* stop() parks the output at the idle level directly, without waiting
     * for the current PWM period to finish. */
    ledc_stop(HEATER_MODE, s_channel[zone], 0);
}

void heater_off_all(void)
{
    for (int z = 0; z < HEATER_ZONE_COUNT; z++) {
        heater_off(z);
    }
}
