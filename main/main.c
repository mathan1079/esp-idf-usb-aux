#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "usb_device_uac.h"

static const char *TAG = "USB_UAC";

#ifndef CONFIG_UAC_SAMPLE_RATE
#define CONFIG_UAC_SAMPLE_RATE 48000
#endif

#ifndef CONFIG_UAC_BYTES_PER_SAMPLE
#define CONFIG_UAC_BYTES_PER_SAMPLE 2
#endif

#ifndef CONFIG_UAC_MIC_CHANNEL_NUM
#define CONFIG_UAC_MIC_CHANNEL_NUM 1
#endif

#define TONE_VOLUME 0.25f

static float phase = 0.0f;
static float current_freq = 440.0f;

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
    // We are not receiving audio from host yet.
    return ESP_OK;
}

static esp_err_t uac_device_input_cb(uint8_t *buf,
                                     size_t len,
                                     size_t *bytes_read,
                                     void *arg)
{
    static uint64_t last_change = 0;

    uint64_t now = esp_timer_get_time();

    if ((now - last_change) >= 1000000)
    {
        last_change = now;

        if (current_freq == 440.0f)
        {
            current_freq = 880.0f;
        }
        else
        {
            current_freq = 440.0f;
        }

        ESP_LOGI(TAG, "Generating %.0f Hz tone", current_freq);
    }

    int16_t *samples = (int16_t *)buf;

    size_t sample_count = len / sizeof(int16_t);

    float phase_step =
        2.0f *
        (float)M_PI *
        current_freq /
        (float)CONFIG_UAC_SAMPLE_RATE;

    for (size_t i = 0; i < sample_count; i += CONFIG_UAC_MIC_CHANNEL_NUM)
    {

        int16_t sample =
            (int16_t)(sinf(phase) * 32767.0f * TONE_VOLUME);

        for (int ch = 0; ch < CONFIG_UAC_MIC_CHANNEL_NUM; ch++)
        {
            samples[i + ch] = sample;
        }

        phase += phase_step;

        if (phase >= (2.0f * (float)M_PI))
        {
            phase -= (2.0f * (float)M_PI);
        }
    }

    *bytes_read = len;

    return ESP_OK;
}

static void uac_device_set_mute_cb(uint32_t mute, void *arg)
{
    ESP_LOGI(TAG, "Mute: %" PRIu32, mute);
}

static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
    ESP_LOGI(TAG, "Volume: %" PRIu32, volume);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting USB Audio Device");

    uac_device_config_t config = {
        .output_cb = uac_device_output_cb,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
        .cb_ctx = NULL,
    };

    ESP_ERROR_CHECK(uac_device_init(&config));

    ESP_LOGI(TAG, "USB Audio initialized");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}