#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"
#include "class/vendor/vendor_device.h"

static const char *USB_TAG = "USB_BULK_AUDIO";

#define SAMPLE_RATE 48000
#define TONE_VOLUME 0.25f
#define EP_VENDOR_OUT 0x02
#define EP_VENDOR_IN 0x82
#define EP_SIZE 64

#define TUSB_CFG_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_VENDOR * TUD_VENDOR_DESC_LEN)

static float phase = 0.0f;
static float current_freq = 440.0f;

static const uint8_t fs_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, CFG_TUD_VENDOR, 0, TUSB_CFG_DESC_TOTAL_LEN, 0x00, 100),
    TUD_VENDOR_DESCRIPTOR(0, 4, EP_VENDOR_OUT, EP_VENDOR_IN, EP_SIZE),
};

static const tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(device_descriptor),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_VENDOR_SPECIFIC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,
    .idProduct = 0x4040,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

static void fill_audio_packet(uint8_t *buf, size_t len)
{
    static uint64_t last_change = 0;
    uint64_t now = esp_timer_get_time();

    if ((now - last_change) >= 1000000)
    {
        last_change = now;
        current_freq = current_freq == 440.0f ? 880.0f : 440.0f;
        ESP_LOGI(USB_TAG, "Tone %.0f Hz", current_freq);
    }

    int16_t *samples = (int16_t *)buf;
    size_t sample_count = len / sizeof(int16_t);
    float phase_step = 2.0f * (float)M_PI * current_freq / (float)SAMPLE_RATE;

    for (size_t i = 0; i < sample_count; i++)
    {
        samples[i] = (int16_t)(sinf(phase) * 32767.0f * TONE_VOLUME);
        phase += phase_step;

        if (phase >= 2.0f * (float)M_PI)
        {
            phase -= 2.0f * (float)M_PI;
        }
    }
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP)
    {
        return true;
    }
    return false;
}

static void bulk_audio_task(void *arg)
{
    uint8_t packet[EP_SIZE];

    while (1)
    {
        if (!tud_mounted())
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        uint32_t available = tud_vendor_write_available();

        if (available > 0)
        {
            uint32_t write_len = available >= EP_SIZE ? EP_SIZE : available;

            if ((write_len % 2) != 0)
            {
                write_len -= 1;
            }

            if (write_len >= 2)
            {
                fill_audio_packet(packet, write_len);
                uint32_t written = tud_vendor_write(packet, write_len);
                tud_vendor_write_flush();

                if (written == 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(USB_TAG, "Starting USB Bulk Audio");

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.descriptor.device = &device_descriptor;
    tusb_cfg.descriptor.full_speed_config = fs_configuration_descriptor;

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    xTaskCreatePinnedToCore(
        bulk_audio_task,
        "bulk_audio_task",
        4096,
        NULL,
        5,
        NULL,
        0);

    ESP_LOGI(USB_TAG, "USB Bulk Audio started");
}