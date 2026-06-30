#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "tusb.h"
#include "tinyusb_default_config.h"
#include "class/vendor/vendor_device.h"
#include "esp_task_wdt.h"
#include "esp_err.h"

#include "audio.h" // your generated PCM header

static const char *TAG = "USB_PCM";

#define EP_VENDOR_OUT 0x02
#define EP_VENDOR_IN 0x82

#define TUSB_CFG_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_VENDOR * TUD_VENDOR_DESC_LEN)

#define EP_SIZE 64
#define TX_CHUNK 64

static const uint8_t fs_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, CFG_TUD_VENDOR, 0, TUSB_CFG_DESC_TOTAL_LEN, 0x00, 100),
    TUD_VENDOR_DESCRIPTOR(0, 0, EP_VENDOR_OUT, EP_VENDOR_IN, EP_SIZE),
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

static const char *string_desc[] = {
    "ESP32-S3",
    "PCM Bulk Audio",
    "123456",
    "Vendor Interface",
};

static void usb_pcm_stream_task(void *arg)
{
    uint32_t offset = 0;

    while (1)
    {
        if (!tud_mounted())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        for (int i = 0; i < 16; i++)
        {
            uint32_t available = tud_vendor_write_available();

            if (available == 0)
            {
                break;
            }

            uint32_t remaining = audio_pcm_len - offset;

            if (remaining == 0)
            {
                offset = 0;
                remaining = audio_pcm_len;
            }

            uint32_t chunk = remaining;

            if (chunk > available)
            {
                chunk = available;
            }

            if (chunk > TX_CHUNK)
            {
                chunk = TX_CHUNK;
            }

            uint32_t written = tud_vendor_write((void *)(audio_pcm + offset), chunk);

            if (written == 0)
            {
                break;
            }

            offset += written;

            if (offset >= audio_pcm_len)
            {
                offset = 0;
            }
        }

        tud_vendor_flush();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();

    tusb_cfg.descriptor.device = &device_descriptor;
    tusb_cfg.descriptor.string = string_desc;
    tusb_cfg.descriptor.string_count = sizeof(string_desc) / sizeof(string_desc[0]);
    tusb_cfg.descriptor.full_speed_config = fs_configuration_descriptor;

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    esp_err_t wdt_ret = esp_task_wdt_delete(xTaskGetIdleTaskHandleForCore(1));
    if (wdt_ret != ESP_OK && wdt_ret != ESP_ERR_NOT_FOUND)
    {
        ESP_LOGW(TAG, "Failed to remove IDLE1 from WDT: %s", esp_err_to_name(wdt_ret));
    }

    xTaskCreatePinnedToCore(
        usb_pcm_stream_task,
        "usb_pcm_stream_task",
        4096,
        NULL,
        1,
        NULL,
        1);

    ESP_LOGI(TAG, "USB PCM bulk streaming started");
}