#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"
#include "class/vendor/vendor_device.h"

#include "led_strip.h"

static const char *USB_TAG = "USB_BULK_AUDIO";

#define SAMPLE_RATE 48000
#define TONE_VOLUME 0.25f
#define EP_VENDOR_OUT 0x02
#define EP_VENDOR_IN 0x82
#define EP_SIZE 64

#define LED_GPIO 48
#define LED_COUNT 1
#define LED_BRIGHTNESS 4

#define TUSB_CFG_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_VENDOR * TUD_VENDOR_DESC_LEN)

#define SAFE_DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms) < 1 ? 1 : pdMS_TO_TICKS(ms))

typedef enum
{
    LED_STATE_BOOT,
    LED_STATE_IDLE,
    LED_STATE_STREAMING
} led_state_t;

static float phase = 0.0f;
static float current_freq = 440.0f;
static led_strip_handle_t led_strip;

static volatile bool usb_connected = false;
static volatile bool streaming_started = false;

static void led_set_state(led_state_t state);

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

static void led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (led_strip == NULL)
    {
        return;
    }

    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

static void led_set_state(led_state_t state)
{
    switch (state)
    {
    case LED_STATE_BOOT:
        led_set_rgb(0, 0, LED_BRIGHTNESS);
        break;

    case LED_STATE_IDLE:
        led_set_rgb(LED_BRIGHTNESS, LED_BRIGHTNESS * 3 / 4, 0);
        break;

    case LED_STATE_STREAMING:
        led_set_rgb(0, LED_BRIGHTNESS, 0);
        break;
    }
}

static void tinyusb_event_cb(tinyusb_event_t *event, void *arg)
{
    (void)arg;

    switch (event->id)
    {
    case TINYUSB_EVENT_ATTACHED:
        usb_connected = true;
        streaming_started = false;
        led_set_state(LED_STATE_STREAMING);
        ESP_LOGI(USB_TAG, "USB Attached");
        break;

    case TINYUSB_EVENT_DETACHED:
        usb_connected = false;
        streaming_started = false;
        led_set_state(LED_STATE_IDLE);
        ESP_LOGI(USB_TAG, "USB Detached");
        break;

    default:
        break;
    }
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;

    usb_connected = false;
    streaming_started = false;
    led_set_state(LED_STATE_IDLE);

    ESP_LOGI(USB_TAG, "USB Suspended");
}

void tud_resume_cb(void)
{
    usb_connected = true;
    streaming_started = false;
    led_set_state(LED_STATE_STREAMING);

    ESP_LOGI(USB_TAG, "USB Resumed");
}

static void led_init_app(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_set_state(LED_STATE_BOOT);
}

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
        if (!usb_connected || !tud_mounted())
        {
            if (streaming_started)
            {
                streaming_started = false;
                led_set_state(LED_STATE_IDLE);
            }

            SAFE_DELAY_MS(20);
            continue;
        }

        static uint64_t last_usb_wait_log = 0;
        uint64_t now = esp_timer_get_time();

        uint32_t available = tud_vendor_write_available();

        if (available == 0)
        {
            if ((now - last_usb_wait_log) >= 1000000)
            {
                last_usb_wait_log = now;
                ESP_LOGI(USB_TAG, "mounted=%d ready=%d available=%lu",
                         tud_mounted(),
                         tud_ready(),
                         available);
            }

            SAFE_DELAY_MS(10);
            continue;
        }

        if (!streaming_started)
        {
            streaming_started = true;
            led_set_state(LED_STATE_STREAMING);
        }

        uint32_t write_len = available >= EP_SIZE ? EP_SIZE : available;

        if ((write_len % 2) != 0)
        {
            write_len -= 1;
        }

        if (write_len >= 2)
        {
            fill_audio_packet(packet, write_len);

            uint32_t written = tud_vendor_write(packet, write_len);

            if (written > 0 && usb_connected && tud_mounted())
            {
                tud_vendor_write_flush();
            }
        }

        SAFE_DELAY_MS(1);
    }
}

void app_main(void)
{
    led_init_app();

    ESP_LOGI(USB_TAG, "Starting USB Bulk Audio");

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.descriptor.device = &device_descriptor;
    tusb_cfg.descriptor.full_speed_config = fs_configuration_descriptor;
    tusb_cfg.event_cb = tinyusb_event_cb;
    tusb_cfg.event_arg = NULL;

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    usb_connected = tud_mounted();
    led_set_state(usb_connected ? LED_STATE_IDLE : LED_STATE_BOOT);

    xTaskCreatePinnedToCore(
        bulk_audio_task,
        "bulk_audio_task",
        4096,
        NULL,
        5,
        NULL,
        1);

    ESP_LOGI(USB_TAG, "USB Bulk Audio started");
}