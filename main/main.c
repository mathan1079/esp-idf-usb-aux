#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"

#include "tinyusb.h"
#include "tusb.h"
#include "tinyusb_default_config.h"
#include "class/vendor/vendor_device.h"

static const char *TAG = "PCM_USB_BULK";

#define EP_VENDOR_OUT 0x02
#define EP_VENDOR_IN 0x82

#define EP_SIZE 64
#define TX_CHUNK 64

#define TUSB_CFG_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_VENDOR * TUD_VENDOR_DESC_LEN)

#define I2S_SAMPLE_RATE 48000
#define I2S_MCLK GPIO_NUM_4
#define I2S_BCLK GPIO_NUM_15
#define I2S_WS GPIO_NUM_16
#define I2S_DIN GPIO_NUM_18

#define I2S_SAMPLE_WORDS 1024
#define AUDIO_RING_SIZE 32768

static i2s_chan_handle_t rx_chan = NULL;
static int32_t i2s_samples[I2S_SAMPLE_WORDS];
static int16_t usb_samples[I2S_SAMPLE_WORDS / 2];

static uint8_t audio_ring[AUDIO_RING_SIZE];
static volatile uint32_t ring_head = 0;
static volatile uint32_t ring_tail = 0;
static volatile uint32_t ring_used = 0;
static volatile uint32_t ring_overflow_count = 0;
static volatile uint32_t usb_short_write_count = 0; /* NEW: tracks partial tud_vendor_write() calls */
static volatile uint32_t usb_dropped_bytes = 0;     /* NEW: bytes dropped due to short writes */
static portMUX_TYPE ring_mux = portMUX_INITIALIZER_UNLOCKED;

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

static uint32_t ring_free_nolock(void)
{
    return AUDIO_RING_SIZE - ring_used;
}

static uint32_t ring_used_bytes(void)
{
    uint32_t used;
    portENTER_CRITICAL(&ring_mux);
    used = ring_used;
    portEXIT_CRITICAL(&ring_mux);
    return used;
}

static uint32_t ring_write_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t written = 0;

    portENTER_CRITICAL(&ring_mux);

    uint32_t free_bytes = ring_free_nolock();
    if (len > free_bytes)
    {
        ring_overflow_count++;
        len = free_bytes;
        len &= ~((uint32_t)1); /* round down to even: never split a 16-bit sample */
    }

    while (written < len)
    {
        uint32_t chunk = len - written;
        uint32_t to_end = AUDIO_RING_SIZE - ring_head;

        if (chunk > to_end)
        {
            chunk = to_end;
        }

        memcpy(&audio_ring[ring_head], data + written, chunk);
        ring_head = (ring_head + chunk) % AUDIO_RING_SIZE;
        ring_used += chunk;
        written += chunk;
    }

    portEXIT_CRITICAL(&ring_mux);
    return written;
}

static uint32_t ring_read_bytes(uint8_t *out, uint32_t len)
{
    uint32_t read = 0;

    portENTER_CRITICAL(&ring_mux);

    if (len > ring_used)
    {
        len = ring_used;
    }

    while (read < len)
    {
        uint32_t chunk = len - read;
        uint32_t to_end = AUDIO_RING_SIZE - ring_tail;

        if (chunk > to_end)
        {
            chunk = to_end;
        }

        memcpy(out + read, &audio_ring[ring_tail], chunk);
        ring_tail = (ring_tail + chunk) % AUDIO_RING_SIZE;
        ring_used -= chunk;
        read += chunk;
    }

    portEXIT_CRITICAL(&ring_mux);
    return read;
}

static void pcm1808_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = 256;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = I2S_SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    ESP_LOGI(TAG, "I2S STD started: ESP32-S3 MASTER, PCM1808 SLAVE");
    ESP_LOGI(TAG, "Format: 48kHz stereo, 24-bit ADC data in 32-bit slot");
    ESP_LOGI(TAG, "Pins: MCLK/SCK=GPIO%d BCLK=GPIO%d LRC/WS=GPIO%d DIN/OUT=GPIO%d", I2S_MCLK, I2S_BCLK, I2S_WS, I2S_DIN);
}

static void tinyusb_vendor_init(void)
{
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();

    tusb_cfg.descriptor.device = &device_descriptor;
    tusb_cfg.descriptor.string = string_desc;
    tusb_cfg.descriptor.string_count = sizeof(string_desc) / sizeof(string_desc[0]);
    tusb_cfg.descriptor.full_speed_config = fs_configuration_descriptor;

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB Vendor Bulk started");
}

static void i2s_capture_task(void *arg)
{
    size_t bytes_read = 0;

    while (1)
    {
        esp_err_t ret = i2s_channel_read(
            rx_chan,
            i2s_samples,
            sizeof(i2s_samples),
            &bytes_read,
            portMAX_DELAY);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_read failed: %s", esp_err_to_name(ret));
            continue;
        }

        if (bytes_read > 0)
        {
            int words = bytes_read / sizeof(int32_t);
            int out = 0;

            for (int i = 0; i < words; i += 1)
            {
                int32_t s = i2s_samples[i] >> 16;
                usb_samples[out++] = (int16_t)s;
            }

            ring_write_bytes(
                (const uint8_t *)usb_samples,
                out * sizeof(int16_t));
        }
    }
}

/*
 * FIX: previously, when tud_vendor_write() accepted fewer bytes than we
 * asked it to (a "short write"), the leftover bytes were pushed back into
 * the ring with ring_write_bytes(), which appends at the HEAD (newest
 * position). That re-inserted OLDER, already-read samples behind whatever
 * the capture task wrote in the meantime, scrambling playback order and
 * producing the robotic/garbled sound.
 *
 * Fix: never re-inject leftover bytes at the head. Just drop them — this
 * causes a tiny (sub-millisecond at this chunk size) gap, never a
 * reorder. We also now track how often this happens via
 * usb_short_write_count / usb_dropped_bytes so you can see if it's
 * actually occurring in your setup.
 */
static void usb_pcm_stream_task(void *arg)
{
    uint8_t tx_buf[TX_CHUNK];
    uint32_t log_counter = 0;

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

            uint32_t chunk = available;

            if (chunk > TX_CHUNK)
            {
                chunk = TX_CHUNK;
            }

            chunk &= ~((uint32_t)1); /* round down to even: never request a half-sample from the ring */

            if (chunk == 0)
            {
                break;
            }

            uint32_t got = ring_read_bytes(tx_buf, chunk);

            if (got == 0)
            {
                break;
            }

            uint32_t written = tud_vendor_write(tx_buf, got);

            if (written < got)
            {
                /* Do NOT re-inject leftover bytes into the ring here —
                 * that corrupts sample ordering. Drop them and track it. */
                usb_short_write_count++;
                usb_dropped_bytes += (got - written);
                break;
            }
        }

        tud_vendor_flush();

        if (++log_counter % 1000 == 0)
        {
            ESP_LOGI(TAG, "USB ring=%u overflow=%lu short_writes=%lu dropped=%lu mounted=%d",
                     (unsigned int)ring_used_bytes(),
                     (unsigned long)ring_overflow_count,
                     (unsigned long)usb_short_write_count,
                     (unsigned long)usb_dropped_bytes,
                     tud_mounted());
        }
        taskYIELD();
        // vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    tinyusb_vendor_init();
    pcm1808_i2s_init();

    esp_err_t wdt_ret = esp_task_wdt_delete(xTaskGetIdleTaskHandleForCore(1));
    if (wdt_ret != ESP_OK && wdt_ret != ESP_ERR_NOT_FOUND)
    {
        ESP_LOGW(TAG, "Failed to remove IDLE1 from WDT: %s", esp_err_to_name(wdt_ret));
    }

    xTaskCreatePinnedToCore(
        i2s_capture_task,
        "i2s_capture_task",
        8192,
        NULL,
        6,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        usb_pcm_stream_task,
        "usb_pcm_stream_task",
        4096,
        NULL,
        5,
        NULL,
        1);

    ESP_LOGI(TAG, "PCM1808 to USB Bulk streaming started");
}