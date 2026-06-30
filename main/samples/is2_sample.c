#include <stdint.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2s_std.h"

static const char *TAG = "PCM1808_STD";

#define I2S_SAMPLE_RATE 48000

#define I2S_MCLK GPIO_NUM_4
#define I2S_BCLK GPIO_NUM_15
#define I2S_WS GPIO_NUM_16
#define I2S_DIN GPIO_NUM_18

#define SAMPLE_WORDS 1024

static i2s_chan_handle_t rx_chan = NULL;
static int32_t samples[SAMPLE_WORDS];

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
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_24BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = 32,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false,
        },
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

    ESP_LOGI(TAG, "STD I2S role: ESP32-S3 MASTER");
    ESP_LOGI(TAG, "PCM1808 expected role: SLAVE");
    ESP_LOGI(TAG, "Format: 48kHz, stereo, 24-bit ADC data in 32-bit slot");
    ESP_LOGI(TAG, "Expected clocks: MCLK=12.288MHz BCLK=3.072MHz WS=48kHz");
    ESP_LOGI(
        TAG,
        "Pins: MCLK/SCK=GPIO%d BCLK=GPIO%d LRC/WS=GPIO%d DIN/OUT=GPIO%d",
        I2S_MCLK,
        I2S_BCLK,
        I2S_WS,
        I2S_DIN);
}

static void pcm1808_read_task(void *arg)
{
    size_t bytes_read = 0;
    uint32_t frame_index = 0;

    while (1)
    {
        esp_err_t ret = i2s_channel_read(
            rx_chan,
            samples,
            sizeof(samples),
            &bytes_read,
            portMAX_DELAY);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_read failed: %s", esp_err_to_name(ret));
            continue;
        }

        int count = bytes_read / sizeof(int32_t);
        int non_zero = 0;
        int32_t min_v = INT32_MAX;
        int32_t max_v = INT32_MIN;

        for (int i = 0; i < count; i++)
        {
            int32_t v = samples[i];

            if (v != 0)
            {
                non_zero++;
            }

            if (v < min_v)
            {
                min_v = v;
            }

            if (v > max_v)
            {
                max_v = v;
            }
        }

        int32_t l0_raw = samples[0];
        int32_t r0_raw = samples[1];
        int32_t l100_raw = samples[100];
        int32_t r100_raw = samples[101];

        int32_t l0_24 = l0_raw >> 8;
        int32_t r0_24 = r0_raw >> 8;
        int32_t l100_24 = l100_raw >> 8;
        int32_t r100_24 = r100_raw >> 8;

        ESP_LOGI(
            TAG,
            "frame=%lu I2S_RX=%s PCM1808_DATA=%s bytes=%u words=%d non_zero=%d min=%ld max=%ld L0=%ld R0=%ld L100=%ld R100=%ld L0_24=%ld R0_24=%ld L100_24=%ld R100_24=%ld",
            frame_index++,
            bytes_read > 0 ? "ACTIVE" : "NO",
            non_zero > 0 ? "DETECTED" : "EMPTY",
            (unsigned int)bytes_read,
            count,
            non_zero,
            min_v,
            max_v,
            l0_raw,
            r0_raw,
            l100_raw,
            r100_raw,
            l0_24,
            r0_24,
            l100_24,
            r100_24);

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void app_main(void)
{
    pcm1808_i2s_init();

    xTaskCreatePinnedToCore(
        pcm1808_read_task,
        "pcm1808_read_task",
        8192,
        NULL,
        5,
        NULL,
        1);
}