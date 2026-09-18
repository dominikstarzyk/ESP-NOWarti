#include "rgb_led.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include <string.h>

static const char *TAG = "rgb_led";
static spi_device_handle_t spi_handle = NULL;

// Use SPI2 (GPSPI2) as it's the most common for general purpose on S3
#define LED_SPI_HOST SPI2_HOST

esp_err_t rgb_led_init(int gpio_num)
{
    ESP_LOGI(TAG, "Initializing built-in RGB LED on GPIO %d (SPI Mode)", gpio_num);

    spi_bus_config_t buscfg = {
        .mosi_io_num = gpio_num,
        .miso_io_num = -1,
        .sclk_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    esp_err_t ret = spi_bus_initialize(LED_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { // INVALID_STATE means already initialized
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2400000, // 2.4 MHz
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_NO_DUMMY,
        .duty_cycle_pos = 128,
    };

    ret = spi_bus_add_device(LED_SPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initial clear
    return rgb_led_set_color(0, 0, 0);
}

/**
 * @brief Encodes 8 bits of color into 24 bits of SPI data (3 SPI bits per 1 LED bit)
 * 1 bit -> 0b110
 * 0 bit -> 0b100
 */
static void encode_byte(uint8_t val, uint8_t *out) {
    uint32_t acc = 0;
    for (int i = 7; i >= 0; --i) {
        uint8_t bit = (val >> i) & 1;
        acc = (acc << 3) | (bit ? 0b110 : 0b100);
    }
    out[0] = (acc >> 16) & 0xFF;
    out[1] = (acc >> 8) & 0xFF;
    out[2] = acc & 0xFF;
}

esp_err_t rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    if (spi_handle == NULL) return ESP_ERR_INVALID_STATE;

    uint8_t tx[9]; // 3 colors * 3 bytes each = 9 bytes
    encode_byte(green, tx + 0); // WS2812 is GRB
    encode_byte(red,   tx + 3);
    encode_byte(blue,  tx + 6);

    spi_transaction_t t = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
    }

    // Reset pulse (at least 80us low)
    esp_rom_delay_us(80);
    
    return ret;
}

void rgb_led_set_error(void)
{
    ESP_LOGI(TAG, "LED -> RED");
    rgb_led_set_color(255, 0, 0);
}

void rgb_led_set_ready(void)
{
    ESP_LOGI(TAG, "LED -> GREEN");
    rgb_led_set_color(0, 255, 0);
}

void rgb_led_set_blue(void)
{
    ESP_LOGI(TAG, "LED -> BLUE");
    rgb_led_set_color(0, 0, 255);
}

void rgb_led_set_orange(void)
{
    ESP_LOGI(TAG, "LED -> ORANGE");
    rgb_led_set_color(255, 128, 0);
}
