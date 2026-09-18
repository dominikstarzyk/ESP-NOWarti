#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static imu_vec3_t g_accel = {0};
static imu_vec3_t g_gyro = {0};
static imu_vec3_t g_gyro_bias = {0};

static const char *TAG = "MPU9250";

// MPU9250 Registers
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_WHO_AM_I        0x75
#define MPU9250_ACCEL_XOUT_H    0x3B
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_CONFIG          0x1A
#define MPU9250_ACCEL_CONFIG_2  0x1D

// Calibration/Sensitivity
// Set to ±8g and ±2000 deg/s
#define ACCEL_SENSITIVITY       4096.0f   // LSB/g
#define GYRO_SENSITIVITY        16.4f     // LSB/(deg/s)
#define GRAVITY                 9.81f

static esp_err_t i2c_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static QueueHandle_t external_imu_queue = NULL;

static void imu_task(void *pvParameters)
{
    imu_vec3_t accel, gyro;

    while (1) {
        if (imu_read(&accel, &gyro) == ESP_OK) {
            // Update global state
            g_accel = accel;
            g_gyro = gyro;

            // Send to external flight controller queue
            if (external_imu_queue != NULL) {
                imu_msg_t msg = { .accel = accel, .gyro = gyro };
                xQueueSend(external_imu_queue, &msg, 0);
            }
        }
        
        // Sampling frequency (100Hz = 10ms)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t imu_init(QueueHandle_t imu_queue)
{
    external_imu_queue = imu_queue;
    // Configure I2C Master
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    // Initial check (Who Am I)
    uint8_t who_am_i = 0x00;
    esp_err_t err = i2c_read_bytes(MPU9250_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register (I2C error: %s). Check hardware wiring/pins!", esp_err_to_name(err));
        return err;
    }
    if (who_am_i != 0x71 && who_am_i != 0x73 && who_am_i != 0x79) { // 0x71 is common, 0x73/0x79 seen in variants/clones
        ESP_LOGW(TAG, "Unexpected WHO_AM_I: 0x%02X", who_am_i);
    }

    // Wake up sensor (Clear sleep bit)
    ESP_ERROR_CHECK(i2c_write_byte(MPU9250_PWR_MGMT_1, 0x01)); // Auto select clock

    // Configure Gyro (±2000 dps)
    ESP_ERROR_CHECK(i2c_write_byte(MPU9250_GYRO_CONFIG, 0x18)); // 0x18 = 11 << 3

    // Configure Accel (±8g)
    ESP_ERROR_CHECK(i2c_write_byte(MPU9250_ACCEL_CONFIG, 0x10)); // 0x10 = 10 << 3

    // Configure Digital Low Pass Filter (DLPF)
    // 0x03 corresponds to ~41Hz bandwidth for both Gyro and Accel.
    // This significantly reduces motor vibration noise.
    ESP_ERROR_CHECK(i2c_write_byte(MPU9250_CONFIG, 0x03));          // Gymo DLPF
    ESP_ERROR_CHECK(i2c_write_byte(MPU9250_ACCEL_CONFIG_2, 0x03));  // Accel DLPF

    // Perform initial Gyro Calibration
    imu_calibrate();

    // Start IMU cyclic reading task
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "MPU9250 Initialized. Cyclic task started (100Hz).");
    return ESP_OK;
}

void imu_get_latest(imu_vec3_t *accel, imu_vec3_t *gyro)
{
    if (accel) *accel = g_accel;
    if (gyro) *gyro = g_gyro;
}

esp_err_t imu_read(imu_vec3_t *accel, imu_vec3_t *gyro)
{
    uint8_t data[14];
    esp_err_t ret = i2c_read_bytes(MPU9250_ACCEL_XOUT_H, data, 14);
    if (ret != ESP_OK) return ret;

    // Decode raw bits
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];
    // Temperature (data[6], data[7]) ignored for now
    int16_t gx = (data[8] << 8) | data[9];
    int16_t gy = (data[10] << 8) | data[11];
    int16_t gz = (data[12] << 8) | data[13];

    // Convert to engineering units
    accel->x = (float)ax / ACCEL_SENSITIVITY * GRAVITY;
    accel->y = (float)ay / ACCEL_SENSITIVITY * GRAVITY;
    accel->z = (float)az / ACCEL_SENSITIVITY * GRAVITY;

    gyro->x = ((float)gx / GYRO_SENSITIVITY) - g_gyro_bias.x;
    gyro->y = ((float)gy / GYRO_SENSITIVITY) - g_gyro_bias.y;
    gyro->z = ((float)gz / GYRO_SENSITIVITY) - g_gyro_bias.z;

    return ESP_OK;
}

esp_err_t imu_calibrate(void)
{
    ESP_LOGI(TAG, "Calibrating Gyro... keep drone still.");
    imu_vec3_t temp_accel, temp_gyro;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 200;
    
    for (int i = 0; i < samples; i++) {
        if (imu_read(&temp_accel, &temp_gyro) == ESP_OK) {
            sum_x += temp_gyro.x;
            sum_y += temp_gyro.y;
            sum_z += temp_gyro.z;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    g_gyro_bias.x = sum_x / samples;
    g_gyro_bias.y = sum_y / samples;
    g_gyro_bias.z = sum_z / samples;
    
    ESP_LOGI(TAG, "Calibration Done. Offsets: X:%.2f Y:%.2f Z:%.2f", 
             g_gyro_bias.x, g_gyro_bias.y, g_gyro_bias.z);
    
    return ESP_OK;
}
