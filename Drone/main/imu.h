#ifndef IMU_H
#define IMU_H

#include "esp_err.h"

// I2C Configuration
#define I2C_MASTER_SDA_IO           13
#define I2C_MASTER_SCL_IO           14
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

// MPU9250 I2C Address
#define MPU9250_ADDR                0x68

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    float x;
    float y;
    float z;
} imu_vec3_t;

typedef struct {
    imu_vec3_t accel;
    imu_vec3_t gyro;
} imu_msg_t;

/**
 * @brief Initialize I2C and the MPU9250 sensor, and start the periodic reading task
 * @param imu_queue Queue to send sensor data messages to
 */
esp_err_t imu_init(QueueHandle_t imu_queue);

/**
 * @brief Read data from the accelerometer and gyroscope
 * @param accel Out pointer for accelerometer data (m/s^2)
 * @param gyro Out pointer for gyroscope data (deg/s)
 */
esp_err_t imu_read(imu_vec3_t *accel, imu_vec3_t *gyro);

/**
 * @brief Retrieve the latest cached IMU readings from the cyclic task
 */
void imu_get_latest(imu_vec3_t *accel, imu_vec3_t *gyro);

/**
 * @brief Perform Gyro Calibration (Drone must be stationary!)
 */
esp_err_t imu_calibrate(void);

#endif // IMU_H
