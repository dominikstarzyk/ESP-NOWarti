#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

typedef struct {
    float roll;     // Degrees
    float pitch;    // Degrees
} drone_attitude_t;

/**
 * @brief Initialize the flight controller task
 * @param imu_queue Queue from which the flight controller will receive IMU data
 */
esp_err_t flight_controller_init(QueueHandle_t imu_queue);

/**
 * @brief Get the current calculated attitude of the drone
 */
void flight_controller_get_attitude(drone_attitude_t *attitude);

#endif // FLIGHT_CONTROLLER_H
