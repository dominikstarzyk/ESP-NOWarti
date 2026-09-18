#ifndef MOTORS_H
#define MOTORS_H

#include "esp_err.h"

// Motor GPIO Pins
#define MOTOR_LF_GPIO 5  // Left Forward
#define MOTOR_RF_GPIO 6  // Right Forward
#define MOTOR_RR_GPIO 7  // Right Rear (Bottom)
#define MOTOR_LR_GPIO 8  // Left Rear (Bottom)

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    float lf;
    float rf;
    float rr;
    float lr;
} motor_speeds_t;

/**
 * @brief Initialize the PWM for brushless motors and start the motor task consuming from the provided queue
 * @param motor_queue Queue from which the motor task will receive speed updates
 */
esp_err_t motors_init(QueueHandle_t motor_queue);

#endif // MOTORS_H
