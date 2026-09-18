/* Drone Project Main
   Basic template for ESP-IDF
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_now_receive.h"
#include "motors.h"
#include "imu.h"
#include "flight_controller.h"
#include "rgb_led.h"

static const char *TAG = "drone_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Drone Project Started");

    // Initialize RGB LED (GPIO 48 is standard for S3 DevKits)
    rgb_led_init(RGB_LED_GPIO);
    rgb_led_set_error(); // Start with Red (No connection)

    // Create the motor queue in main
    QueueHandle_t motor_queue = xQueueCreate(10, sizeof(motor_speeds_t));
    
    // Create the IMU queue for the flight controller
    QueueHandle_t imu_queue = xQueueCreate(10, sizeof(imu_msg_t));
    
    // Initialize ESP-NOW Receiver with the motor queue
    esp_now_receive_init(motor_queue);

    // Initialize Motor PWM task with the same queue
    motors_init(motor_queue);

    // Initialize IMU (MPU9250) and tell it where to send data
    imu_init(imu_queue);

    // Initialize Flight Controller to consume IMU data
    flight_controller_init(imu_queue);

    // The rest is handled by the ESP-NOW task in esp_now_receive.c
}
