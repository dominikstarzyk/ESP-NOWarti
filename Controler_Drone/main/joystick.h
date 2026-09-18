#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>
#include <stdbool.h>

//#define DEBUG_LOGSV2

/**
 * @brief Initialize the joystick module.
 * Sets up the ADC for VRX and VRY, and GPIO for the Switch (SW).
 */
void joystick_init(void);

/**
 * @brief Read analog and switch values from the joysticks.
 * 
 * @param roll Pointer to store the mapped roll value (-100.0 to 100.0)
 * @param pitch Pointer to store the mapped pitch value (-100.0 to 100.0)
 * @param yaw Pointer to store the mapped yaw value (-100.0 to 100.0)
 * @param throttle Pointer to store the mapped throttle value (-100.0 to 100.0)
 * @param sw1_pressed Pointer to store the state of Joystick 1 switch (true if pressed)
 * @param sw2_pressed Pointer to store the state of Joystick 2 switch (true if pressed)
 */
void joystick_read(float *roll, float *pitch, float *yaw, float *throttle, bool *sw1_pressed, bool *sw2_pressed);

/**
 * @brief FreeRTOS task that reads the joystick continuously
 * and queues the data to be sent via ESP-NOW.
 */
void joystick_task(void *pvParameters);

#endif // JOYSTICK_H
