#ifndef SERIAL_CONNECTION_H
#define SERIAL_CONNECTION_H

#include <stdbool.h>

/* Switch to choose between Serial connection or Static values */
#define USE_SERIAL_CONNECTION 0 // Set to 1 for Serial, 0 for Joystick

/**
 * @brief Initialize the serial connection (UART)
 */
void serial_connection_init(void);

#endif // SERIAL_CONNECTION_H
