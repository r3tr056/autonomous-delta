#ifndef USB_SERIAL_TRANSPORT_H
#define USB_SERIAL_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Forward declaration to avoid circular dependency
struct uxrCustomTransport;

/**
 * USB Serial Transport for micro-ROS on ESP32
 * 
 * This transport uses UART2 (GPIO 16/17) to communicate with an external 
 * USB-to-serial converter connected to a Raspberry Pi via USB.
 * 
 * Hardware Setup:
 * - Connect ESP32 GPIO 17 (TX) to USB-to-serial converter RX
 * - Connect ESP32 GPIO 16 (RX) to USB-to-serial converter TX  
 * - Connect GND between ESP32 and USB-to-serial converter
 * - Connect USB-to-serial converter to Raspberry Pi USB port
 * 
 * This setup enables ROS2 communication between ESP32 and Raspberry Pi
 * through a reliable serial connection over USB.
 */

#ifdef __cplusplus
extern "C" 
{
#endif

/**
 * @brief Open USB serial transport
 * @param transport Pointer to the custom transport structure
 * @return true if successful, false otherwise
 */
bool usb_serial_open(struct uxrCustomTransport* transport);

/**
 * @brief Close USB serial transport
 * @param transport Pointer to the custom transport structure
 * @return true if successful, false otherwise
 */
bool usb_serial_close(struct uxrCustomTransport* transport);

/**
 * @brief Write data to USB serial transport
 * @param transport Pointer to the custom transport structure
 * @param buf Buffer containing data to write
 * @param len Length of data to write
 * @param err Pointer to error flag
 * @return Number of bytes written
 */
size_t usb_serial_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* err);

/**
 * @brief Read data from USB serial transport
 * @param transport Pointer to the custom transport structure
 * @param buf Buffer to store read data
 * @param len Maximum length to read
 * @param timeout Timeout in milliseconds
 * @param err Pointer to error flag
 * @return Number of bytes read
 */
size_t usb_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif // USB_SERIAL_TRANSPORT_H
