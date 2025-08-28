#include "usb_serial_transport.h"
#include <uxr/client/transport.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_chip_info.h>

static const char *TAG = "usb_serial_transport";

#define USB_SERIAL_UART_NUM     UART_NUM_2    // Use UART2 to avoid conflict with console
#define USB_SERIAL_TX_BUF_QUEUE (2 * 1024)    // TX ring – 2 KB
#define USB_SERIAL_RX_BUF_DMA   (6 * 1024)    // RX ring – 6 KB
#define USB_SERIAL_BAUD_RATE    921600

// UART pins for external USB-to-serial converter connected to Raspberry Pi
#define USB_SERIAL_TXD_PIN      (GPIO_NUM_22)  // TX pin for UART2
#define USB_SERIAL_RXD_PIN      (GPIO_NUM_23)  // RX pin for UART2
#ifndef UART_PIN_NO_CHANGE
#define UART_PIN_NO_CHANGE (-1)
#endif
#define USB_SERIAL_RTS_PIN (UART_PIN_NO_CHANGE)
#define USB_SERIAL_CTS_PIN (UART_PIN_NO_CHANGE)

bool usb_serial_open(struct uxrCustomTransport* transport) {
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = USB_SERIAL_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_install(USB_SERIAL_UART_NUM, USB_SERIAL_RX_BUF_DMA, USB_SERIAL_TX_BUF_QUEUE, 0, NULL, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_param_config(USB_SERIAL_UART_NUM, &uart_config));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_pin(USB_SERIAL_UART_NUM, USB_SERIAL_TXD_PIN, USB_SERIAL_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%d up @%d baud  TX=%d  RX=%d  RTS=%d  CTS=%d", USB_SERIAL_UART_NUM, USB_SERIAL_BAUD_RATE, USB_SERIAL_TXD_PIN, USB_SERIAL_RXD_PIN, USB_SERIAL_RTS_PIN, USB_SERIAL_CTS_PIN);
    return true;
}

bool usb_serial_close(struct uxrCustomTransport* transport) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_delete(USB_SERIAL_UART_NUM));
    ESP_LOGI(TAG, "USB serial transport closed successfully");
    return true;
}

size_t usb_serial_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* err)
{
    if (!buf || !len) { if (err) *err = 1; return 0; }
    int sent = uart_write_bytes(USB_SERIAL_UART_NUM, (const char *)buf, len);
    if (sent < 0) { if (err) *err = 1; return 0; }
    if (err) *err = 0;
    return (size_t)sent;
}

size_t usb_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err)
{
    if (!buf || !len) { if (err) *err = 1; return 0; }
    TickType_t to = timeout ? pdMS_TO_TICKS(timeout) : 0;
    int r = uart_read_bytes(USB_SERIAL_UART_NUM, buf, len, to);
    if (r < 0) { if (err) *err = 1; return 0; }
    if (err) *err = 0;
    return (size_t)r;
}

void usb_serial_flush(void) {
    uart_flush_input(USB_SERIAL_UART_NUM);
    ESP_LOGI(TAG, "UART flush complete");
}
