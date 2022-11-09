#include <stddef.h>
#include "uart_server.h"
#include "protocolfunctions.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define UART_SERVER_TXD 17
#define UART_SERVER_RXD 18
#define UART_SERVER_RTS (UART_PIN_NO_CHANGE)
#define UART_SERVER_CTS (UART_PIN_NO_CHANGE)

#define UART_SERVER_PORT_NUM      2
#define UART_SERVER_BAUD_RATE     3000000 // seams to be the limit
#define UART_SERVER_TASK_STACK_SIZE    4096

static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)

static int send_serial_data( unsigned char *data, int len ) {
    uart_write_bytes(UART_SERVER_PORT_NUM, data, len);
    return len;
}

static void server_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_SERVER_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	.rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_SERVER_PORT_NUM, 2*1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config( UART_SERVER_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin( UART_SERVER_PORT_NUM, UART_SERVER_TXD, UART_SERVER_RXD, UART_SERVER_RTS, UART_SERVER_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    setup_protocol(&sUSART2);
    sUSART2.send_serial_data = send_serial_data;

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes( UART_SERVER_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len) {
            //ESP_LOGI(TAG, "Recv str len: %d", len);
	    //ESP_LOG_BUFFER_HEX(TAG, data, len);
            for(int i = 0; i<len; i++) {
		protocol_byte( &sUSART2, (unsigned char) data[i]);
            }
             
            data[len] = '\0';
        }
        protocol_tick( &sUSART2 );
    }
}

UARTServer::UARTServer()
{
    xTaskCreate(server_task, "uart_server_task", UART_SERVER_TASK_STACK_SIZE, NULL, 10, NULL);
}
