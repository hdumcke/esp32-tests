/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

#include "mini_pupper_host.h"
#include "mini_pupper_servos.h"
#include "driver/uart.h"
#include "esp_log.h"

#define HOST_SERVER_TXD 17
#define HOST_SERVER_RXD 18
#define HOST_SERVER_RTS (UART_PIN_NO_CHANGE)
#define HOST_SERVER_CTS (UART_PIN_NO_CHANGE)

void HOST_TASK(void * parameters);

HOST host;

HOST::HOST() : 
_uart_port_num(2),
_task_handle(NULL)
{
    // set UART port
    uart_config_t uart_config;
    uart_config.baud_rate = 3000000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(_uart_port_num, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(_uart_port_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(_uart_port_num, HOST_SERVER_TXD, HOST_SERVER_RXD, HOST_SERVER_RTS, HOST_SERVER_CTS));

    xTaskCreate(
        HOST_TASK,                 /* Function that implements the task. */
        "HOST BUS SERVICE",        /* Text name for the task. */
        10000,                      /* Stack size in words, not bytes. */
        (void*)this,                /* Parameter passed into the task. */
        tskIDLE_PRIORITY,           /* Priority at which the task is created. */
        &_task_handle                /* Used to pass out the created task's handle. */
    );
}

void HOST_TASK(void * parameters)
{
    HOST * host = reinterpret_cast<HOST*>(parameters);
    for(;;)
    {
        // delay 1ms
        // - about 1KHz refresh frequency for sync write servo setpoints
        // - about 80Hz refresh frequency for read/ack servo feedbacks
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // wait for a frame from host
        size_t static rx_buffer_size {128};
        u8 rx_buffer[rx_buffer_size] {0};
        // copy RX fifo into local buffer (4 bytes : Header + ID + Length)
        int const read_length {uart_read_bytes(host->_uart_port_num,rx_buffer,rx_buffer_size,2)};

        // waiting for a header...
        if(read_length != 4) 
        {
            // flush RX FIFO
            uart_flush(host->_uart_port_num);    
            // next
            continue;
        }

        // waiting for a valid frame header with my ID...
        bool const rx_header_check { 
                    (rx_buffer[0]==0xFF) 
                &&  (rx_buffer[1]==0xFF) 
                &&  (rx_buffer[2]==0x01) // my ID
                &&  (rx_buffer[3]<64)
        };  
        if(!rx_header_check) 
        {
            // flush RX FIFO
            uart_flush(host->_uart_port_num);    
            // next
            continue;
        }

        // read payalod length from frame header
        size_t const rx_payload_length {(size_t)rx_buffer[3]};






        // flush RX FIFO
        uart_flush(host->_uart_port_num);    
    }    
}