/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

#include "mini_pupper_host.h"
#include "mini_pupper_servos.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

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
}

void HOST::start()
{
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
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // wait for a frame from host
        size_t static rx_buffer_size {128};
        u8 rx_buffer[rx_buffer_size] {0};
        // copy RX fifo into local buffer (4 bytes : Header + ID + Length)
        int read_length {uart_read_bytes(host->_uart_port_num,rx_buffer,4,0)}; // timeout = 0

        // trace
        //if(read_length>0)
        //    printf(" (1) read_length:%d. ",read_length);

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
                &&  (rx_buffer[3]<=(rx_buffer_size-4)) // keep the header in the rx buffer
        };  
        if(!rx_header_check) 
        {
            // flush RX FIFO
            uart_flush(host->_uart_port_num);    
            // next
            continue;
        }

        //printf(" (2) header valid! ");

        // read payalod length from frame header
        size_t const rx_payload_length {(size_t)rx_buffer[3]};

        // copy RX fifo into local buffer (L bytes : Payload + Checksum)
        read_length = uart_read_bytes(host->_uart_port_num,rx_buffer+4,rx_payload_length,2);
        //printf(" (3) read_length:%d. ",read_length);

        // waiting for a (full) payload...
        if(read_length != rx_payload_length) 
        {
            // flush RX FIFO
            uart_flush(host->_uart_port_num);    
            // next
            continue;
        }

        // compute checksum
        u8 chk_sum {0};
        for(size_t chk_index=2; chk_index<(rx_payload_length+4-1); ++chk_index)
        {
            chk_sum += rx_buffer[chk_index];
        }   
        bool const rx_checksum_check { rx_buffer[rx_payload_length+4-1] == (u8)(~chk_sum) };
        //printf(" (4.0) checksum recv:%d comp:%d. ",(int)rx_buffer[rx_payload_length+4-1],(int)(u8)(~chk_sum));

        // waiting for a valid instruction and checksum...
        bool const rx_payload_checksum_check { 
                    (rx_buffer[4]==INST_CONTROL) 
                &&  rx_checksum_check
        };  
        if(!rx_payload_checksum_check) 
        {
            // flush RX FIFO
            uart_flush(host->_uart_port_num);    
            // next
            continue;
        }

        //printf(" (4) payload and checksum valid! ");

        // decode parameters
        parameters_control_instruction_format data {0};
        memcpy(&data,rx_buffer+5,sizeof(parameters_control_instruction_format));

        //printf(" (5) goal_position[0]:%d. ", data.goal_position[0]);

        servo.enable();
        servo.setPosition12Async(data.goal_position);

        // flush RX FIFO
        uart_flush(host->_uart_port_num);    
    }    
}