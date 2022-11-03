#include "SCSCL.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


class SERVO : public SCSCL
{
public:
    void TestBroadcast() {
        int i;
        this->EnableTorque(0xfe,1);
        vTaskDelay(500 / portTICK_RATE_MS);
        this->WritePos(0xfe, 0, 1000);
        vTaskDelay(1000 / portTICK_RATE_MS);
   
        while(1) {	
          printf("Start of Cycle!\n");
          for(i = 0; i<1024; i++)
          {
            this->WritePos(0xfe,i,20);
            printf("Position: %d\n", i);
            vTaskDelay(20 / portTICK_RATE_MS);
          }
          for(i = 1023; i > 0; i--)
          {
            this->WritePos(0xfe,i,10);
            printf("Position: %d\n", i);
            vTaskDelay(10 / portTICK_RATE_MS);
          }
        }
    }
};

extern "C" void app_main(void)
{

    // enable servos
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;//disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;//set as output mode
    io_conf.pin_bit_mask = (1ULL<<8);//bit mask of the pins that you want to set,e.g.GPIO18
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;//disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;//disable pull-up mode
    ESP_ERROR_CHECK(gpio_config(&io_conf));//configure GPIO with the given settings
    gpio_set_level(GPIO_NUM_8, 1);

    // set UART port
    uart_config_t uart_config;
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;
#if SOC_UART_SUPPORT_REF_TICK
    uart_config.source_clk = UART_SCLK_REF_TICK;
#elif SOC_UART_SUPPORT_XTAL_CLK
    uart_config.source_clk = UART_SCLK_XTAL;
#endif
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 2*1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    SERVO servo;
    servo.uart_port_num = UART_NUM_2;

    servo.TestBroadcast();

}
