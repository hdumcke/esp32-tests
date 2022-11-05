#include "mini_pupper_servos.hpp"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

SERVO::SERVO() {
    // setup enable pin
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;//disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;//set as output mode
    io_conf.pin_bit_mask = (1ULL<<8);//bit mask of the pins that you want to set,e.g.GPIO18
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;//disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;//disable pull-up mode
    ESP_ERROR_CHECK(gpio_config(&io_conf));//configure GPIO with the given settings
    // set UART port
    uart_config_t uart_config;
    uart_config.baud_rate = 500000;
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
    this->uart_port_num = UART_NUM_2;
    this->disable();
}

void SERVO::disable() {
    gpio_set_level(GPIO_NUM_8, 0);
    isEnabled = false;
}

void SERVO::enable() {
    gpio_set_level(GPIO_NUM_8, 1);
    isEnabled = true;
}

void SERVO::rotate(u8 servoID) {
    int i;
    this->EnableTorque(servoID,1);
    vTaskDelay(500 / portTICK_RATE_MS);
    this->WritePos(servoID, 0, 1000);
    vTaskDelay(1000 / portTICK_RATE_MS);
   
    while(1) {	
      printf("Start of Cycle!\n");
      for(i = 0; i<1024; i++)
      {
        this->WritePos(servoID,i,20);
        printf("Position: %d\n", i);
        vTaskDelay(20 / portTICK_RATE_MS);
      }
      for(i = 1023; i > 0; i--)
      {
        this->WritePos(servoID,i,10);
        printf("Position: %d\n", i);
        vTaskDelay(10 / portTICK_RATE_MS);
      }
    }
}

void SERVO::setStartPos(u8 servoID) {
    this->WritePos(servoID, 0, 10);
}

void SERVO::setMidPos(u8 servoID) {
    this->WritePos(servoID, 512, 10);
}

void SERVO::setEndPos(u8 servoID) {
    this->WritePos(servoID, 1023, 10);
}

void SERVO::setPosition(u8 servoID, u16 position) {
    this->WritePos(servoID, position, 10);
}

void SERVO::setID(u8 servoID, u8 newID) {
	unLockEprom(servoID);
	writeByte(servoID, SCSCL_ID, newID);
	LockEprom(newID);
}
