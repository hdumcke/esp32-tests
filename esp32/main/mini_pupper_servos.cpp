#include "mini_pupper_servos.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "MINIPUPPERSERVOS";

// number of retries for servo functions
int retries = 3;

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
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2*1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    this->uart_port_num = UART_NUM_1;
    this->disable();
    this->disableTorque();
    write_type = swt_gen;
}

void SERVO::disable() {
    gpio_set_level(GPIO_NUM_8, 0);
    isEnabled = false;
}

void SERVO::enable() {
    gpio_set_level(GPIO_NUM_8, 1);
    isEnabled = true;
}

void SERVO::enableTorque() {
    this->EnableTorque(0xFE, 1);
    isTorqueEnabled = true;
}

void SERVO::disableTorque() {
    this->EnableTorque(0xFE, 0);
    isTorqueEnabled = false;
}

void SERVO::rotate(u8 servoID) {
    int i;
    setPosition(servoID, 0);
   
    for(int l=0; l<5; l++) {	
      printf("Start of Cycle!\n");
      for(i = 0; i<1024; i++)
      {
        setPosition(servoID,i);
        printf("Position: %d\n", i);
      }
      for(i = 1023; i > 0; i--)
      {
        setPosition(servoID,i);
        printf("Position: %d\n", i);
      }
    }
}

void SERVO::setStartPos(u8 servoID) {
    setPosition(servoID, 0);
}

void SERVO::setMidPos(u8 servoID) {
    setPosition(servoID, 511);
}

void SERVO::setEndPos(u8 servoID) {
    setPosition(servoID, 1023);
}

int SERVO::setPosition(u8 servoID, u16 position, u16 speed) {
    int retry_counter = retries;

    for( int i=0; i<retry_counter; i++) {
        switch(write_type) {
            case swt_gen:
                this->WritePos(servoID, position, speed);
		break;
            case swt_reg:
                this->RegWritePos(servoID, position, speed);
		break;
            default:
		break;
	}
	if(!this->Err) { 
	    return i;
        }
	else {
            ESP_LOGI(TAG, "Retrying WritePos((%d)", servoID);
            vTaskDelay(20 / portTICK_PERIOD_MS);
	}
    }

    return 999; //too many retries
}

bool SERVO::checkPosition(u8 servoID, u16 position, int accuracy = 5) {
    int pos = 0;
    bool ret = false;
    int retry_counter = retries;
    
    for( int i=0; i<retry_counter; i++) {
        pos = this->ReadPos(servoID);
	if(!this->Err) { 
	    retry_counter = 0;
        }
	else {
            ESP_LOGI(TAG, "Retrying ReadPos(%d)", servoID);
            vTaskDelay(20 / portTICK_PERIOD_MS);
	}
    }
    ret = (pos>=(position-accuracy) && pos<=(position+accuracy));
    //ESP_LOGI(TAG, "Position: %d FeedBack %d Servo: %d %d", pos, this->FeedBack(servoID), servoID, ret);
    //ESP_LOGI(TAG, "Position: %d Speed %d Servo: %d %d", pos, this->ReadSpeed(servoID), servoID, ret);
    //ESP_LOGI(TAG, "Position: %d Load %d Servo: %d %d", pos, this->ReadLoad(servoID), servoID, ret);
    //ESP_LOGI(TAG, "Position: %d Voltage %d Servo: %d %d", pos, this->ReadSpeed(servoID), servoID, ret);
    //ESP_LOGI(TAG, "Position: %d Temper %d Servo: %d %d", pos, this->ReadSpeed(servoID), servoID, ret);
    //ESP_LOGI(TAG, "Position: %d Move %d Servo: %d %d", pos, this->ReadSpeed(servoID), servoID, ret);
    //ESP_LOGI(TAG, "Position: %d Current %d Servo: %d %d", pos, this->ReadSpeed(servoID), servoID, ret);
    return ret;
}

void SERVO::setID(u8 servoID, u8 newID) {
	unLockEprom(servoID);
	writeByte(servoID, SCSCL_ID, newID);
	LockEprom(newID);
}
