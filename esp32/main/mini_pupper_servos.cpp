#include "mini_pupper_servos.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "esp_log.h"

// reference :
//https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html

//static const char *TAG = "MINIPUPPERSERVOS";

void SERVO_TASK(void * parameters);

SERVO servo;

SERVO::SERVO() {
    // setup enable pin
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;//disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;//set as output mode
    io_conf.pin_bit_mask = (1ULL<<8);//bit mask of the pins that you want to set,e.g.GPIO18
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;//disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;//disable pull-up mode
    ESP_ERROR_CHECK(gpio_config(&io_conf));//configure GPIO with the given settings

    // power off servo system
    disable();

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
    uart_port_num = UART_NUM_1;
    ESP_ERROR_CHECK(uart_driver_install(uart_port_num, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_port_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_port_num, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /*** ASYNC API service ***/

    xTaskCreate(
        SERVO_TASK,                 /* Function that implements the task. */
        "SERVO BUS SERVICE",        /* Text name for the task. */
        10000,                      /* Stack size in words, not bytes. */
        (void*)this,                /* Parameter passed into the task. */
        tskIDLE_PRIORITY,           /* Priority at which the task is created. */
        &task_handle                /* Used to pass out the created task's handle. */
    );

    /*** ASYNC API service ***/
}

void SERVO::disable() {
    gpio_set_level(GPIO_NUM_8, 0);
    isEnabled = false;
    isTorqueEnabled = false;
    isSyncRunning = false;
}

void SERVO::enable() {
    gpio_set_level(GPIO_NUM_8, 1);
    isEnabled = true;
    isTorqueEnabled = false;
    isSyncRunning = false;
}

int SERVO::ping(u8 ID)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // abort command when broadcasting
    if(ID==0XFE) return SERVO_STATUS_FAIL;

    // send ping instruction
    write_frame(ID,INST_PING,nullptr,0);

    // wait for reply
    u8 reply_id {0};
    u8 reply_state {0};
    int const status = reply_frame(reply_id,reply_state,nullptr,0);
    ///printf(" reply_id:%d reply_state:%d status:%d.",reply_id,reply_state,status);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // check reply
    if(reply_id!=ID || reply_state!=0) return SERVO_STATUS_FAIL;

    return SERVO_STATUS_OK;
}

int SERVO::recovery(u8 ID)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // abort command when broadcasting
    if(ID==0XFE) return SERVO_STATUS_FAIL;

    // send ping instruction
    write_frame(ID,INST_RECOVERY,nullptr,0);

    // wait for reply
    u8 reply_id {0};
    u8 reply_state {0};
    int const status = reply_frame(reply_id,reply_state,nullptr,0);
    ///printf(" reply_id:%d reply_state:%d status:%d.",reply_id,reply_state,status);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // check reply
    if(reply_id!=ID || reply_state!=0) return SERVO_STATUS_FAIL;

    return SERVO_STATUS_OK;
}

int SERVO::enable_torque(u8 ID)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // write instruction
    write_register_byte(ID, SERVO_TORQUE_ENABLE, 1);

    // if broadcast, do not wait for reply
    if(ID==0XFE)
    {
        isTorqueEnabled = true;
        return SERVO_STATUS_OK;    
    }

    // wait for reply
    u8 reply_id {0};
    u8 reply_state {0};
    int const status = reply_frame(reply_id,reply_state,nullptr,0);
    ///printf(" reply_id:%d reply_state:%d status:%d.",reply_id,reply_state,status);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // check reply
    if(reply_id!=ID || reply_state!=0) return SERVO_STATUS_FAIL;

    isTorqueEnabled = true;
    return SERVO_STATUS_OK;    
}

int SERVO::disable_torque(u8 ID)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // send write frame
    write_register_byte(ID, SERVO_TORQUE_ENABLE, 0);
    
    // if broadcast, do not wait for reply
    if(ID==0XFE)
    {
        isTorqueEnabled = true;
        return SERVO_STATUS_OK;    
    }

    // wait for reply
    u8 reply_id {0};
    u8 reply_state {0};
    int const status = reply_frame(reply_id,reply_state,nullptr,0);
    ///printf(" reply_id:%d reply_state:%d status:%d.",reply_id,reply_state,status);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // check reply
    if(reply_id!=ID || reply_state!=0) return SERVO_STATUS_FAIL;

    isTorqueEnabled = false;
    return SERVO_STATUS_OK; 
}

int SERVO::set_position(u8 ID, u16 position)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // abort command when broadcasting
    if(ID==0XFE) return SERVO_STATUS_FAIL;

    // send write instruction
    write_register_word(ID, SERVO_GOAL_POSITION_L, position);

    // wait for reply
    u8 reply_id {0};
    u8 reply_state {0};
    int const status = reply_frame(reply_id,reply_state,nullptr,0);
    ///printf(" reply_id:%d reply_state:%d status:%d.",reply_id,reply_state,status);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // check reply
    if(reply_id!=ID || reply_state!=0) return SERVO_STATUS_FAIL;

    return SERVO_STATUS_OK;
}

int SERVO::get_position(u8 ID, u16 & position)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // send read instruction
    int const status = read_register_word(ID, SERVO_PRESENT_POSITION_L,position);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    return SERVO_STATUS_OK;
}

int SERVO::get_velocity(u8 ID, s16 & velocity)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // send read instruction
    u16 present_value {0};
    int const status = read_register_word(ID, SERVO_PRESENT_SPEED_L,present_value);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // make signed
    velocity = (s16)present_value;
    if(velocity&(1<<15))
        velocity = -(velocity&~(1<<15));

    return SERVO_STATUS_OK;
}

int SERVO::get_load(u8 ID, s16 & load)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // send read instruction
    u16 present_value {0};
    int const status = read_register_word(ID, SERVO_PRESENT_LOAD_L,present_value);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // make signed
    load = (s16)present_value;
    if(load&(1<<10))
        load = -(load&~(1<<10));

    return SERVO_STATUS_OK;
}

int SERVO::get_voltage(u8 ID, u8 & voltage)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // send read instruction
    u8 present_value {0};
    int const status = read_register_byte(ID, SERVO_PRESENT_VOLTAGE,present_value);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    voltage = present_value;

    return SERVO_STATUS_OK;
}

int SERVO::get_temperature(u8 ID, u8 & temperature)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // send read instruction
    u8 present_value {0};
    int const status = read_register_byte(ID, SERVO_PRESENT_TEMPERATURE,present_value);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    temperature = present_value;

    return SERVO_STATUS_OK;
}

int SERVO::get_move(u8 ID, u8 & move)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // send read instruction
    u8 present_value {0};
    int const status = read_register_byte(ID, SERVO_MOVING,present_value);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    move = present_value;

    return SERVO_STATUS_OK;
}

int SERVO::get_current(u8 ID, s16 & current)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // send read instruction
    u16 present_value {0};
    int const status = read_register_word(ID, SERVO_PRESENT_CURRENT_L,present_value);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // make signed
    current = (s16)present_value;
    if(current&(1<<15))
        current = -(current&~(1<<15));

    return SERVO_STATUS_OK;
}

int SERVO::unlock_eeprom(u8 ID)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // write instruction
    write_register_byte(ID, SERVO_LOCK, 0);

    // wait for reply
    u8 reply_id {0};
    u8 reply_state {0};
    int const status = reply_frame(reply_id,reply_state,nullptr,0);
    ///printf(" reply_id:%d reply_state:%d status:%d.",reply_id,reply_state,status);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // check reply
    if(reply_id!=ID || reply_state!=0) return SERVO_STATUS_FAIL;

    return SERVO_STATUS_OK;  
}

int SERVO::lock_eeprom(u8 ID)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    // write instruction
    write_register_byte(ID, SERVO_LOCK, 1);

    // wait for reply
    u8 reply_id {0};
    u8 reply_state {0};
    int const status = reply_frame(reply_id,reply_state,nullptr,0);
    ///printf(" reply_id:%d reply_state:%d status:%d.",reply_id,reply_state,status);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // check reply
    if(reply_id!=ID || reply_state!=0) return SERVO_STATUS_FAIL;

    return SERVO_STATUS_OK; 
}

void SERVO::set_position_all(u16 const servoPositions[])
{
    // suspend sync service
    enableAsyncService(false);

    static size_t const L {2};                      // Length of data sent to each servo
    static size_t const N {12};                     // Servo Number
    static size_t const Length {(L+1)*N+4};         // Length field value
    static size_t const buffer_size {2+1+1+Length}; // 0xFF 0xFF ID LENGTH (INSTR PARAM... CHK)
    // prepare frame header and parameters (fixed)
    static u8 buffer[buffer_size] {
        0xFF,                                       // Start of Frame
        0xFF,                                       // Start of Frame
        0xFE,                                       // ID
        Length,                                     // Length
        INST_SYNC_WRITE,                            // Instruction
        SERVO_GOAL_POSITION_L,                      // Parameter 1 : Register address
        L                                           // Parameter 2 : L
    };
    // build frame payload
    size_t index {7};
    static u8 const servoIDs[] {1,2,3,4,5,6,7,8,9,10,11,12};
    for(size_t servo_index=0; servo_index<N; ++servo_index) {
        buffer[index++] = servoIDs[servo_index];            // Parameter 3 = Servo Number
        buffer[index++] = (servoPositions[servo_index]>>8); // Write the first data of the first servo
        buffer[index++] = (servoPositions[servo_index]&0xff);
    }
    // compute checksum
    u8 chk_sum {0};
    for(size_t chk_index=2; chk_index<(buffer_size-1); ++chk_index) {
        chk_sum += buffer[chk_index];
    }
    buffer[index++] = ~chk_sum;
    // send frame to uart
    uart_write_bytes(uart_port_num, buffer, buffer_size);
}


int SERVO::setID(u8 servoID, u8 newID)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    unlock_eeprom(servoID);
    write_register_byte(servoID, SERVO_ID, newID);
    lock_eeprom(newID);

    return SERVO_STATUS_OK;
}

int SERVO::rotate(u8 servoID)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);

    int i {0};
    set_position(servoID, 0);
   
    for(int l=0; l<5; l++) {    
      printf("Start of Cycle!\n");
      for(i = 0; i<1024; i++)
      {
        set_position(servoID,i);
        printf("Position: %d\n", i);
      }
      for(i = 1023; i > 0; i--)
      {
        set_position(servoID,i);
        printf("Position: %d\n", i);
      }
    }

    return SERVO_STATUS_OK;
}

int SERVO::setStartPos(u8 servoID)
{
    return set_position(servoID, 0);
}

int SERVO::setMidPos(u8 servoID)
{
    return set_position(servoID, 511);
}

int SERVO::setEndPos(u8 servoID)
{
    return set_position(servoID, 1023);
}

bool SERVO::checkPosition(u8 servoID, u16 position, int accuracy = 5)
{
    // abort if servo not powered on
    if(!isEnabled) return SERVO_STATUS_FAIL;

    // suspend sync service
    enableAsyncService(false);


/*
    u16 pos = 0;
    bool ret = false;
    int retry_counter = retries;
    
    for( int i=0; i<retry_counter; i++) {
        get_position(servoID,pos);
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
    */
    return false;
}

void SERVO::setPositionAsync(u8 servoID, u16 servoPosition)
{
    if(0<servoID && servoID<=12)
        state[servoID-1].goal_position=servoPosition;
    
    // (re)start sync service
    enableAsyncService(true);
}

void SERVO::setPosition12Async(u16 const servoPositions[])
{
    for(size_t index=0;index<12;++index)
        state[index].goal_position = servoPositions[index];
    
    // (re)start sync service
    enableAsyncService(true);
}

u16  SERVO::getPositionAsync(u8 servoID)
{
    // (re)start sync service
    enableAsyncService(true);

    if(0<servoID && servoID<=12)
        return state[servoID-1].present_position;
    else
        return 0;
}

s16  SERVO::getVelocityAsync(u8 servoID)
{
    // (re)start sync service
    enableAsyncService(true);

    if(0<servoID && servoID<=12)
        return state[servoID-1].present_velocity;
    else
        return 0;
}

s16  SERVO::getLoadAsync(u8 servoID)
{
    // (re)start sync service
    enableAsyncService(true);

    if(0<servoID && servoID<=12)
        return state[servoID-1].present_load;
    else
        return 0;
}

u8  SERVO::getVoltageAsync(u8 servoID)
{
    // (re)start sync service
    enableAsyncService(true);

    if(0<servoID && servoID<=12)
        return state[servoID-1].present_load;
    else
        return 0;
}

u8  SERVO::getTemperatureAsync(u8 servoID)
{
    // (re)start sync service
    enableAsyncService(true);

    if(0<servoID && servoID<=12)
        return state[servoID-1].present_temperature;
    else
        return 0;
}

u8  SERVO::getMoveAsync(u8 servoID)
{
    // (re)start sync service
    enableAsyncService(true);

    if(0<servoID && servoID<=12)
        return state[servoID-1].present_move;
    else
        return 0;
}

s16 SERVO::getCurrentAsync(u8 servoID)
{
    // (re)start sync service
    enableAsyncService(true);

    if(0<servoID && servoID<=12)
        return state[servoID-1].present_current;
    else
        return 0;
}

void SERVO::enableAsyncService(bool enable)
{
    // cannot start is power off
    if(!isEnabled) return;

    // nothing to do
    if(isSyncRunning==enable) return;

    // (re)start sync service
    if(enable)
    {
        // (re)start sync task and wait a moment to synchronise local feedback data base
        isSyncRunning = true; 
        vTaskDelay(20 / portTICK_PERIOD_MS);   
    }
    // suspend sync service
    else
    {
        // suspend sync task and wait a moment
        isSyncRunning = false;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    // flush RX FIFO
    uart_flush(uart_port_num);    
}

void SERVO::sync_all_goal_position()
{
    // prepare sync write frame to all servo
    static size_t const L {2};                      // Length of data sent to each servo
    static size_t const N {12};                     // Servo Number
    static size_t const Length {(L+1)*N+4};         // Length field value
    static size_t const buffer_size {2+1+1+Length}; // 0xFF 0xFF ID LENGTH (INSTR PARAM... CHK)
    // prepare frame header and parameters (fixed)
    static u8 buffer[buffer_size] {
        0xFF,                                       // Start of Frame
        0xFF,                                       // Start of Frame
        0xFE,                                       // ID
        Length,                                     // Length
        INST_SYNC_WRITE,                            // Instruction
        SERVO_GOAL_POSITION_L,                      // Parameter 1 : Register address
        L                                           // Parameter 2 : L
    };
    // build frame payload
    size_t index {7};
    for(size_t servo_index=0; servo_index<N; ++servo_index) {
        buffer[index++] = state[servo_index].ID;                    // Parameter 3 = Servo Number
        buffer[index++] = (state[servo_index].goal_position>>8);    // Write the first data of the first servo
        buffer[index++] = (state[servo_index].goal_position&0xff);
    }
    // compute checksum
    u8 chk_sum {0};
    for(size_t chk_index=2; chk_index<(buffer_size-1); ++chk_index) {
        chk_sum += buffer[chk_index];
    }
    buffer[index++] = ~chk_sum;
    // send frame to all servo    
    uart_write_bytes(uart_port_num,buffer,buffer_size);
}

void SERVO::cmd_feedback_one_servo(SERVO_STATE & servoState)
{
    // prepare read frame to one servo
    static size_t const buffer_size {8};            
    u8 buffer[buffer_size] {
        0xFF,                                       // Start of Frame
        0xFF,                                       // Start of Frame
        servoState.ID,                              // ID
        0x04,                                       // Length of read instruction
        INST_READ,                                  // Read instruction
        SERVO_PRESENT_POSITION_L,                   // Parameter 1 : Register address
        0x06,                                       // Parameter 2 : 6 bytes (position, speed, load)
        0x00                                        // checksum
    };
    // compute checksum
    u8 chk_sum {0};
    for(size_t chk_index=2; chk_index<(buffer_size-1); ++chk_index) {
        chk_sum += buffer[chk_index];
    }
    buffer[buffer_size-1] = ~chk_sum;
    // send frame to servo
    uart_write_bytes(uart_port_num,buffer,buffer_size);
    // flush RX FIFO
    uart_flush(uart_port_num);
}

void SERVO::ack_feedback_one_servo(SERVO_STATE & servoState)
{
    // a buffer to process read ack from one servo
    static size_t const buffer_size {12};     
    u8 buffer[buffer_size] {0};
    // copy RX fifo into local buffer
    int const read_length = uart_read_bytes(uart_port_num,buffer,buffer_size,1);
    // check expected frame size
    if(read_length==buffer_size)
    {
        // check frame header, servo id, instruction, length and working condition returned by servo
        if( buffer[0] == 0xFF &&             // Start of Frame
            buffer[1] == 0xFF &&             // Start of Frame
            buffer[2] == servoState.ID &&    // ID
            buffer[3] == 0x08 &&             // Length of read reply
            buffer[4] == 0x00                // Length of working condition
        )
        {
            // compute checksum
            u8 chk_sum {0};
            for(size_t chk_index=2; chk_index<(buffer_size-1); ++chk_index) {
                chk_sum += buffer[chk_index];
            }   
            // check checksum
            if(buffer[buffer_size-1]==(u8)(~chk_sum))
            {
                // decode parameters and update feedback local data base for this servo
                servoState.present_position = (u16)(buffer[5])<<8 | buffer[6];
                servoState.present_velocity = (u16)(buffer[7])<<8 | buffer[8];
                servoState.present_load =     (u16)(buffer[9])<<8 | buffer[10];

                // TODO : MORE DATA
                // TODO : MORE DATA
                // TODO : MORE DATA

                // MORE : MAKE SIGNED
                // MORE : MAKE SIGNED
                // MORE : MAKE SIGNED
            }
        }
    }
    // flush RX FIFO
    uart_flush(uart_port_num); 
}

void SERVO_TASK(void * parameters)
{
    SERVO * servo = reinterpret_cast<SERVO*>(parameters);
    u8 servoID {0};
    for(;;)
    {
        if(servo->isEnabled && servo->isSyncRunning)
        {
            // process read ack from one servo
            servo->ack_feedback_one_servo(servo->state[servoID]);
            // sync write setpoint to all servo
            servo->sync_all_goal_position();
            // basic round robin algorithm for feedback
            servoID = (servoID+1)%12;
            // read one servo feedback
            servo->cmd_feedback_one_servo(servo->state[servoID]);
        }
        // delay 1ms
        // - about 1KHz refresh frequency for sync write servo setpoints
        // - about 80Hz refresh frequency for read/ack servo feedbacks
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

int SERVO::write_frame(u8 ID, u8 instruction, u8 const * parameters, size_t parameter_length)
{
    // prepare frame to one servo
    size_t const length {parameter_length+2};       // Length : instruction + params + checksum
    size_t const buffer_size {2+1+1+length};        // 0xFF 0xFF ID LENGTH (INSTR PARAM... CHK)    
    u8 buffer[buffer_size] {
        0xFF,                                       // Start of Frame
        0xFF,                                       // Start of Frame
        ID,                                         // ID
        (u8)length,                                 // Length of payload
        instruction                                 // Instruction
    };
    // fill payload
    if(parameters)
    {
        for(size_t index=0; index<parameter_length; ++index)
        {
            buffer[index+5]=parameters[index];
        }
    }
    else
    {
        for(size_t index=0; index<parameter_length; ++index)
        {
            buffer[index+5]=0;
        }        
    }
    // compute checksum and fill payload
    u8 chk_sum {0};
    for(size_t chk_index=2; chk_index<(buffer_size-1); ++chk_index)
    {
        chk_sum += buffer[chk_index];
    }
    buffer[buffer_size-1] = ~chk_sum;
    // flush RX FIFO
    uart_flush(uart_port_num);  
    // send frame to servo
    uart_write_bytes(uart_port_num,buffer,buffer_size);

    return SERVO_STATUS_OK;
}

int SERVO::reply_frame(u8 & ID, u8 & state, u8 * parameters, size_t parameter_length)
{
    // a buffer to process reply packet from one servo
    // prepare frame to one servo
    size_t const length {parameter_length+2};       // Length : state + params + checksum
    size_t const buffer_size {2+1+1+length};        // 0xFF 0xFF ID LENGTH (STATE PARAM... CHK)    
    u8 buffer[buffer_size] {0};
    // copy RX fifo into local buffer
    int const read_length = uart_read_bytes(uart_port_num,buffer,buffer_size,2);
    // flush RX FIFO
    uart_flush(uart_port_num);    
    // check expected frame size
    ///printf("   buffer_size:%d read_length:%d.",buffer_size,read_length);
    if(read_length!=buffer_size) return SERVO_STATUS_FAIL;
    // check frame header
    if(buffer[0]!=0xFF || buffer[1]!=0xFF) return SERVO_STATUS_FAIL;
    // check length
    if(buffer[3]!=length) return SERVO_STATUS_FAIL;
    // compute checksum
    u8 chk_sum {0};
    for(size_t chk_index=2; chk_index<(buffer_size-1); ++chk_index)
    {
        chk_sum += buffer[chk_index];
    }   
    // check checksum
    if(buffer[buffer_size-1]!=(u8)(~chk_sum)) return SERVO_STATUS_FAIL;
    // extract servo ID
    ID = buffer[2];
    // extract state
    state = buffer[4];
    // extract parameters
    if(parameters)
    {
        for(size_t index=0; index<parameter_length; ++index)
        {
            parameters[index]=buffer[index+5];
        }
    }
    return SERVO_STATUS_OK;   
}

int SERVO::write_register_byte(u8 id, u8 reg, u8 value)
{
    u8 const buffer[2] {reg,value};
    write_frame(id,INST_WRITE,buffer,2);

    return SERVO_STATUS_OK;
}

int SERVO::write_register_word(u8 id, u8 reg, u16 value)
{
    u8 const buffer[3] {
        reg,
        (u8)(value>>8),
        (u8)(value&0xff)
    };
    write_frame(id,INST_WRITE,buffer,3);

    return SERVO_STATUS_OK;
}

int SERVO::read_register_byte(u8 id, u8 reg, u8 & value)
{
    // abort command when broadcasting
    if(id==0XFE) return SERVO_STATUS_FAIL;

    // send read instruction
    u8 const buffer[2] {reg,1};
    write_frame(id,INST_READ,buffer,2);

    // wait for reply
    u8 reply_id {0};
    u8 reply_state {0};
    u8 data[1] {0};
    int const status = reply_frame(reply_id,reply_state,data,1);
    ///printf(" reply_id:%d reply_state:%d status:%d.",reply_id,reply_state,status);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // check reply
    if(reply_id!=id || reply_state!=0) return SERVO_STATUS_FAIL;

    // make byte
    value = data[0];

    return SERVO_STATUS_OK;
}

int SERVO::read_register_word(u8 id, u8 reg, u16 & value)
{
    // abort command when broadcasting
    if(id==0XFE) return SERVO_STATUS_FAIL;

    // send read instruction
    u8 const buffer[2] {reg,2};
    write_frame(id,INST_READ,buffer,2);

    // wait for reply
    u8 reply_id {0};
    u8 reply_state {0};
    u8 data[2] {0};
    int const status = reply_frame(reply_id,reply_state,data,2);
    ///printf(" reply_id:%d reply_state:%d status:%d.",reply_id,reply_state,status);

    // check reply
    if(status!=SERVO_STATUS_OK) return SERVO_STATUS_FAIL;

    // check reply
    if(reply_id!=id || reply_state!=0) return SERVO_STATUS_FAIL;

    // make word
    value = (u16)(data[0])<<8 | data[1];

    return SERVO_STATUS_OK;
}

