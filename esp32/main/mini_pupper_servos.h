#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// MENUCONFIG > COMPONENTS > FREERTOS > KERNEL > 1000Hz
// MENUCONFIG > COMPONENTS > FREERTOS > KERNEL > 1000Hz
// MENUCONFIG > COMPONENTS > FREERTOS > KERNEL > 1000Hz

#ifndef _mini_pupper_servos_H
#define _mini_pupper_servos_H

typedef char s8;
typedef unsigned char u8;   
typedef unsigned short u16; 
typedef short s16;
typedef unsigned long u32;  
typedef long s32;

// servo instruction code
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83
#define INST_RECOVERY 0x06

// servo data rate (register value)
#define _1M 0
#define _0_5M 1
#define _250K 2
#define _128K 3
#define _115200 4
#define _76800 5
#define _57600 6
#define _38400 7
#define _19200 8
#define _14400 9
#define _9600 10
#define _4800 11

// servo EEPROM register address (configuration)
#define SCSCL_VERSION_L 3
#define SCSCL_VERSION_H 4
#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

// servo SRAM register address (setpoints)
#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

// servo SRAM register address (feedback)
#define SCSCL_PRESENT_POSITION_L 56
#define SCSCL_PRESENT_POSITION_H 57
#define SCSCL_PRESENT_SPEED_L 58
#define SCSCL_PRESENT_SPEED_H 59
#define SCSCL_PRESENT_LOAD_L 60
#define SCSCL_PRESENT_LOAD_H 61
#define SCSCL_PRESENT_VOLTAGE 62
#define SCSCL_PRESENT_TEMPERATURE 63
#define SCSCL_MOVING 66
#define SCSCL_PRESENT_CURRENT_L 69
#define SCSCL_PRESENT_CURRENT_H 70

struct SERVO_STATE
{
    SERVO_STATE(u8 id) : ID(id) {}

    u8 ID                   {0};
    u8 torque_switch        {0};
    u16 goal_position       {512}; // middle position
    u16 present_position    {0};
    u16 present_velocity    {0};
    u16 present_load        {0};
    // TODO add torque enable 
    // TODO add torque enable 
    // TODO add torque enable 
};

class SERVO
{
public:
    SERVO();

    /* Power distribution API
     *
     */

    void enable();
    void disable();

    /* Sync API
     *
     * Each member function generates one or several instruction/ack frames on servo BUS 
     *
     */

    void enableTorque();
    void disableTorque(); 
    void rotate(u8 servoID);
    void setStartPos(u8 servoID);
    void setMidPos(u8 servoID);
    void setEndPos(u8 servoID);
    int  setPosition(u8 servoID, u16 position, u16 speed = 0); // default maximum speed
    int  setPositionFast(u8 servoID, u16 position);                         
    void setPosition12(u8 const servoIDs[], u16 const servoPositions[]);    
    bool checkPosition(u8 servoID, u16 position, int accuracy);
    void setID(u8 servoID, u8 newID);

    bool isEnabled {false}; 
    bool isTorqueEnabled {false}; 


    /* ASYNC API 
     *
     * A task synchronise a setpoint/feedback database and control the servo BUS trafic
     *
     * Members functions allow changing setpoint (pos) and reading feedback (pos,speed,load)
     *
     */

    void setPositionAsync(u8 servoID, u16 servoPosition);
    void setPosition12Async(u16 const servoPositions[]);    
    
    u16  getPositionAsync(u8 servoID);
    u16  getVelocityAsync(u8 servoID);
    u16  getLoadAsync(u8 servoID);

    // async service enable
    void enableAsyncService(bool enable);

    // internals
    void sync_all_goal_position();
    void cmd_feedback_one_servo(SERVO_STATE & servoState);
    void ack_feedback_one_servo(SERVO_STATE & servoState);

    // state of all servo
    SERVO_STATE state[12] {1,2,3,4,5,6,7,8,9,10,11,12}; // hard-coded ID list

    // background servo bus service
    bool isSyncRunning {false};
    TaskHandle_t task_handle {NULL};

    /* LOW LEVEL API
     *
     * R/W access to register through serial bus using SCS protocol
     * 
     */

    void write_register_byte(u8 reg, u8 value);
    void write_register_word(u8 reg, u8 value);

    u8 read_register_byte(u8 reg);
    u16 read_register_word(u8 reg);

    int uart_port_num;

};

extern SERVO servo;

#endif
