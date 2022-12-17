/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

/* Manual : 
 * 
 *  Configure servo 1 to 12 : 
 *  -------------------------
 *   For each servo from N = 2 to 12 :
 *    a. Connect a SCS 0009 servo to the main board (facotry default ID is 1).
 *    b. CLI > servo-enable
 *    c. CLI > servo-scan (check servo ID=1 is listed)
 *    d. CLI > servo-setID --id 1 --newid N
 *    e. CLI > servo-scan (check new servo ID=N is listed)
 *    f. CLI > servo-disable
 *   Finish by connecting servo #1, that doesnt require any ID change.
 *    x. CLI > servo-enable
 *    y. CLI > servo-scan (check reply : "Servos on the bus:1 2 3 4 5 6 7 8 9 10 11 12")
 *    z. CLI > servo-disable
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* IMPORTANT : Mini Servo API requires to setup FreeRTOS frequency at 1000Hz.
 *             Use IDF ESP32 : MENUCONFIG > COMPONENTS > FREERTOS > KERNEL > 1000Hz
 *
 */

/* Select servo model :
 * - SCS 0009 : Mini Pupper V2
 * - Generic : to be defined
 */
#define SERVO_USE_SCS_0009
//#define SERVO_USE_SCS_GENERIC

#ifndef _mini_pupper_servos_H
#define _mini_pupper_servos_H

// types
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

// servo data rate (baud rate register values)
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
#define SERVO_VERSION_L 3
#define SERVO_VERSION_H 4
#define SERVO_ID 5
#define SERVO_BAUD_RATE 6
#define SERVO_MIN_ANGLE_LIMIT_L 9
#define SERVO_MIN_ANGLE_LIMIT_H 10
#define SERVO_MAX_ANGLE_LIMIT_L 11
#define SERVO_MAX_ANGLE_LIMIT_H 12
#define SERVO_CW_DEAD 26
#define SERVO_CCW_DEAD 27

// servo SRAM register address (setpoints)
#define SERVO_TORQUE_ENABLE 40
#define SERVO_GOAL_POSITION_L 42
#define SERVO_GOAL_POSITION_H 43
#define SERVO_GOAL_TIME_L 44
#define SERVO_GOAL_TIME_H 45
#define SERVO_GOAL_SPEED_L 46
#define SERVO_GOAL_SPEED_H 47
#define SERVO_LOCK 48

// servo SRAM register address (feedback)
#define SERVO_PRESENT_POSITION_L 56
#define SERVO_PRESENT_POSITION_H 57
#define SERVO_PRESENT_SPEED_L 58
#define SERVO_PRESENT_SPEED_H 59
#define SERVO_PRESENT_LOAD_L 60
#define SERVO_PRESENT_LOAD_H 61
// SCS 0009 registers :
#define SERVO_STATUS 65
#define SERVO_MOVING 66
// SCS generic registers :
#define SERVO_PRESENT_VOLTAGE 62
#define SERVO_PRESENT_TEMPERATURE 63
#define SERVO_PRESENT_CURRENT_L 69
#define SERVO_PRESENT_CURRENT_H 70


struct SERVO_STATE
{
    SERVO_STATE(u8 id) : ID(id) {}
    u8 ID                   {0};
    u16 goal_position       {512}; // middle position
    u16 present_position    {0};
    s16 present_velocity    {0};
    s16 present_load        {0};
    u8 present_voltage      {0};
    u8 present_temperature  {0};
    u8 present_move         {0};
    s16 present_current     {0};
};

enum {
    SERVO_STATUS_OK = 0,
    SERVO_STATUS_FAIL,
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
     * Each member function generates one or several instruction/reply frames on servo BUS 
     *
     */

    // single servo API
    int ping(u8 ID);
    int recovery(u8 ID);
    int enable_torque(u8 ID = 0xFE);  // not param means ALL servo
    int disable_torque(u8 ID = 0xFE); // not param means ALL servo
    int is_torque_enable(u8 ID, u8 & enable);
    int set_position(u8 ID, u16 position);
    int get_position(u8 ID, u16 & position);
    int get_velocity(u8 ID, s16 & velocity);
    int get_load(u8 ID, s16 & load);
    int get_voltage(u8 ID, u8 & voltage);           // return 0 with SCS 0009
    int get_temperature(u8 ID, u8 & temperature);   // return 0 with SCS 0009
    int get_move(u8 ID, u8 & move);
    int get_current(u8 ID, s16 & current);          // return 0 with SCS 0009
    int unlock_eeprom(u8 ID);
    int lock_eeprom(u8 ID);

    // all-servo API
    void set_position_all(u16 const servoPositions[]);    

    // advanced API
    int setID(u8 ID, u8 newID);

    // deprecated API
    int setStartPos(u8 servoID);
    int setMidPos(u8 servoID);
    int setEndPos(u8 servoID);
    int rotate(u8 servoID);
    bool checkPosition(u8 servoID, u16 position, int accuracy);

    bool isEnabled {false}; 

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
    s16  getVelocityAsync(u8 servoID);
    s16  getLoadAsync(u8 servoID);
    u8   getVoltageAsync(u8 servoID);       // return 0 with SCS 0009 and SCS Generic (future functionality)
    u8   getTemperatureAsync(u8 servoID);   // return 0 with SCS 0009 and SCS Generic (future functionality)
    u8   getMoveAsync(u8 servoID);          // return 0 with SCS 0009 and SCS Generic (future functionality)
    s16  getCurrentAsync(u8 servoID);       // return 0 with SCS 0009 and SCS Generic (future functionality)

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

    /* LOW LEVEL helpers
     *
     * R/W access to register through serial bus using smart protocol
     * 
     */
    int write_frame(u8 ID, u8 instruction, u8 const * parameters, size_t parameter_length);
    int reply_frame(u8 & ID, u8 & state, u8 * parameters, size_t parameter_length);

    int write_register_byte(u8 id, u8 reg, u8 value);
    int write_register_word(u8 id, u8 reg, u16 value);

    int read_register_byte(u8 id, u8 reg, u8 & value);
    int read_register_word(u8 id, u8 reg, u16 & value);

    int check_reply_frame_no_parameter(u8 & ID);

    int uart_port_num {1};
};

extern SERVO servo;

#endif
