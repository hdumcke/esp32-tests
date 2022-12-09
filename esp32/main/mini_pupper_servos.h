#include "SCSCL.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifndef _mini_pupper_servos_H
#define _mini_pupper_servos_H

// MENUCONFIG > COMPONENTS > FREERTOS > KERNEL > 1000Hz
// MENUCONFIG > COMPONENTS > FREERTOS > KERNEL > 1000Hz
// MENUCONFIG > COMPONENTS > FREERTOS > KERNEL > 1000Hz

struct SERVO_STATE
{
    SERVO_STATE(u8 id) : ID(id) {}

    u8 ID                   = 0;
    u8 torque_switch        = 0;
    u16 goal_position       = 512; // middle position
    u16 present_position    = 0;
    u16 present_velocity    = 0;
    u16 present_load        = 0;
};

void SERVO_TASK(void * parameters);

class SERVO : public SCSCL
{
public:
    SERVO();

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
};

#endif
