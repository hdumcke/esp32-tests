#include "SCSCL.h"

#ifndef _mini_pupper_servos_H
#define _mini_pupper_servos_H

typedef enum 
{
    swt_gen     = 0x00,
    swt_reg     = 0x01,
    swt_sync    = 0x02
} servo_write_type;

class SERVO : public SCSCL
{
public:
    SERVO();
    void disable();
    void enable();
    void enableTorque();
    void disableTorque();
    void rotate(u8 servoID);
    void setStartPos(u8 servoID);
    void setMidPos(u8 servoID);
    void setEndPos(u8 servoID);
    int  setPosition(u8 servoID, u16 position, u16 speed = 1); //maximum speed
    bool checkPosition(u8 servoID, u16 position, int accuracy);
    void setID(u8 servoID, u8 newID);
    servo_write_type write_type;
    bool isEnabled; 
    bool isTorqueEnabled; 
};

#endif
