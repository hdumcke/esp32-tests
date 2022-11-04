#include "SCSCL.h"

#ifndef _mini_pupper_servos_H
#define _mini_pupper_servos_H

class SERVO : public SCSCL
{
public:
    SERVO();
    void disable();
    void enable();
    void rotate(u8 servoID);
    void setStartPos(u8 servoID);
    void setMidPos(u8 servoID);
    void setEndPos(u8 servoID);
    void setPosition(u8 servoID, u16 position);
    void setID(u8 servoID, u8 newID);
    void readServo(u8 servoID);
    bool isEnabled; 
};

#endif
