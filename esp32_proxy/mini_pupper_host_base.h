/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

#ifndef _mini_pupper_host_base_H
#define _mini_pupper_host_base_H

#include "mini_pupper_types.h"

// host instruction code
#define INST_CONTROL 0x01   // Host sends servo position setpoints, ESP32 replies with servo feedback, attitude, ....

// frame parameters format for control instruction
struct parameters_control_instruction_format
{
    u16 goal_position[12];
};

// frame parameters format for control acknowledge
struct parameters_control_acknowledge_format
{
    u16 present_position[12];
    s16 present_load[12];
    // IMU data
    float roll;
    float pitch;
    float yaw;
};


#endif //_mini_pupper_host_base_H
