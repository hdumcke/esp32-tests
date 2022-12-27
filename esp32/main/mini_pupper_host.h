/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

#ifndef _mini_pupper_host_H
#define _mini_pupper_host_H

#include "mini_pupper_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* IMPORTANT : Mini Pupper Host API requires to setup FreeRTOS frequency at 1000Hz.
 *             Use IDF ESP32 : MENUCONFIG > COMPONENTS > FREERTOS > KERNEL > 1000Hz
 *
 */

/* PROTOCOL
 *
 * Frame format : 
 *  Header    (16bits) : 0xFF 0xFF
 *  Identifier (8bits) : [0x01,0xFE]
 *  Length     (8bits) : Payload length in bytes + 1
 *  Payload  (L bytes) : Variable size data payload
 *  Checksum   (8bits) : Checksum of all fields except header
 *
 *
 * Payload format :
 *  Instruction code (8bits) : [0x01..]
 *  Parameters     (N bytes) : [0..64]
 *
 *
 * CONTROL exchange :
 *
 *  1) HOST sends a CONTROL instruction.
 *     ID = 0x01
 *     Length = 26 bytes
 *     Instruction code = 0x01
 *     Parameters = 12 x position (u16) [0..1023]
 *
 *     Test frame with all servo to neutral :
 *          0xff 0xff 0x01 0x1a     Header, ID=1, Length=26
 *          0x01 0x00 0x02 0x00     Instruction=1(CONTROL), goal_position[0]=512...
 *          0x02 0x00 0x02 0x00
 *          0x02 0x00 0x02 0x00
 *          0x02 0x00 0x02 0x00
 *          0x02 0x00 0x02 0x00
 *          0x02 0x00 0x02 0x00     ...goal_position[11]=512
 *          0x02 0xcb               Checksum
 *
 *  2) ESP32 replies a CONTROL acknowledge in less than 2ms.
 *     ID = 0x01
 *     Length = 50 bytes
 *     Status code = 0x00
 *     Parameters = 
 *      * 12 x position (u16) [0..1023]
 *      * 12 x load     (s16) [-1000..+1000]
 *
 *
 */


// host instruction code
#define INST_CONTROL 0x01   // Host sends servo position setpoints, ESP32 replies with servo feedback, attitude, ....


// host parameter format for control instruction
struct parameters_control_instruction_format
{
    u16 goal_position[12];
};
struct parameters_control_acknowledge_format
{
    u16 present_position[12];
    s16 present_load[12];
    // IMU data
};


struct HOST_STATE
{
    HOST_STATE(u8 id) : ID(id) {}
    u8 ID                   {0};
    u16 goal_position       {512}; // middle position
    u16 present_position    {0};
    s16 present_speed       {0};
    s16 present_load        {0};
};

enum {
    HOST_STATUS_OK = 0,
    HOST_STATUS_FAIL,
};

void HOST_TASK(void * parameters);

struct HOST
{
    HOST();

    void start();

private:
    int _uart_port_num {2};
    
    // background host serial bus service
    TaskHandle_t _task_handle {NULL};    
    friend void HOST_TASK(void * parameters);
};


extern HOST host; 

#endif //_mini_pupper_host_H
