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
 * REQUEST SETPOINT exchange :
 *
 *  1) HOST send a REQUEST SETPOINT instruction.
 *     ID = 0x01
 *     Length = 26 bytes
 *     Instruction code = 0x01
 *     Parameters = 12 x position (u16) [0..1023]
 *
 *  2) ESP32 replies a REQUEST SETPOINT acknowledge.
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
#define INST_REQUEST_SETPOINT 0x01

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


struct HOST
{
    HOST();

private:
    int _uart_port_num {2};
    
    // background host serial bus service
    TaskHandle_t _task_handle {NULL};    
};


extern HOST host;

#endif //_mini_pupper_host_H
