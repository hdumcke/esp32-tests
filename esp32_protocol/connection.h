#include <stdint.h>

#define SOCKET_NAME "/tmp/9Lq7BNBnBycd6nxy.socket"


typedef uint16_t u16;
typedef uint8_t u8;

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


// state of all servo
SERVO_STATE state[12] {1,2,3,4,5,6,7,8,9,10,11,12}; // hard-coded ID list

#define INST_SETPOS 0x01
#define INST_GETPOS 0x02
#define INST_GETIMU 0x03
#define INST_DISABLE 0x04

// prepare sync write frame to all servo
static size_t const L {2};                      // Length of data sent to each servo
static size_t const N {12};                     // Servo Number
static size_t const Length {L*N};             // Length field value
static size_t const buffer_size {1+1+Length}; // 0xFF 0xFF ID LENGTH (INSTR PARAM... CHK)
