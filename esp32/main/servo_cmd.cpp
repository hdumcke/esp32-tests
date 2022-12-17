#include "mini_pupper_servos.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include "argtable3/argtable3.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "SERVOCMD";

static struct
{
    struct arg_int *servo_id;
    struct arg_end *end;
} servo_id_args;

static struct
{
    struct arg_int *loop;
    struct arg_end *end;
} servo_loop_args;

static struct
{
    struct arg_int *servo_id;
    struct arg_int *start_pos;
    struct arg_int *end_pos;
    struct arg_int *accuracy;
    struct arg_end *end;
} servo_perftest_args;

static struct
{
    struct arg_int *servo_id;
    struct arg_int *loop;
    struct arg_end *end;
} servo_id_loop_args;

static struct
{
    struct arg_int *servo_pos12;
    struct arg_end *end;
} servo_pos12_args;

static uint64_t start_time = 0;
static uint64_t end_time = 0;

/*
 * Switch ON/OFF servo power supply
 *
 */

static int servo_cmd_disable(int argc, char **argv)
{
    servo.disable();
    return 0;
}

static void register_servo_cmd_disable(void)
{
    const esp_console_cmd_t cmd_servo_disable = {
        .command = "servo-disable",
        .help = "disabing the servos",
        .hint = NULL,
        .func = &servo_cmd_disable,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_disable) );
}

static int servo_cmd_enable(int argc, char **argv)
{
    servo.enable();
    return 0;
}

static void register_servo_cmd_enable(void)
{
    const esp_console_cmd_t cmd_servo_enable = {
        .command = "servo-enable",
        .help = "disabing the servos",
        .hint = NULL,
        .func = &servo_cmd_enable,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_enable) );
}

static int servo_cmd_isEnabled(int argc, char **argv)
{
    if(servo.isEnabled) {
        printf("servos are enabled\r\n");
    }
    else{
        printf("servos are not enabled\r\n");
    }
    return 0;
}

static void register_servo_cmd_isEnabled(void)
{
    const esp_console_cmd_t cmd_servo_isEnabled = {
        .command = "servo-isEnabled",
        .help = "check if the servos are enabled",
        .hint = NULL,
        .func = &servo_cmd_isEnabled,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_isEnabled) );
}

/*
 * Switch ON/OFF servo torque
 *
 */

static int servo_cmd_disableTorque(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 && servo_id != 254 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id == 254) {
        printf("Warning: your servo ID is the broadcast ID\r\n");
    }
    servo.disable_torque((u8)servo_id);
    return 0;
}

static void register_servo_cmd_disableTorque(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo = {
        .command = "servo-disableTorque",
        .help = "disable torque",
        .hint = "--id <servoID>",
        .func = &servo_cmd_disableTorque,
        .argtable = &servo_id_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo) );
}


static int servo_cmd_enableTorque(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 && servo_id != 254 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id == 254) {
        printf("Warning: your servo ID is the broadcast ID\r\n");
    }
    servo.enable_torque((u8)servo_id);
    return 0;
}

static void register_servo_cmd_enableTorque(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo = {
        .command = "servo-enableTorque",
        .help = "enable torque",
        .hint = "--id <servoID>",
        .func = &servo_cmd_enableTorque,
        .argtable = &servo_id_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo) );
}


static int servo_cmd_isTorqueEnabled(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    u8 enable {0};
    servo.is_torque_enable((u8)servo_id,enable);
    if(enable) {
        printf("servos torque is enabled\r\n");
    }
    else{
        printf("servos torque is not enabled\r\n");
    }
    return 0;
}

static void register_servo_cmd_isTorqueEnabled(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo = {
        .command = "servo-isTorqueEnable",
        .help = "check torque switch",
        .hint = "--id <servoID>",
        .func = &servo_cmd_isTorqueEnabled,
        .argtable = &servo_id_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo) );
}

/*
 * Ping servo
 *
 */

static int servo_cmd_scan(int argc, char **argv)
{
    printf("Servos on the bus:\r\n");
    for(u8 i = 1; i<13; i++)
    {
        if(servo.ping(i) == SERVO_STATUS_OK)
        {
            printf("%d ", i);
	   }
    }
    printf("\r\n");
    return 0;
}

static void register_servo_cmd_scan(void)
{
    const esp_console_cmd_t cmd_servo_scan = {
        .command = "servo-scan",
        .help = "scan the bus for servos",
        .hint = NULL,
        .func = &servo_cmd_scan,
	   .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_scan) );
}

static int servo_cmd_rotate(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 && servo_id != 254 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id == 254) {
        printf("Warning: your servo ID is the broadcast ID\r\n");
    }
    servo.rotate((u8)servo_id);
    return 0;
}

static void register_servo_cmd_rotate(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_rotate = {
        .command = "servo-rotate",
        .help = "rotate the servos",
        .hint = "--id <servoID>",
        .func = &servo_cmd_rotate,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_rotate) );
}

static int servo_cmd_rotate12(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_loop_args.end, argv[0]);
        return 0;
    }
    int loop = servo_loop_args.loop->ival[0];
    int i = 0;
    int l = 0;
    static u16 servoPositions[12] {0};
    servo.set_position_all(servoPositions);
    // make sure the servo is in the starting position before we measure time
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    start_time = esp_timer_get_time();
    for (l = 0; l < loop; l++) {
        for (i = 0; i < 1024; i++) {
            for(size_t index=0;index<12;++index) {
                servoPositions[index] = i;
            }
    	    servo.set_position_all(servoPositions);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        for (i = 1023; i > 0; i--) {
            for(size_t index=0;index<12;++index) {
                servoPositions[index] = i;
            }
    	    servo.set_position_all(servoPositions);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    end_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Time: %llu microseconds", end_time-start_time);

    return 0;
}

static void register_servo_cmd_rotate12(void)
{
    servo_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_rotate12 = {
        .command = "servo-rotate12",
        .help = "rotate the servos",
        .hint = "--loop <n>",
        .func = &servo_cmd_rotate12,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_rotate12) );
}

static int servo_cmd_perftest(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_perftest_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_perftest_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_perftest_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 && servo_id != 999 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }

    int start_pos = servo_perftest_args.start_pos->ival[0];
    int end_pos = servo_perftest_args.end_pos->ival[0];
    int accuracy = servo_perftest_args.accuracy->ival[0];
    int start_range = 0;
    int end_range = 0;

    if( servo_id == 999) {
        start_range = 1;
        end_range = 12;
    }
    else {
        start_range = servo_id;
        end_range = servo_id;
    }
    int id = 0;
    for (id = start_range; id <= end_range; id++) {
        servo.set_position((u8)id, (u16)start_pos);
    }
    // make sure the servo is in the starting position before we measure time
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    bool hasError = false;
    for (id = start_range; id <= end_range; id++) {
        if(! servo.checkPosition((u8)id, (u16)start_pos, accuracy)) {
            u16 position {0};
            servo.get_position((u8)id,position);
            ESP_LOGE(TAG, "Servo %d has not reached it's starting position. Expected % d Read, %d", id, start_pos, position);
            hasError = true;
        }
    }
    if(hasError) { return 0; }

    start_time = esp_timer_get_time();
    for (id = start_range; id <= end_range; id++) {
        servo.set_position((u8)id, (u16)end_pos);
    }
    if(accuracy<=1023) {
        for (id = start_range; id <= end_range; id++) {
            while(! servo.checkPosition((u8)id, (u16)end_pos, accuracy)) {
                vTaskDelay(1 / portTICK_PERIOD_MS);
            }
        }
    }
    end_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Time: %llu microseconds", end_time-start_time);
    return 0;
}

static void register_servo_cmd_perftest(void)
{
    servo_perftest_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_perftest_args.start_pos = arg_int1(NULL, "start", "<n>", "Start Position");
    servo_perftest_args.end_pos = arg_int1(NULL, "end", "<n>", "End Position");
    servo_perftest_args.accuracy = arg_int1(NULL, "acc", "<n>", "Read Accuracy");
    servo_perftest_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_perftest = {
        .command = "servo-perftest",
        .help = "servo performance test",
        .hint = "--id <servoID> --start <startpos> --end <endpos>  --acc <n> # id=999: servos 1-12; acc>1023: we do not read position",
        .func = &servo_cmd_perftest,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_perftest) );
}


static int servo_cmd_setStartPos(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 && servo_id != 254 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id == 254) {
        printf("Warning: your servo ID is the broadcast ID\r\n");
    }
    servo.setStartPos((u8)servo_id);
    return 0;
}

static void register_servo_cmd_setStartPos(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_setStartPos = {
        .command = "servo-setStartPos",
        .help = "rotate the servos to the start position",
        .hint = "--id <servoID>",
        .func = &servo_cmd_setStartPos,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_setStartPos) );
}

static int servo_cmd_setMidPos(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 && servo_id != 254 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id == 254) {
        printf("Warning: your servo ID is the broadcast ID\r\n");
    }
    servo.setMidPos((u8)servo_id);
    return 0;
}

static void register_servo_cmd_setMidPos(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_setMidPos = {
        .command = "servo-setMidPos",
        .help = "rotate the servos to the middle position",
        .hint = "--id <servoID>",
        .func = &servo_cmd_setMidPos,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_setMidPos) );
}

static int servo_cmd_setEndPos(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 && servo_id != 254 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id == 254) {
        printf("Warning: your servo ID is the broadcast ID\r\n");
    }
    servo.setEndPos((u8)servo_id);
    return 0;
}

static void register_servo_cmd_setEndPos(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_setEndPos = {
        .command = "servo-setEndPos",
        .help = "rotate the servos to the end position",
        .hint = "--id <servoID>",
        .func = &servo_cmd_setEndPos,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_setEndPos) );
}

static struct {
    struct arg_int *servo_id;
    struct arg_int *servo_pos;
    struct arg_end *end;
} servo_pos_args;

static int servo_cmd_setPosition(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_pos_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_pos_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_pos_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 && servo_id != 254 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id == 254) {
        printf("Warning: your servo ID is the broadcast ID\r\n");
    }

    /* Check servo_pos "--pos" option */
    int servo_pos = servo_pos_args.servo_pos->ival[0];
    if(servo_pos<0 || servo_pos>1023) {
        printf("Invalid servo position\r\n");
        return 0;
    }
    servo.set_position((u8)servo_id, (u16)servo_pos);
    return 0;
}

static void register_servo_cmd_setPosition(void)
{
    servo_pos_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_pos_args.servo_pos = arg_int1(NULL, "pos", "<n>", "Servo Position");
    servo_pos_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_setPosition = {
        .command = "servo-setPosition",
        .help = "rotate the servos to a given position",
        .hint = "--id <servoID> --pos <position>",
        .func = &servo_cmd_setPosition,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_setPosition) );
}

static int servo_cmd_setPosition12(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_pos12_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_pos_args.end, argv[0]);
        return 0;
    }
    static u16 servoPositions[12] {0};
    for(size_t index=0;index<12;++index) {
        int const & value = servo_pos12_args.servo_pos12->ival[index];
        if(0<=value && value<1024)
            servoPositions[index]=static_cast<u16>(value);
        else
            servoPositions[index]=512;
    }
    servo.set_position_all(servoPositions);
    return 0;
}

static void register_servo_cmd_setPosition12(void)
{
    servo_pos12_args.servo_pos12 = arg_intn(NULL,NULL,"<pos>",12,12,"Servo position array (x12)");
    servo_pos12_args.end = arg_end(1);
    const esp_console_cmd_t cmd_servo_setPosition12 = {
        .command = "servo-setPosition12",
        .help = "rotate the servos to a given position",
        .hint = "<pos> (x12)",
        .func = &servo_cmd_setPosition12,
        .argtable = &servo_pos12_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_setPosition12) );
}

static struct {
    struct arg_int *servo_id;
    struct arg_int *servo_newid;
    struct arg_end *end;
} servo_newid_args;

static int servo_cmd_setID(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_newid_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_newid_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_newid_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }

    /* Check servo_newid "--newid" option */
    int servo_newid = servo_newid_args.servo_newid->ival[0];
    if(servo_newid<0 || servo_newid>12) {
        printf("Invalid new servo ID\r\n");
        return 0;
    }
    servo.setID((u8)servo_id, (u8)servo_newid);
    return 0;
}

static void register_servo_cmd_setID(void)
{
    servo_newid_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_newid_args.servo_newid = arg_int1(NULL, "newid", "<n>", "New Servo ID");
    servo_newid_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_setID = {
        .command = "servo-setID",
        .help = "Change the servo ID",
        .hint = "--id <servoID> --newid <new servoID>",
        .func = &servo_cmd_setID,
	   .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_setID) );
}

static int servo_cmd_getPosition(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_loop_args.servo_id->ival[0];
    int loop = servo_id_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        u16 position {0};
        servo.get_position((u8)servo_id, position);
        printf("Pos: %d\r\n", position);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_getPosition(void)
{
    servo_id_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_id_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo = {
        .command = "servo-getPosition",
        .help = "return position",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_getPosition,
        .argtable = &servo_id_loop_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo) );
}

static int servo_cmd_getSpeed(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_loop_args.servo_id->ival[0];
    int loop = servo_id_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        s16 speed {0};
        servo.get_speed((u8)servo_id,speed);
        printf("Speed: %d\r\n", speed);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_getSpeed(void)
{
    servo_id_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_id_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_getSpeed = {
        .command = "servo-getSpeed",
        .help = "return speed",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_getSpeed,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_getSpeed) );
}

static int servo_cmd_getLoad(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_loop_args.servo_id->ival[0];
    int loop = servo_id_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        s16 load {0};
        servo.get_load((u8)servo_id,load);
        printf("Load: %d\r\n", load);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_getLoad(void)
{
    servo_id_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_id_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_getLoad = {
        .command = "servo-getLoad",
        .help = "return load",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_getLoad,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_getLoad) );
}

static int servo_cmd_getVoltage(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_loop_args.servo_id->ival[0];
    int loop = servo_id_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        u8 voltage {0};
        servo.get_voltage((u8)servo_id,voltage);
        printf("Voltage: %d\r\n", voltage);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_getVoltage(void)
{
    servo_id_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_id_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_getVoltage = {
        .command = "servo-getVoltage",
        .help = "return voltage",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_getVoltage,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_getVoltage) );
}

static int servo_cmd_getTemperature(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_loop_args.servo_id->ival[0];
    int loop = servo_id_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        u8 temperature {0};
        servo.get_temperature((u8)servo_id,temperature);
        printf("Temp: %d\r\n", temperature);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_getTemperature(void)
{
    servo_id_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_id_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_getTemperature = {
        .command = "servo-getTemperature",
        .help = "return temperature",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_getTemperature,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_getTemperature) );
}

static int servo_cmd_getMove(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_loop_args.servo_id->ival[0];
    int loop = servo_id_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        u8 move {0};
        servo.get_move((u8)servo_id,move);
        printf("Move: %d\r\n", move);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_getMove(void)
{
    servo_id_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_id_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_getMove = {
        .command = "servo-getMove",
        .help = "return move",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_getMove,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_getMove) );
}

static int servo_cmd_getCurrent(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_loop_args.servo_id->ival[0];
    int loop = servo_id_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        s16 current {0};
        servo.get_current((u8)servo_id,current);
        printf("Current: %d\r\n", current);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_getCurrent(void)
{
    servo_id_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_id_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_getCurrent = {
        .command = "servo-getCurrent",
        .help = "return current",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_getCurrent,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_getCurrent) );
}

static int servo_cmd_setPositionAsync(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_pos_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_pos_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_pos_args.servo_id->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 && servo_id != 254 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id == 254) {
        printf("Warning: your servo ID is the broadcast ID\r\n");
    }

    /* Check servo_pos "--pos" option */
    int servo_pos = servo_pos_args.servo_pos->ival[0];
    if(servo_pos<0 || servo_pos>1023) {
        printf("Invalid servo position\r\n");
        return 0;
    }
    servo.setPositionAsync((u8)servo_id, (u16)servo_pos);
    return 0;
}

static void register_servo_cmd_setPositionAsync(void)
{
    servo_pos_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_pos_args.servo_pos = arg_int1(NULL, "pos", "<n>", "Servo Position");
    servo_pos_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_setPosition = {
        .command = "servo-setPositionAsync",
        .help = "rotate the servos to a given position",
        .hint = "--id <servoID> --pos <position>",
        .func = &servo_cmd_setPositionAsync,
        .argtable = &servo_pos_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_setPosition) );
}


static int servo_cmd_setPosition12Async(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_pos12_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_pos_args.end, argv[0]);
        return 0;
    }
    static u16 servoPositions[12] {0};
    for(size_t index=0;index<12;++index) {
        int const & value = servo_pos12_args.servo_pos12->ival[index];
        if(0<=value && value<1024)
            servoPositions[index]=static_cast<u16>(value);
        else
            servoPositions[index]=512;
    }
    servo.setPosition12Async(servoPositions);
    return 0;
}

static void register_servo_cmd_setPosition12Async(void)
{
    servo_pos12_args.servo_pos12 = arg_intn(NULL,NULL,"<pos>",12,12,"Servo position array (x12)");
    servo_pos12_args.end = arg_end(1);
    const esp_console_cmd_t cmd_servo_setPosition12 = {
        .command = "servo-setPosition12Async",
        .help = "rotate the servos to a given position",
        .hint = "<pos> (x12)",
        .func = &servo_cmd_setPosition12Async,
        .argtable = &servo_pos12_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_setPosition12) );
}

static int servo_cmd_getPositionAsync(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<=0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("Pos: %d\r\n", servo.getPositionAsync((u8)servo_id));
    return 0;
}

static void register_servo_cmd_getPositionAsync(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadPos = {
        .command = "servo-getPositionAsync",
        .help = "return position",
        .hint = "--id <servoID>",
        .func = &servo_cmd_getPositionAsync,
        .argtable = &servo_id_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadPos) );
}

static int servo_cmd_getSpeedAsync(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<=0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("ReadVel: %d\r\n", servo.getSpeedAsync((u8)servo_id));
    return 0;
}

static void register_servo_cmd_getSpeedAsync(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadVel = {
        .command = "servo-getSpeedAsync",
        .help = "return speed",
        .hint = "--id <servoID>",
        .func = &servo_cmd_getSpeedAsync,
        .argtable = &servo_id_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadVel) );
}

static int servo_cmd_getLoadAsync(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_id_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_id_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_id_args.servo_id->ival[0];
    if(servo_id<=0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("Load: %d\r\n", servo.getLoadAsync((u8)servo_id));
    return 0;
}

static void register_servo_cmd_getLoadAsync(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_getLoad = {
        .command = "servo-getLoadAsync",
        .help = "return load",
        .hint = "--id <servoID>",
        .func = &servo_cmd_getLoadAsync,
        .argtable = &servo_id_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_getLoad) );
}

void register_servo_cmds(void)
{
    register_servo_cmd_disable();
    register_servo_cmd_enable();
    register_servo_cmd_isEnabled();
    register_servo_cmd_disableTorque();
    register_servo_cmd_enableTorque();
    register_servo_cmd_isTorqueEnabled();
    register_servo_cmd_scan();
    register_servo_cmd_rotate();
    register_servo_cmd_rotate12();
    register_servo_cmd_perftest();
    register_servo_cmd_setStartPos();
    register_servo_cmd_setMidPos();
    register_servo_cmd_setEndPos();
    register_servo_cmd_setPosition();
    register_servo_cmd_setPosition12();
    register_servo_cmd_setID();
    register_servo_cmd_getPosition();
    register_servo_cmd_getSpeed();
    register_servo_cmd_getLoad();
    register_servo_cmd_getVoltage();
    register_servo_cmd_getTemperature();
    register_servo_cmd_getMove();
    register_servo_cmd_getCurrent();
    register_servo_cmd_setPositionAsync();
    register_servo_cmd_setPosition12Async();
    register_servo_cmd_getPositionAsync();
    register_servo_cmd_getSpeedAsync();
    register_servo_cmd_getLoadAsync();
}
