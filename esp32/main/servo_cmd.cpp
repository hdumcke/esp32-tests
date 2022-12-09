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

SERVO servo;

static const char *TAG = "SERVOCMD";
static uint64_t start_time = 0;
static uint64_t end_time = 0;

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
        .command = "servo-is-enabled",
        .help = "check if the servos are enabled",
        .hint = NULL,
        .func = &servo_cmd_isEnabled,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_isEnabled) );
}

static int servo_cmd_disableTorque(int argc, char **argv)
{
    servo.disableTorque();
    return 0;
}

static void register_servo_cmd_disableTorque(void)
{
    const esp_console_cmd_t cmd_servo_disableTorque = {
        .command = "servo-disableTorque",
        .help = "disabing torque on the servos",
        .hint = NULL,
        .func = &servo_cmd_disableTorque,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_disableTorque) );
}

static int servo_cmd_enableTorque(int argc, char **argv)
{
    servo.enableTorque();
    return 0;
}

static void register_servo_cmd_enableTorque(void)
{
    const esp_console_cmd_t cmd_servo_enableTorque = {
        .command = "servo-enableTorque",
        .help = "enabling torque on the servos",
        .hint = NULL,
        .func = &servo_cmd_enableTorque,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_enableTorque) );
}

static int servo_cmd_isTorqueEnabled(int argc, char **argv)
{
    if(servo.isTorqueEnabled) {
        printf("servos torque is enabled\r\n");
    }
    else{
        printf("servos torque is not enabled\r\n");
    }
    return 0;
}

static void register_servo_cmd_isTorqueEnabled(void)
{
    const esp_console_cmd_t cmd_servo_isTorqueEnabled = {
        .command = "servo-is-torque-enabled",
        .help = "check if the servos torque is enabled",
        .hint = NULL,
        .func = &servo_cmd_isTorqueEnabled,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_isTorqueEnabled) );
}

static int servo_cmd_scan(int argc, char **argv)
{
    printf("Servos on the bus:\r\n");
    for(u8 i = 0; i<13; i++)
    {
        if(servo.Ping(i) == i) {
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

static struct {
    struct arg_int *servo_id;
    struct arg_end *end;
} servo_id_args;

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

static struct {
    struct arg_int *loop;
    struct arg_end *end;
} servo_rotate12_args;

static int servo_cmd_rotate12(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_rotate12_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_rotate12_args.end, argv[0]);
        return 0;
    }
    int loop = servo_rotate12_args.loop->ival[0];
    int i = 0;
    int l = 0;
    static u8 const servoIDs[] {1,2,3,4,5,6,7,8,9,10,11,12};
    static u16 servoPositions[12] {0};
    servo.setPosition12(servoIDs,servoPositions);
    // make sure the servo is in the starting position before we measure time
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    start_time = esp_timer_get_time();
    for (l = 0; l < loop; l++) {
        for (i = 0; i < 1024; i++) {
            for(size_t index=0;index<12;++index) {
                servoPositions[index] = i;
            }
    	    servo.setPosition12(servoIDs,servoPositions);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        for (i = 1023; i > 0; i--) {
            for(size_t index=0;index<12;++index) {
                servoPositions[index] = i;
            }
    	    servo.setPosition12(servoIDs,servoPositions);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    end_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Time: %llu microseconds", end_time-start_time);

    return 0;
}

static void register_servo_cmd_rotate12(void)
{
    servo_rotate12_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_rotate12_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_rotate12 = {
        .command = "servo-rotate12",
        .help = "rotate the servos",
        .hint = "--loop <n>",
        .func = &servo_cmd_rotate12,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_rotate12) );
}

static struct {
    struct arg_int *servo_id;
    struct arg_int *start_pos;
    struct arg_int *end_pos;
    struct arg_int *accuracy;
    struct arg_int *speed;
    struct arg_end *end;
} servo_perftest_args;

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
    int speed = servo_perftest_args.speed->ival[0];
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
        servo.setPosition((u8)id, (u16)start_pos, (u16) speed);
    }
    // make sure the servo is in the starting position before we measure time
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    bool hasError = false;
    for (id = start_range; id <= end_range; id++) {
        if(! servo.checkPosition((u8)id, (u16)start_pos, accuracy)) {
            ESP_LOGE(TAG, "Servo %d has not reached it's starting position. Expected % d Read, %d", id, start_pos, servo.ReadPos((u8)id));
            hasError = true;
        }
    }
    if(hasError) { return 0; }

    start_time = esp_timer_get_time();
    for (id = start_range; id <= end_range; id++) {
        servo.setPosition((u8)id, (u16)end_pos, (u16) speed);
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
    servo_perftest_args.speed = arg_int1(NULL, "speed", "<n>", "Speed");
    servo_perftest_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_perftest = {
        .command = "servo-perftest",
        .help = "servo performance test",
        .hint = "--id <servoID> --start <startpos> --end <endpos>  --acc <n> --speed <n># id=999: servos 1-12; acc>1023: we do not read position",
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
    servo.setPosition((u8)servo_id, (u16)servo_pos);
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

static struct {
    struct arg_int *servo_pos12;
    struct arg_end *end;
} servo_pos12_args;

static int servo_cmd_setPosition12(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_pos12_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_pos_args.end, argv[0]);
        return 0;
    }
    static u8 const servoIDs[] {1,2,3,4,5,6,7,8,9,10,11,12};
    static u16 servoPositions[12] {0};
    for(size_t index=0;index<12;++index) {
        int const & value = servo_pos12_args.servo_pos12->ival[index];
        if(0<=value && value<1024)
            servoPositions[index]=static_cast<u16>(value);
        else
            servoPositions[index]=512;
    }
    servo.setPosition12(servoIDs,servoPositions);
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

static struct {
    struct arg_int *servo_id;
    struct arg_int *loop;
    struct arg_end *end;
} servo_loop_args;

static int servo_cmd_FeedBack(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_loop_args.servo_id->ival[0];
    int loop = servo_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        printf("FeedBack: %d\r\n", servo.FeedBack((u8)servo_id));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_FeedBack(void)
{
    servo_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_FeedBack = {
        .command = "servo-FeedBack",
        .help = "return FeedBack",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_FeedBack,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_FeedBack) );
}

static int servo_cmd_ReadPos(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_loop_args.servo_id->ival[0];
    int loop = servo_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        printf("ReadPos: %d\r\n", servo.ReadPos((u8)servo_id));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_ReadPos(void)
{
    servo_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadPos = {
        .command = "servo-ReadPos",
        .help = "return ReadPos",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_ReadPos,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadPos) );
}

static int servo_cmd_ReadSpeed(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_loop_args.servo_id->ival[0];
    int loop = servo_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        printf("ReadSpeed: %d\r\n", servo.ReadSpeed((u8)servo_id));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_ReadSpeed(void)
{
    servo_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadSpeed = {
        .command = "servo-ReadSpeed",
        .help = "return ReadSpeed",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_ReadSpeed,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadSpeed) );
}

static int servo_cmd_ReadLoad(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_loop_args.servo_id->ival[0];
    int loop = servo_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        printf("ReadLoad: %d\r\n", servo.ReadLoad((u8)servo_id));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_ReadLoad(void)
{
    servo_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadLoad = {
        .command = "servo-ReadLoad",
        .help = "return ReadLoad",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_ReadLoad,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadLoad) );
}

static int servo_cmd_ReadVoltage(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_loop_args.servo_id->ival[0];
    int loop = servo_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        printf("ReadVoltage: %d\r\n", servo.ReadVoltage((u8)servo_id));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_ReadVoltage(void)
{
    servo_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadVoltage = {
        .command = "servo-ReadVoltage",
        .help = "return ReadVoltage",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_ReadVoltage,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadVoltage) );
}

static int servo_cmd_ReadTemper(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_loop_args.servo_id->ival[0];
    int loop = servo_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        printf("ReadTemper: %d\r\n", servo.ReadTemper((u8)servo_id));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_ReadTemper(void)
{
    servo_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadTemper = {
        .command = "servo-ReadTemper",
        .help = "return ReadTemper",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_ReadTemper,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadTemper) );
}

static int servo_cmd_ReadMove(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_loop_args.servo_id->ival[0];
    int loop = servo_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        printf("ReadMove: %d\r\n", servo.ReadMove((u8)servo_id));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_ReadMove(void)
{
    servo_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadMove = {
        .command = "servo-ReadMove",
        .help = "return ReadMove",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_ReadMove,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadMove) );
}

static int servo_cmd_ReadCurrent(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&servo_loop_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, servo_loop_args.end, argv[0]);
        return 0;
    }

    /* Check servoID "--id" option */
    int servo_id = servo_loop_args.servo_id->ival[0];
    int loop = servo_loop_args.loop->ival[0];
    if(servo_id<0) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    for(int i=0; i<loop; i++){
        printf("ReadCurrent: %d\r\n", servo.ReadCurrent((u8)servo_id));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}

static void register_servo_cmd_ReadCurrent(void)
{
    servo_loop_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_loop_args.loop = arg_int1(NULL, "loop", "<n>", "loop <n> times");
    servo_loop_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadCurrent = {
        .command = "servo-ReadCurrent",
        .help = "return ReadCurrent",
        .hint = "--id <servoID> --loop <n>",
        .func = &servo_cmd_ReadCurrent,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadCurrent) );
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
        .command = "servo-setPosAsync",
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
        .command = "servo-setPos12Async",
        .help = "rotate the servos to a given position",
        .hint = "<pos> (x12)",
        .func = &servo_cmd_setPosition12Async,
        .argtable = &servo_pos12_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_setPosition12) );
}

static int servo_cmd_ReadPosAsync(int argc, char **argv)
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
    printf("ReadPos: %d\r\n", servo.getPositionAsync((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadPosAsync(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadPos = {
        .command = "servo-ReadPosAsync",
        .help = "return ReadPos",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadPosAsync,
        .argtable = &servo_id_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadPos) );
}

static int servo_cmd_ReadVelAsync(int argc, char **argv)
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
    printf("ReadVel: %d\r\n", servo.getVelocityAsync((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadVelAsync(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadVel = {
        .command = "servo-ReadVelAsync",
        .help = "return ReadVel",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadVelAsync,
        .argtable = &servo_id_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadVel) );
}

static int servo_cmd_ReadLoadAsync(int argc, char **argv)
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
    printf("ReadLoad: %d\r\n", servo.getLoadAsync((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadLoadAsync(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadLoad = {
        .command = "servo-ReadLoadAsync",
        .help = "return ReadLoad",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadLoadAsync,
        .argtable = &servo_id_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadLoad) );
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
    register_servo_cmd_FeedBack();
    register_servo_cmd_ReadPos();
    register_servo_cmd_ReadSpeed();
    register_servo_cmd_ReadLoad();
    register_servo_cmd_ReadVoltage();
    register_servo_cmd_ReadTemper();
    register_servo_cmd_ReadMove();
    register_servo_cmd_ReadCurrent();

    register_servo_cmd_setPositionAsync();
    register_servo_cmd_setPosition12Async();
    register_servo_cmd_ReadPosAsync();
    register_servo_cmd_ReadVelAsync();
    register_servo_cmd_ReadLoadAsync();
}
