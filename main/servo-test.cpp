#include "mini_pupper_servos.hpp"
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
#include "nvs.h"
#include "nvs_flash.h"

static const char* TAG = "servo_tests";

SERVO servo;

#if CONFIG_CONSOLE_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
            .format_if_mount_failed = true,
            .max_files = 4,
	    .allocation_unit_size = 1024,
	    .disk_status_check_enable = false
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_STORE_HISTORY

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

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
        .help = "enabling the servos",
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

static int servo_cmd_FeedBack(int argc, char **argv)
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
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("FeedBack: %d\r\n", servo.FeedBack((u8)servo_id));
    return 0;
}

static void register_servo_cmd_FeedBack(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_FeedBack = {
        .command = "servo-FeedBack",
        .help = "return FeedBack",
        .hint = "--id <servoID>",
        .func = &servo_cmd_FeedBack,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_FeedBack) );
}

static int servo_cmd_ReadPos(int argc, char **argv)
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
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("ReadPos: %d\r\n", servo.ReadPos((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadPos(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadPos = {
        .command = "servo-ReadPos",
        .help = "return ReadPos",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadPos,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadPos) );
}

static int servo_cmd_ReadSpeed(int argc, char **argv)
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
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("ReadSpeed: %d\r\n", servo.ReadSpeed((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadSpeed(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadSpeed = {
        .command = "servo-ReadSpeed",
        .help = "return ReadSpeed",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadSpeed,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadSpeed) );
}

static int servo_cmd_ReadLoad(int argc, char **argv)
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
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("ReadLoad: %d\r\n", servo.ReadLoad((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadLoad(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadLoad = {
        .command = "servo-ReadLoad",
        .help = "return ReadLoad",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadLoad,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadLoad) );
}

static int servo_cmd_ReadVoltage(int argc, char **argv)
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
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("ReadVoltage: %d\r\n", servo.ReadVoltage((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadVoltage(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadVoltage = {
        .command = "servo-ReadVoltage",
        .help = "return ReadVoltage",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadVoltage,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadVoltage) );
}

static int servo_cmd_ReadTemper(int argc, char **argv)
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
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("ReadTemper: %d\r\n", servo.ReadTemper((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadTemper(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadTemper = {
        .command = "servo-ReadTemper",
        .help = "return ReadTemper",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadTemper,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadTemper) );
}

static int servo_cmd_ReadMove(int argc, char **argv)
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
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("ReadMove: %d\r\n", servo.ReadMove((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadMove(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadMove = {
        .command = "servo-ReadMove",
        .help = "return ReadMove",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadMove,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadMove) );
}

static int servo_cmd_ReadCurrent(int argc, char **argv)
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
    if( servo_id>12 ) {
        printf("Invalid servo ID\r\n");
        return 0;
    }
    printf("ReadCurrent: %d\r\n", servo.ReadCurrent((u8)servo_id));
    return 0;
}

static void register_servo_cmd_ReadCurrent(void)
{
    servo_id_args.servo_id = arg_int1(NULL, "id", "<n>", "Servo ID");
    servo_id_args.end = arg_end(2);
    const esp_console_cmd_t cmd_servo_ReadCurrent = {
        .command = "servo-ReadCurrent",
        .help = "return ReadCurrent",
        .hint = "--id <servoID>",
        .func = &servo_cmd_ReadCurrent,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_servo_ReadCurrent) );
}

void register_servo_cmds(void)
{
    register_servo_cmd_disable();
    register_servo_cmd_enable();
    register_servo_cmd_isEnabled();
    register_servo_cmd_scan();
    register_servo_cmd_rotate();
    register_servo_cmd_setStartPos();
    register_servo_cmd_setMidPos();
    register_servo_cmd_setEndPos();
    register_servo_cmd_setPosition();
    register_servo_cmd_setID();
    register_servo_cmd_FeedBack();
    register_servo_cmd_ReadPos();
    register_servo_cmd_ReadSpeed();
    register_servo_cmd_ReadLoad();
    register_servo_cmd_ReadVoltage();
    register_servo_cmd_ReadTemper();
    register_servo_cmd_ReadMove();
    register_servo_cmd_ReadCurrent();
}

extern "C" void app_main(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "muni_pupper>";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

    initialize_nvs();

#if CONFIG_CONSOLE_STORE_HISTORY
    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
    ESP_LOGI(TAG, "Command history enabled");
#else
    ESP_LOGI(TAG, "Command history disabled");
#endif

    /* Register commands */
    esp_console_register_help_command();
    register_servo_cmds();

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));

}
