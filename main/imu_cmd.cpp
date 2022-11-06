#include "imu_cmd.h"
#include "QMI8658C.h"
#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_console.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

QMI8658C imu;

static int imu_cmd_init(int argc, char **argv)
{
    uint8_t err;
    err = imu.init();
    if(err) {
        printf("Init error: %d \r\n", err);
    }
    return 0;
}

static void register_imu_cmd_init(void)
{
    const esp_console_cmd_t cmd_imu_init = {
        .command = "imu-init",
        .help = "init the imu",
        .hint = NULL,
        .func = &imu_cmd_init,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_imu_init) );
}

static int imu_cmd_who_am_i(int argc, char **argv)
{
    uint8_t ret;
    ret = imu.who_am_i();
    printf("who_am_i: %d \r\n", ret);
    return 0;
}

static void register_imu_cmd_who_am_i(void)
{
    const esp_console_cmd_t cmd_imu_who_am_i = {
        .command = "imu-who_am_i",
        .help = "return who_am_i from the imu",
        .hint = NULL,
        .func = &imu_cmd_who_am_i,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_imu_who_am_i) );
}

static int imu_cmd_version(int argc, char **argv)
{
    uint8_t ret;
    ret = imu.version();
    printf("version: %d \r\n", ret);
    return 0;
}

static void register_imu_cmd_version(void)
{
    const esp_console_cmd_t cmd_imu_version = {
        .command = "imu-version",
        .help = "return version from the imu",
        .hint = NULL,
        .func = &imu_cmd_version,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_imu_version) );
}

static int imu_cmd_read_6dof(int argc, char **argv)
{
    uint8_t err;
    while(1) {
        err = imu.read_6dof();
        if(err) {
            printf("error: %d \r\n", err);
        }
        else {
            printf("%f\t%f\t%f\t%f\t%f\t%f \r\n", imu.acc.x, imu.acc.y, imu.acc.z, imu.gyro.x, imu.gyro.y, imu.gyro.z);
        }
	vTaskDelay(200 / portTICK_RATE_MS);
    }
    return 0;
}

static void register_imu_cmd_read_6dof(void)
{
    const esp_console_cmd_t cmd_imu_read_6dof = {
        .command = "imu-read_6dof",
        .help = "read 6dof the imu",
        .hint = "# You must boot at the end",
        .func = &imu_cmd_read_6dof,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_imu_read_6dof) );
}

static int imu_cmd_read_attitude(int argc, char **argv)
{
    uint8_t err;
    while(1) {
        err = imu.read_attitude();
        if(err) {
            printf("error: %d \r\n", err);
        }
        else {
            printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d \r\n", imu.dq.w, imu.dq.v.x, imu.dq.v.y, imu.dq.v.x, imu.dv.x, imu.dv.y, imu.dv.z, imu.ae_reg1, imu.ae_reg2);
        }
	vTaskDelay(200 / portTICK_RATE_MS);
    }
    return 0;
}

static void register_imu_cmd_read_attitude(void)
{
    const esp_console_cmd_t cmd_imu_read_attitude = {
        .command = "imu-read_attitude",
        .help = "read attitude the imu",
        .hint = "# You must boot at the end",
        .func = &imu_cmd_read_attitude,
	.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd_imu_read_attitude) );
}

void register_imu_cmds(void)
{
    register_imu_cmd_init();
    register_imu_cmd_who_am_i();
    register_imu_cmd_version();
    register_imu_cmd_read_6dof();
    register_imu_cmd_read_attitude();
}
