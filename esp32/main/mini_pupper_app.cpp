/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>
#include <string.h>

#include "esp_system.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "mini_pupper_cmd.h"

#include "mini_pupper_servos.h"
#include "mini_pupper_host.h"
#include "mini_pupper_imu.h"
#include "mini_pupper_power.h"


static const char* TAG = "MAIN";

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

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

extern "C" void app_main(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "muni_pupper>";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

    initialize_nvs();

    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
    ESP_LOGI(TAG, "Command history enabled");

    /* Register commands */
    esp_console_register_help_command();
    register_mini_pupper_cmds();

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
    // init IMU device
    uint8_t const imu_status = imu.init();
    if(imu_status==0)
        ESP_LOGI(TAG, "IMU device init [OK].");
    else
        ESP_LOGI(TAG, "IMU device configuration [FAILURE] (error:%d)!",imu_status);

    // start IMU service
    imu.start();
    ESP_LOGI(TAG, "IMU service started.");

    // start POWER service
    POWER::start();
    ESP_LOGI(TAG, "Power service started.");

    // start CLI
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    // enable SERVO power supply
    servo.enable();
    ESP_LOGI(TAG, "Servo power supply enabled.");
    servo.soft_start();
    ESP_LOGI(TAG, "Servo in neutral position.");

    // start SERVO interface
    servo.start();
    ESP_LOGI(TAG, "Servo control & feedback service started.");

    // start HOST interface
    host.start();
    ESP_LOGI(TAG, "Host communication service started.");

    int const low_voltage_cutoff_counter_max {60}; // 1 minute
    int low_voltage_cutoff_counter {low_voltage_cutoff_counter_max}; 
    for(;;)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGD(TAG, "heart-beat");


        // low-voltage cutoff
        if(POWER::get_voltage_V()<6.2 && low_voltage_cutoff_counter>0)
        {
            --low_voltage_cutoff_counter;
            ESP_LOGI(TAG, "Warning : Low-voltage!");            
        }
        if(POWER::get_voltage_V()>6.6 && low_voltage_cutoff_counter!=0 && low_voltage_cutoff_counter<low_voltage_cutoff_counter_max)
        {
            ++low_voltage_cutoff_counter;
        }
        if(low_voltage_cutoff_counter==0)
        {
            servo.disable();
            ESP_LOGI(TAG, "Servo power supply disabled (low-voltage cutoff)!");            
        }
    }
}
