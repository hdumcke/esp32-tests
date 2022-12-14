/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

#ifndef _mini_pupper_imu_h
#define _mini_pupper_imu_h

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_timer.h>

#include <stdint.h>

#include "mini_pupper_stats.h"

void IMU_TASK(void * parameters);
void IRAM_ATTR IMU_ISR(void * arg);

struct IMU
{
  IMU();

  uint8_t init();

  void start();

  /* DEBUG */

  uint8_t who_am_i();
  uint8_t version();
  uint8_t read_6dof();

  /* DEBUG */

  float ax, ay, az;
  float gx, gy, gz;
  
  // public stats
    mini_pupper::periodic_process_monitor p_monitor;
    mini_pupper::frame_error_rate_monitor f_monitor;

private:

  // I2C bus helpers
  uint8_t write_byte(uint8_t reg_addr, uint8_t data);
  uint8_t read_byte(uint8_t reg_addr, uint8_t *data);
  uint8_t read_bytes(uint8_t reg_addr, uint8_t data[], uint8_t size);

  // background host serial bus service
  TaskHandle_t _task_handle {NULL};    
  QueueHandle_t _INT2_evt_queue {NULL};
  friend void IMU_TASK(void * parameters);
  friend void IMU_ISR(void * arg);
};

extern IMU imu;

#endif //  _mini_pupper_imu_h
