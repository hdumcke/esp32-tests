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

#include "vector_type.h"
#include "quaternion_type.h"

#include "mini_pupper_stats.h"

void IMU_TASK(void * parameters);
void IRAM_ATTR IMU_ISR(void * arg);

struct IMU
{
  IMU();

  uint8_t init();

  void start();

  float get_roll() const;
  float get_pitch() const;
  float get_yaw() const;

  /* DEBUG */

  uint8_t who_am_i();
  uint8_t version();

  uint8_t read_6dof();
  uint8_t read_attitude();

  /* DEBUG */

  vec3_t acc;
  vec3_t gyro;
  quat_t dq;
  vec3_t dv;
  uint8_t ae_reg1;
  uint8_t ae_reg2;

  // public stats
  mini_pupper::periodic_process_monitor monitor;

private:

  float _roll_deg {0.0f};
  float _pitch_deg {0.0f};
  float _yaw_deg {0.0f};

  static float roll_adjust(float roll_deg);
  static float compute_roll(quat_t dq);
  static float compute_pitch(quat_t dq);

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
