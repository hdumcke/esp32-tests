#include "Wire.h"
#include "vector_type.h"
#include "quaternion_type.h"

struct QMI8658C
{
  QMI8658C();

  uint8_t init();

  uint8_t who_am_i();
  uint8_t version();

  uint8_t read_6dof();
  uint8_t read_attitude();

  vec3_t acc;
  vec3_t gyro;
  quat_t dq;
  vec3_t dv;
  uint8_t ae_reg1;
  uint8_t ae_reg2;

protected:

  uint8_t write_byte(uint8_t address, uint8_t data);
  uint8_t read_byte(uint8_t address, uint8_t & data);
  uint8_t read_bytes(uint8_t address, uint8_t data[], uint8_t size);

private:
  
  TwoWire _I2C;
};

