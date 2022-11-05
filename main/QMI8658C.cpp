#include "QMI8658C.h"

#define I2C_Freq 400000

#define SDA_1 41
#define SCL_1 42

#define I2C_DEV_ADDR (uint8_t)0x6B //0b1101010+1b (A0=GND)

/** registers */
#define QMI8658C_WHO_AM_I_REG               0x00 // ID in QMI8658C default to 0x05
#define QMI8658C_ACC_GYRO_CTRL1_SPI_REG     0x02
#define QMI8658C_ACC_GYRO_CTRL2_ACC_REG			0x03
#define QMI8658C_ACC_GYRO_CTRL3_G_REG			  0x04
#define QMI8658C_ACC_GYRO_CTRL4_M_REG			  0x05
#define QMI8658C_ACC_GYRO_CTRL5_REG			    0x06
#define QMI8658C_ACC_GYRO_CTRL6_AE_SET			0x07
#define QMI8658C_ACC_GYRO_CTRL7_REG			    0x08
#define QMI8658C_ACC_GYRO_CTRL9_REG			    0x0A
#define QMI8658C_ACC_GYRO_CAL1_L_REG			  0x0B
#define QMI8658C_ACC_GYRO_CTRL9_HOST_REG		0x10

#define QMI8658C_STATUSINT_REG		          0x2D
#define QMI8658C_STATUS0_REG		            0x2E


#define QMI8658C_ACC_GYRO_AE_REG1		        0x57
#define QMI8658C_ACC_GYRO_AE_REG2		        0x58

#define QMI8658C_ACC_GYRO_RESET		          0x60

#define QMI8658C_ACC_GYRO_OUT_L_TEMP_REG		0x33
#define QMI8658C_ACC_GYRO_OUT_H_TEMP_REG		0x34

#define QMI8658C_ACC_GYRO_OUTX_L_XL_REG			0x35
#define QMI8658C_ACC_GYRO_OUTX_H_XL_REG			0x36
#define QMI8658C_ACC_GYRO_OUTY_L_XL_REG			0x37
#define QMI8658C_ACC_GYRO_OUTY_H_XL_REG			0x38
#define QMI8658C_ACC_GYRO_OUTZ_L_XL_REG			0x39
#define QMI8658C_ACC_GYRO_OUTZ_H_XL_REG			0x3A

#define QMI8658C_ACC_GYRO_OUTX_L_G_REG			0x3B
#define QMI8658C_ACC_GYRO_OUTX_H_G_REG			0x3C
#define QMI8658C_ACC_GYRO_OUTY_L_G_REG			0x3D
#define QMI8658C_ACC_GYRO_OUTY_H_G_REG			0x3E
#define QMI8658C_ACC_GYRO_OUTZ_L_G_REG			0x3F
#define QMI8658C_ACC_GYRO_OUTZ_H_G_REG			0x40

#define QMI8658C_ACC_GYRO_OUTX_L_M_REG			0x41
#define QMI8658C_ACC_GYRO_OUTX_H_M_REG			0x42
#define QMI8658C_ACC_GYRO_OUTY_L_M_REG			0x43
#define QMI8658C_ACC_GYRO_OUTY_H_M_REG			0x44
#define QMI8658C_ACC_GYRO_OUTZ_L_M_REG			0x45
#define QMI8658C_ACC_GYRO_OUTZ_H_M_REG			0x46

#define QMI8658C_ACC_GYRO_OUTW_L_Q_REG			0x49
#define QMI8658C_ACC_GYRO_OUTW_H_Q_REG			0x4A
#define QMI8658C_ACC_GYRO_OUTX_L_Q_REG			0x4B
#define QMI8658C_ACC_GYRO_OUTX_H_Q_REG			0x4C
#define QMI8658C_ACC_GYRO_OUTY_L_Q_REG			0x4D
#define QMI8658C_ACC_GYRO_OUTY_H_Q_REG			0x4E
#define QMI8658C_ACC_GYRO_OUTZ_L_Q_REG			0x4F
#define QMI8658C_ACC_GYRO_OUTZ_H_Q_REG			0x50

#define QMI8658C_ACC_GYRO_OUTX_L_V_REG			0x51
#define QMI8658C_ACC_GYRO_OUTX_H_V_REG			0x52
#define QMI8658C_ACC_GYRO_OUTY_L_V_REG			0x53
#define QMI8658C_ACC_GYRO_OUTY_H_V_REG			0x54
#define QMI8658C_ACC_GYRO_OUTZ_L_V_REG			0x55
#define QMI8658C_ACC_GYRO_OUTZ_H_V_REG			0x56


QMI8658C::QMI8658C() : 
_I2C(TwoWire(1)) // I2C1
{

}

struct imu_configuration
{
  uint8_t reg;
  uint8_t value;
};

uint8_t QMI8658C::init()
{
  imu_configuration const config[] = {
    {QMI8658C_ACC_GYRO_CTRL1_SPI_REG, 0b01000000 }, // 0b01000000 address auto increment +  read data little endian + sensor enable
    {QMI8658C_ACC_GYRO_CTRL7_REG,     0b00000011 }, // 0b11001011 6D AE mode : enable gyro + enable acc
    {QMI8658C_ACC_GYRO_CTRL2_ACC_REG, 0b00000101 }, // 0b00000101 2g aODR = 235Hz
    {QMI8658C_ACC_GYRO_CTRL3_G_REG,   0b01110101 }  // 0b01110101 2048dps gODR = 235Hz
  };
  
  delay(150);
  _I2C.begin(SDA_1 , SCL_1 , I2C_Freq );

  for(size_t index=0; index < 4; ++index)
  {
    uint8_t error = write_byte(config[index].reg,config[index].value);
    if(error!=0) return index*10;
    delay(10);
    uint8_t value = 0;
    error = read_byte(config[index].reg,value);
    if(error!=1) return index*10+1; 
    if(value!=config[index].value) return index*10+2;
    delay(10);
  }
  return 0;
}

uint8_t QMI8658C::write_byte(uint8_t address, uint8_t data)
{
  _I2C.beginTransmission(I2C_DEV_ADDR);
  _I2C.write(address);
  _I2C.write(data);
  uint8_t const error = _I2C.endTransmission();
  return error;
}

uint8_t QMI8658C::read_byte(uint8_t address, uint8_t & data)
{
  _I2C.beginTransmission(I2C_DEV_ADDR);
  _I2C.write(address);
  uint8_t error = _I2C.endTransmission();
  uint8_t const bytesReceived = _I2C.requestFrom(I2C_DEV_ADDR, (uint8_t)1);
  if(bytesReceived==1)
    data = _I2C.read();
  return bytesReceived;
}

uint8_t QMI8658C::read_bytes(uint8_t address, uint8_t data[], uint8_t size)
{
  _I2C.beginTransmission(I2C_DEV_ADDR);
  _I2C.write(address);
  uint8_t const error = _I2C.endTransmission();
  uint8_t const bytesReceived = _I2C.requestFrom(I2C_DEV_ADDR, size);
  for(size_t index=0; index<bytesReceived;++index)
    data[index] = _I2C.read();
  return bytesReceived;
}

uint8_t QMI8658C::read_6dof()
{
  _I2C.beginTransmission(I2C_DEV_ADDR);
  _I2C.write(QMI8658C_ACC_GYRO_OUTX_L_XL_REG);
  uint8_t const error = _I2C.endTransmission();
  if(error!=0) return 5;
  uint8_t bytesReceived = _I2C.requestFrom(I2C_DEV_ADDR, (uint8_t)12);

  if(bytesReceived==12)
  {
    uint8_t raw[12];
    for(size_t index=0;index<12;++index)
      raw[index]=_I2C.read();
    
    acc.x = 1.0/16384.0*((int16_t)(raw[1]<<8) | raw[0]);
    acc.y = 1.0/16384.0*((int16_t)(raw[3]<<8) | raw[2]);
    acc.z = 1.0/16384.0*((int16_t)(raw[5]<<8) | raw[4]);
    gyro.x = 1.0/16.0* ((int16_t)(raw[7]<<8) | raw[6]);
    gyro.y = 1.0/16.0* ((int16_t)(raw[9]<<8) | raw[8]);
    gyro.z = 1.0/16.0* ((int16_t)(raw[11]<<8) | raw[10]);
    return 0;
  }
  else
  {
    acc.x = 0.0f;
    acc.y = 0.0f;
    acc.z = 0.0f;
    gyro.x = 0.0f;
    gyro.y = 0.0f;
    gyro.z = 0.0f;
    return 6;
  }
  return 0;
}

uint8_t QMI8658C::read_attitude()
{
  _I2C.beginTransmission(I2C_DEV_ADDR);
  _I2C.write(QMI8658C_ACC_GYRO_OUTW_L_Q_REG);
  uint8_t const error = _I2C.endTransmission();
  if(error!=0) return 5;
  uint8_t bytesReceived = _I2C.requestFrom(I2C_DEV_ADDR, (uint8_t)16);

  if(bytesReceived==16)
  {
    uint8_t raw[16];
    for(size_t index=0;index<16;++index)
      raw[index]=_I2C.read();
    
    dq.w = 1.0/16384.0*((int16_t)(raw[1]<<8) | raw[0]);
    dq.v.x = 1.0/16384.0*((int16_t)(raw[3]<<8) | raw[2]);
    dq.v.y = 1.0/16384.0*((int16_t)(raw[5]<<8) | raw[4]);
    dq.v.z = 1.0/16384.0* ((int16_t)(raw[7]<<8) | raw[6]);
    dv.x = 1.0/1024.0* ((int16_t)(raw[9]<<8) | raw[8]);
    dv.y = 1.0/1024.0* ((int16_t)(raw[11]<<8) | raw[10]);
    dv.z = 1.0/1024.0* ((int16_t)(raw[13]<<8) | raw[12]);
    ae_reg1 = raw[14];
    ae_reg2 = raw[15];
    return 0;
  }
  else
  {
    dq.w = 0.0f;
    dq.v.x = 0.0f;
    dq.v.y = 0.0f;
    dq.v.z = 0.0f;
    dv.x = 0.0f;
    dv.y = 0.0f;
    dv.z = 0.0f;
    ae_reg1 = 0;
    ae_reg2 = 0;    
    return 6;
  }
  return 0;
}

uint8_t QMI8658C::who_am_i()
{
  uint8_t value = 0;
  read_byte(QMI8658C_WHO_AM_I_REG,value);
  return value;
}

uint8_t QMI8658C::version()
{
  uint8_t value = 0;
  read_byte(QMI8658C_WHO_AM_I_REG+1,value);
  return value;
}

