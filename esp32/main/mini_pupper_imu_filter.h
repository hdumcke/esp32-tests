#include "quaternion_type.h"

#ifndef _mini_pupper_imu_filter_H
#define _mini_pupper_imu_filter_H

//------------------ Coefficients -------------------- 

#define INV_Q_FACTOR        2           // Filter damping. A smaller value leads to faster response but more oscillations.
#define DEFAULT_GAIN        0.1         // Default filter gain. A faster value 

//---------------- Class definition ------------------ 
                         
class IMU_FILTER
{
  private: 
    vec3_t s;
    quat_t q;
    float last_time_s;
      
  public:
    // Initialization:
    void setup(float current_time_s);
    void setup(float current_time_s, float ax, float ay, float az);

    // Heading estimate:
    void update(float current_time_s, float gx, float gy, float gz);
    void update(float current_time_s, float gx, float gy, float gz, float ax, float ay, float az, float alpha=DEFAULT_GAIN, const bool SCALE_GAIN =true );
                 
    void rotateHeading(float angle, bool const SMALL_ANG);

    //-- Fusion outputs:
    
    // Quaternion
    quat_t getQuat();

    // Axis projections:
    vec3_t getXaxis( const bool );
    vec3_t getYaxis( const bool );
    vec3_t getZaxis( const bool );
    vec3_t projectVector( vec3_t, const bool );
    
    // Euler Angles:
    float roll();
    float pitch();
    float yaw();
};

#endif //_mini_pupper_imu_filter_H
