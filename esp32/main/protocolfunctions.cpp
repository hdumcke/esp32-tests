#include "mini_pupper_servos.h"
#include "protocolfunctions.h"
#include "imu_cmd.h"
#include <cstddef>
#include <cstring>
#include <cstdio>
#include "esp_log.h"

static const char *TAG = "PROTOCOLFUNCTIONS";

PROTOCOL_STAT sUSART2;

uint8_t data[2];
struct SERVOPARAM {
    u16 param[12];
};
SERVOPARAM servo_data;
struct IMU6DOFPARAM {
    vec3_t acc;
    vec3_t gyro;
};
IMU6DOFPARAM imu_6dof_data;
struct IMUATTPARAM {
    quat_t dq;
    vec3_t dv;
    uint8_t ae_reg1;
    uint8_t ae_reg2;
};
IMUATTPARAM imu_att_data;
bool isEnabled;

void fn_servo_enable ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
	    servo.enable();
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_disable ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
	    servo.disable();
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_torque_enable ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
	    servo.enable_torque();
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_torque_disable ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
	    servo.disable_torque();
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_is_enabled ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            isEnabled = servo.isEnabled;
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_is_torque_enabled ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {

 // TODO : pass ID as argument ==> Hdumcke
  // TODO : pass ID as argument ==> Hdumcke
   // TODO : pass ID as argument ==> Hdumcke

    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            u8 ID {1};
            u8 enable {0};
            servo.is_torque_enable(ID,enable);
            isEnabled = (enable == 1);
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_set_position ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    static u16 servoPositions[12] {0};
    fn_defaultProcessing(s, param, cmd, msg);
    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
	    if( msg->lenPayload == 4 )
	    {
                //ESP_LOGI(TAG, "Position: %u %d", i+1, ((SERVOPARAM*) (param->ptr))->param[i]);
                servo.setPositionAsync(((SERVOPARAM*) (param->ptr))->param[0], ((SERVOPARAM*) (param->ptr))->param[1]);
	    }
	    else if(msg->lenPayload == sizeof(servo_data))
	    {
                for(u8 i = 0; i<12; i++)
                {
		              servoPositions[i] = ((SERVOPARAM*) (param->ptr))->param[i];
                }
                servo.setPosition12Async(servoPositions);
	    }
	    else
	    {
                ESP_LOGE(TAG, "Invalid parameter lenght received: %d", msg->lenPayload);
	    }

            break;
    }
}

void fn_servo_get_position ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            for(u8 i = 0; i<12; i++)
	        {
             servo_data.param[i] = servo.getPositionAsync(i+1);
              //ESP_LOGI(TAG, "Position: %u %d", i+1, servo_data.param[i]);
            }
            //ESP_LOG_BUFFER_HEX(TAG, &servo_data, sizeof(servo_data));
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_get_speed ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            for(u8 i = 0; i<12; i++)
	    {
                servo_data.param[i] = servo.getSpeedAsync(i+1);
	    }
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_get_load ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            for(u8 i = 0; i<12; i++)
	    {
                servo_data.param[i] = servo.getLoadAsync(i+1);
	    }
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_get_voltage ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            for(u8 i = 0; i<12; i++)
	    {
                servo_data.param[i] = servo.getVoltageAsync(i+1);
	    }
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_get_temperature ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            for(u8 i = 0; i<12; i++)
	    {
                servo_data.param[i] = servo.getTemperatureAsync(i+1);
	    }
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_get_move ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            for(u8 i = 0; i<12; i++)
	    {
                servo_data.param[i] = servo.getMoveAsync(i+1);
	    }
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_get_current ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            for(u8 i = 0; i<12; i++)
	    {
                servo_data.param[i] = servo.getCurrentAsync(i+1);
	    }
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_servo_ping ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            for(u8 i = 0; i<12; i++)
	    {
                servo_data.param[i] = 0;
                if(servo.ping(i+1) == i+1) {
                    servo_data.param[i] = i+1;
                    //ESP_LOGI(TAG, "Ping: %u", i+1);
                }
	    }
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_imu_get_6dof ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            imu.read_6dof();
	    memcpy(&imu_6dof_data.acc, &imu.acc, sizeof(imu.acc));
	    memcpy(&imu_6dof_data.gyro, &imu.gyro, sizeof(imu.gyro));
            //ESP_LOG_BUFFER_HEX(TAG, &imu_6dof_data, sizeof(imu_6dof_data));
	    //printf("%f\t%f\t%f\t%f\t%f\t%f \r\n", imu.acc.x, imu.acc.y, imu.acc.z, imu.gyro.x, imu.gyro.y, imu.gyro.z);
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

void fn_imu_get_attitude ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
            imu.read_attitude();
	    memcpy(&imu_att_data.dq, &imu.dq, sizeof(imu.dq));
	    memcpy(&imu_att_data.dv, &imu.dv, sizeof(imu.dv));
            imu_att_data.ae_reg1 = imu.ae_reg1;
            imu_att_data.ae_reg2 = imu.ae_reg2;
            //ESP_LOG_BUFFER_HEX(TAG, &imu_att_data, sizeof(imu_att_data));
            //printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d \r\n", imu.dq.w, imu.dq.v.x, imu.dq.v.y, imu.dq.v.x, imu.dv.x, imu.dv.y, imu.dv.z, imu.ae_reg1, imu.ae_reg2);
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
int setup_protocol(PROTOCOL_STAT *s) {

    int errors = 0;

    errors += protocol_init(&sUSART2);

    //sUSART2.send_serial_data=USART2_IT_send;
    //sUSART2.send_serial_data_wait=USART2_IT_send;
    sUSART2.timeout1 = 500;
    sUSART2.timeout2 = 100;
    sUSART2.allow_ascii = 0;

    errors += setParamVariable( s, 0x70, UI_NONE, data, sizeof(data) );
    setParamHandler( s, 0x70, fn_servo_enable );

    errors += setParamVariable( s, 0x71, UI_NONE, data, sizeof(data) );
    setParamHandler( s, 0x71, fn_servo_disable );

    errors += setParamVariable( s, 0x72, UI_NONE, data, sizeof(data) );
    setParamHandler( s, 0x72, fn_servo_torque_enable );

    errors += setParamVariable( s, 0x73, UI_NONE, data, sizeof(data) );
    setParamHandler( s, 0x73, fn_servo_torque_disable );

    errors += setParamVariable( s, 0x74, UI_NONE, &isEnabled, sizeof(isEnabled) );
    setParamHandler( s, 0x74, fn_servo_is_enabled );

    errors += setParamVariable( s, 0x75, UI_NONE, &isEnabled, sizeof(isEnabled) );
    setParamHandler( s, 0x75, fn_servo_is_torque_enabled );

    errors += setParamVariable( s, 0x76, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x76, fn_servo_set_position );

    errors += setParamVariable( s, 0x77, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x77, fn_servo_get_position );

/* TODO : remove this from Linux API
    errors += setParamVariable( s, 0x78, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x78, fn_servo_get_feedback );
*/
    errors += setParamVariable( s, 0x79, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x79, fn_servo_get_speed );

    errors += setParamVariable( s, 0x7A, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x7A, fn_servo_get_load );

    errors += setParamVariable( s, 0x7B, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x7B, fn_servo_get_voltage );

    errors += setParamVariable( s, 0x7C, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x7C, fn_servo_get_temperature );

    errors += setParamVariable( s, 0x7D, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x7D, fn_servo_get_move );

    errors += setParamVariable( s, 0x7E, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x7E, fn_servo_get_current );

    errors += setParamVariable( s, 0x7F, UI_NONE, (void*)&servo_data, sizeof(servo_data) );
    setParamHandler( s, 0x7F, fn_servo_ping );

    errors += setParamVariable( s, 0x60, UI_NONE, (void*)&imu_6dof_data, sizeof(imu_6dof_data) );
    setParamHandler( s, 0x60, fn_imu_get_6dof );

    errors += setParamVariable( s, 0x61, UI_NONE, (void*)&imu_att_data, sizeof(imu_att_data) );
    setParamHandler( s, 0x61, fn_imu_get_attitude );

    return errors;
}
