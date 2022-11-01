#include "SCSCL.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class SERVO : public SCSCL
{
public:
    void TestBroadcast() {
        int i;
        this->EnableTorque(0xfe,1);
        vTaskDelay(500 / portTICK_RATE_MS);
        this->WritePos(0xfe, 0, 1000);
        vTaskDelay(1000 / portTICK_RATE_MS);
    
        for(i = 0; i<1024; i++)
        {
          this->WritePos(0xfe,i,20);
          vTaskDelay(20 / portTICK_RATE_MS);
        }
        for(i = 1023; i > 0; i--)
        {
          this->WritePos(0xfe,i,10);
          vTaskDelay(10 / portTICK_RATE_MS);
        }
    }
};

extern "C" void app_main(void)
{

    printf("Hello world!\n");
    SERVO servo;
    servo.uart_port_num = 2;

    servo.TestBroadcast();

}
