/* Authors : 
 * - Hdumcke
 * - Pat92fr
 */

#ifndef _mini_pupper_host_H
#define _mini_pupper_host_H

#include "mini_pupper_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* IMPORTANT : Mini Pupper Host API requires to setup FreeRTOS frequency at 1000Hz.
 *             Use IDF ESP32 : MENUCONFIG > COMPONENTS > FREERTOS > KERNEL > 1000Hz
 *
 */

#include "mini_pupper_host_base.h"

void HOST_TASK(void * parameters);

struct HOST
{
    HOST();

    void start();

private:
    int _uart_port_num {2};
    
    // background host serial bus service
    TaskHandle_t _task_handle {NULL};    
    friend void HOST_TASK(void * parameters);
};

extern HOST host; 

#endif //_mini_pupper_host_H
