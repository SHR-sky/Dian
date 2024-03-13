#include <stdio.h>
#include "led.h"
#include "FreeRTOS/freertos.h"
#include "FreeRTOS/task.h"


void app_main(void)
{
    while(1)
    {
        led_init();
        led_on();
        vTaskDelay(50);
        led_off();
        vTaskDelay(50);
    }
    
}
