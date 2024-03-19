#include <stdio.h>
#include <string.h>


#define IDF_mode 1

float Yaw,Pitch,Roll;

#if (IDF_mode == 0)

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

#endif

#if (IDF_mode == 1)

void app_main()
{
    myusart_init();
    myusart_test();
}

#endif



#if (IDF_mode == 2)

void app_main()
{
    char message[1000]="";
    float uint[6];

    myusart_init();
    myiic_Init();

    int16_t ax,ay,az,gx,gy,gz;
    mpu6050_get_six(&gx, &gy, &gz, &ax, &ay, &az);
    change_unit(&gx, &gy, &gz, &ax, &ay, &az, uint);
    sprintf(message,"gx:%f, gy:%f, gz:%f, ax:%f, ay:%f, az:%f \n",uint[0],uint[1],uint[2],uint[3],uint[4],uint[5]); 
    myusart_print(message);
    mpu6050_get_data();
    sprintf(message,"yaw:%d, roll:%d, pitch:%d \n",Yaw,Roll,Pitch);
    myusart_print(message);
}


#endif