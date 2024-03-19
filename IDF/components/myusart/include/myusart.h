#ifndef __MYUSART_H
#define __MYUSART_H

#include "FreeRTOS/freertos.h"
#include "FreeRTOS/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

//缓冲区
#define BUF_SIZE 1024

//使用uart0
const uart_port_t uart_num = UART_NUM_0;  

//定义TX与RX引脚，查询手册可得串口0的对应引脚
#define TXD_PIN 43
#define RXD_PIN 44


void myusart_init(void);
void myusart_test(void);
void myusart_print(char *rev);


#endif


