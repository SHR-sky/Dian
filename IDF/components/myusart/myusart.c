#include <stdio.h>
#include <string.h>
#include "myusart.h"


void myusart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,            //APB时钟
    };

    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, BUF_SIZE*2, 0, 0, NULL, 0);   //主机不接收故为0
}



void myusart_test(void)
{
    char* test_str = "Hello world\r\n";
    while (1)
    {
        uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));
        vTaskDelay(1000);
    }
}

void myusart_print(char *rev)
{
    uart_write_bytes(uart_num, (const char*)rev, strlen(rev));
}