#include <stdio.h>
#include "led.h"


#define LED_PIN 2

void led_init(void)
{
    gpio_config_t led_pin_config;
    led_pin_config.pin_bit_mask = 1<<LED_PIN;
    led_pin_config.mode = GPIO_MODE_OUTPUT;
    led_pin_config.pull_up_en = GPIO_PULLUP_DISABLE;
    led_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&led_pin_config);
}

void led_on(void)
{    
    gpio_set_level(LED_PIN, 1);
}

void led_off(void)
{
    gpio_set_level(LED_PIN, 0);
}
