#include "LED.h"
#include <Arduino.h>

/// @brief 
/// @param pin 
void LED_Init(int pin)
{
    pinMode(pin,OUTPUT);
}

void LED_ON(int pin)
{
    digitalWrite(pin,HIGH);
    delay(200);
}
