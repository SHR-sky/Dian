#include <Arduino.h>
#include "LED.h"

#define LED 2

void setup() {
  LED_Init(2);
}

void loop() {
  LED_ON(LED);
}

// put function definitions here:
