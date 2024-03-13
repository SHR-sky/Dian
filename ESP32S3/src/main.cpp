#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 

MPU6050 accelgyro;
 
int16_t ax, ay, az;
int16_t gx, gy, gz;
 
 
 

#define OUTPUT_READABLE_ACCELGYRO



#define LED_PIN 13
bool blinkState = false;
 
void setup() {

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(21,47,10000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
 
    Serial.begin(115200);
 
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
 
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    pinMode(LED_PIN, OUTPUT);
}
 
void loop() {

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO

        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif
 
    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
 
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(100);
}




/*
#include <Arduino.h>
#include <HardwareSerial.h>
#include "LED.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define LED 2
String rev="hello_world";

void setup() {
  Serial.begin(115200);
  
}

void loop() {




}

// put function definitions here:
*/