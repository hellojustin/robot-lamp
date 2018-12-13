/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#ifndef SERVOMIN
#define SERVOMIN 110
#endif

#ifndef SERVOMAX
#define SERVOMAX 570
#endif

uint8_t servonum = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("Servo speed control test!");
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);
}

void loop()
{
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    Serial.print("Pulse Length: "); Serial.println(pulselen);
    delay(10);
    pwm.setPWM(servonum, 0, pulselen);
  }
  pwm.setPWM(servonum, 0, SERVOMIN);
  // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
  //   pwm.setPWM(servonum, 0, pulselen);
  // }
  delay(500);
}
