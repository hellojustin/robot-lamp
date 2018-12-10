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
#define SERVOMIN 150
#endif

#ifndef SERVOMAX
#define SERVOMAX 600
#endif

uint8_t servonum = 0;

void setup()
{
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);
}

void loop()
{
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  delay(500);

  servonum ++;
  if (servonum > 4) servonum = 0;
}
