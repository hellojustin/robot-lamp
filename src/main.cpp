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

#define LEFTRIGHT A2
#define UPDOWN    A3
#define DRAG      100

int16_t leftright  = 0;
int16_t lrzero     = 0;
int16_t updown     = 0;
int16_t udzero     = 0;
uint8_t servo0num  = 0;
uint8_t servo1num  = 1;
int16_t servo0pos  = (SERVOMAX - SERVOMIN) / 2;
int16_t servo1pos  = (SERVOMAX - SERVOMIN) / 2;

void setup()
{
  Serial.begin(9600);
  Serial.println("Servo speed control test!");
  pwm.begin();
  pwm.setPWMFreq(60);

  analogReference(EXTERNAL);
  pinMode(LEFTRIGHT, INPUT_PULLUP);
  pinMode(UPDOWN,    INPUT_PULLUP);
  delay(10);
  lrzero = analogRead(LEFTRIGHT);
  udzero = analogRead(UPDOWN);

  delay(10);
}

void loop()
{
  updown    = (analogRead(UPDOWN) - udzero) / DRAG;
  servo0pos = max(SERVOMIN, min(SERVOMAX, servo0pos + updown));
  pwm.setPWM(servo0num, 0, servo0pos);

  leftright = (analogRead(LEFTRIGHT) - lrzero) / DRAG;
  servo1pos = max(SERVOMIN, min(SERVOMAX, servo1pos + leftright));
  pwm.setPWM(servo1num, 0, servo1pos);

  delay(5);


  // Serial.print("leftright: "); Serial.println(leftright);
  // Serial.print("updown:    "); Serial.println(updown);
  //
  // delay(1000);


  // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
  //   Serial.print("Pulse Length: "); Serial.println(pulselen);
  //   delay(10);
  //   pwm.setPWM(servonum, 0, pulselen);
  // }
  // pwm.setPWM(servonum, 0, SERVOMIN);
}
