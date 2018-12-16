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

#define JOY0XIN A0
#define JOY0YIN A1
#define JOY1XIN A2
#define JOY1YIN A3
#define DRAG    100

#define SERVO0 0
#define SERVO1 1
#define SERVO2 2
#define SERVO2 3
#define SERVO2 4

int16_t joy0x  = 0;
int16_t joy0x0 = 0;
int16_t joy0y  = 0;
int16_t joy0y0 = 0;
int16_t joy0z  = 0;
int16_t joy1x  = 0;
int16_t joy1x0 = 0;
int16_t joy1y  = 0;
int16_t joy1y0 = 0;
int16_t joy1z  = 0;
int16_t servo0pos  = (SERVOMAX - SERVOMIN) / 2;
int16_t servo1pos  = (SERVOMAX - SERVOMIN) / 2;
int16_t servo2pos  = (SERVOMAX - SERVOMIN) / 2;
int16_t servo3pos  = (SERVOMAX - SERVOMIN) / 2;
int16_t servo4pos  = (SERVOMAX - SERVOMIN) / 2;

void setup()
{
  Serial.begin(9600);
  Serial.println("Servo speed control test!");
  pwm.begin();
  pwm.setPWMFreq(60);

  analogReference(EXTERNAL);
  pinMode(JOY0XIN, INPUT_PULLUP);
  pinMode(JOY0YIN, INPUT_PULLUP);
  pinMode(JOY1XIN, INPUT_PULLUP);
  pinMode(JOY1YIN, INPUT_PULLUP);
  delay(10);
  joy0x0 = analogRead(JOY0XIN);
  joy0y0 = analogRead(JOY0YIN);
  joy1x0 = analogRead(JOY1XIN);
  joy1y0 = analogRead(JOY1YIN);

  delay(10);
}

void loop()
{
  joy0x     = (analogRead(JOY0XIN) - joy0x0) / DRAG;
  servo1pos = max(SERVOMIN, min(SERVOMAX, servo1pos + joy0x));
  pwm.setPWM(SERVO1, 0, servo1pos);

  joy0y     = (analogRead(JOY0YIN) - joy0y0) / DRAG;
  servo2pos = max(SERVOMIN, min(SERVOMAX, servo2pos + joy0y));
  pwm.setPWM(SERVO2, 0, servo2pos);

  joy1x     = (analogRead(JOY1XIN) - joy1x0) / DRAG;
  servo4pos = max(SERVOMIN, min(SERVOMAX, servo4pos + joy1x));
  pwm.setPWM(SERVO4, 0, servo4pos);

  joy1y     = (analogRead(JOY1YIN) - joy1y0) / DRAG;
  servo3pos = max(SERVOMIN, min(SERVOMAX, servo3pos + joy1y));
  pwm.setPWM(SERVO3, 0, servo3pos);

  delay(5);
}
