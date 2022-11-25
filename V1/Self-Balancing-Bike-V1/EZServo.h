#include "arduino.h"
#include <analogWrite.h>
#include <ESP32Servo.h>
#include<cmath>

#ifndef _EZServo_H
#define _EZServo_H

class EZServo{
  public:
  EZServo(int iport, int iminValue = 0, int imaxValue = 180);

  void write(double iValue);

  void setMinValue(int iminValue);

  void setMaxValue(int imaxValue);

  private:
  Servo servo;
  int minValue, maxValue;
};

#endif
