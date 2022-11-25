#include "arduino.h"
#include <analogWrite.h>

#include<cmath>

#ifndef _DCMotor_H
#define _DCMotor_H

class DCMotor{
  public:
  DCMotor(int ipin1, int ipin2, int ipwmPin, bool ireversed = false);

  void setPower(int ipower);

  void stop();

  void setMaxPower(int imaxPower);

  void setMinPower(int iminPower);

  private:
  int pin1, pin2, pwmPin;
  bool isReversed = false;
  int minPower = -255;
  int maxPower = 255;
};

#endif
