#include "EZServo.h"

EZServo::EZServo(int iport, int iminValue, int imaxValue) : minValue(iminValue), maxValue(imaxValue) {
  servo.attach(iport);
}

void EZServo::write(double iValue){
  iValue = min(max(iValue, -1.0), 1.0);
  int deg = map((int)(100 * iValue), -100, 100, minValue, maxValue);
  servo.write(deg);
}

void EZServo::setMinValue(int iminValue){
  minValue = iminValue;
}

void EZServo::setMaxValue(int imaxValue){
  maxValue = imaxValue;
}
