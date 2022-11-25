#include "PID.h"

PID::PID(double ikP, double ikI, double ikD) : kP(ikP), kI(ikI), kD(ikD) {
  reset();
}

double PID::step(double iError, double idT) {
  error = iError, dT = idT;
  derivative = (error - prevError) / dT;
  integral += error * dT;
  if((error > 0 && prevError < 0) || (error < 0 && prevError > 0)){
    integral = 0;
  }
  prevError = error;

  if(integral > 255/kI){
    integral = 255/kI;
  }
  else if(integral < -255/kI){
    integral = -255/kI;
  }
  output = error * kP + integral * kI + derivative * kD;


  
  if(output > 255){
    output = 255;
  }
  else if(output < -255){
    output = -255;
  }
  return output;
}

double PID::getOutput() const{
  return output;
}

double PID::getIntegral() const{
  return integral * kI;
}

void PID::reset() {
  error = 0;
  derivative = 0;
  integral = 0;
  prevError = 0;
  output = 0;
  dT = 0;
}
