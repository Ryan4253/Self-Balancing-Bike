#include "DCMotor.h"

DCMotor::DCMotor(int ipin1, int ipin2, int ipwmPin, bool ireversed) : pin1(ipin1), pin2(ipin2), pwmPin(ipwmPin), isReversed(ireversed){
  pinMode(ipin1, OUTPUT);
  pinMode(ipin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);
}

void DCMotor::setPower(int ipower){
  ipower = std::min(std::max(ipower, minPower), maxPower);

  if(ipower == 0){
    stop();
    return;
  }
  
  if(!isReversed && ipower > 0 || isReversed && ipower < 0){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pwmPin, abs(ipower));
  }
  else{
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(pwmPin, abs(ipower));
  }
}

void DCMotor::stop(){
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  
}

void DCMotor::setMaxPower(int imaxPower){
  if(imaxPower > 255){
    return;
  }
  
  maxPower = imaxPower;
}

void DCMotor::setMinPower(int iminPower){
  if(iminPower < -255){
    return;
  }

  minPower = iminPower;
}
