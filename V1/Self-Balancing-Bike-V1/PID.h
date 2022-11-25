#ifndef _PID_H
#define _PID_H

class PID{
  public:

  PID(double ikP, double ikI, double ikD);

  double step(double iError, double idT = 0.01);

  double getOutput() const;

  void reset();

  double getIntegral() const;

  private:
  double kP, kI, kD;
  double error, derivative, integral, prevError, dT, output; 
};


#endif;
