/*
  Code for library for a motor.
*/
#ifndef Rover_h
#define Rover_h

#include <Arduino.h>

class RoverMotor
{
  public:
    RoverMotor(int pwm, int dir1, int dir2, int ENCA, int ENCB, float kp,float ki);
    void setMotor(int dir, int pwmVal);
    void readEcoder();
    
    int pos_i;
    volatile long prevT_i;
    volatile float velocity_i;
  private:
    int pwmPin;
    int dir1Pin;
    int dir2Pin;

    int ENCAPin;
    int ENCBPin;

    
};

#endif
