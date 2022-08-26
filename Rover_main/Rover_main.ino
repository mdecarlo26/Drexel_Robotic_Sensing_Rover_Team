#include <Rover.h>
#include <Arduino.h>

//RoverMotor(int pwm, int dir1, int dir2, int ENCA, int ENCB, float kp,float ki);
RoverMotor motor1=RoverMotor(12,34,35,18,31,3,22);
RoverMotor motor2=RoverMotor(8,37,36,19,38,3,22);
RoverMotor motor3=RoverMotor(9,43,42,3,49,3,22);
// RoverMotor motor4=RoverMotor(5,26,27,2,23,3,22);
void setup(){
  //insert 
  Serial.begin(9600);
    
  
}

void loop() {
  motor1.setMotor(1, 100);
  motor1.readEncoder();
  Serial.println(motor1.velocity_i);
  // Set the motor speed and direction
}
