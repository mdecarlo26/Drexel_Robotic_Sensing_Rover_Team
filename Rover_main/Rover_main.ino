//#include <Rover.h>
#include <Arduino.h>

class RoverMotor
{
  public:
    RoverMotor(int pwm, int dir1, int dir2, int ENCA, int ENCB, float kp,float ki);
    void setMotor(int dir, int pwmVal);
    void readEncoder();
  private:
    int pwmPin;
    int dir1Pin;
    int dir2Pin;
    int ENCAPin;
    int ENCBPin;
    
    float kiMotor;
    float kpMotor;
      
    volatile int pos_i=0;
    volatile float velocity_i = 0;
    volatile long prevT_i=0;

    float v1Filt = 0;
    float v1Prev = 0;
    float v2Filt = 0;
    float v2Prev = 0;
    
    float eintegral = 0;
    
};

RoverMotor::RoverMotor(int pwm, int dir1, int dir2, int ENCA, int ENCB, float kp,float ki){
  pwmPin = pwm;
  dir1Pin = dir1;
  dir2Pin = dir2;

  ENCAPin = ENCA;
  ENCBPin = ENCB;

  kpMotor = kp;
  kiMotor = ki;
  
  pinMode(pwmPin,OUTPUT);
  pinMode(dir1Pin,OUTPUT);
  pinMode(dir2Pin,OUTPUT);
  
  };

void RoverMotor::setMotor(int dir, int pwmVal) {
  // sets speed and direction of motor
  analogWrite(pwmPin,pwmVal);//Motor Speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(dir1Pin,HIGH);
    digitalWrite(dir2Pin,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(dir1Pin,LOW);
    digitalWrite(dir2Pin,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(dir1Pin,LOW);
    digitalWrite(dir2Pin,LOW);    
  }
};

void RoverMotor::readEncoder() {
  // Read encoder B when ENCA rises
  int B = digitalRead(ENCBPin);
  int increment = 0;
  if(B>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = millis();
  float deltaT = ((float) (currT - prevT_i))/1.0e3;
  velocity_i = increment/deltaT;
  prevT_i = currT;
};







//RoverMotor(int pwm, int dir1, int dir2, int ENCA, int ENCB, float kp,float ki);

//RoverMotor motor2=RoverMotor(8,37,36,19,38,3,22);
//RoverMotor motor3=RoverMotor(9,43,42,3,49,3,22);
// RoverMotor motor4=RoverMotor(5,26,27,2,23,3,22);
RoverMotor motor1(12,34,35,18,31,3,22);
RoverMotor motor2(8,37,36,19,38,3,22);
void setup(){
  //insert 
  Serial.begin(9600);
  Serial.println();

  
}

void loop() {
  motor1.setMotor(1, 255);
  motor2.setMotor(-1,255);
//  motor1.readEncoder();
//  // Set the motor speed and direction
}
