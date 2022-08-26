/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

/*
1500 pulses a second
66 rpm on the wheel
72 ppr

525 pulses a second
24 rpm on the wheel
69 ppr
*/
#include <Encoder.h>
Encoder myEnc(19, 38);
  
#define ENC_COUNT_REV 70.0

int sampleTime = 1000;
unsigned long long lastTime = 0;
unsigned long long now=0;
unsigned long long timeChange;

 
float rpm_right = 0;
float pulse = 0;

double setpoint, input, output;
double error, sumError, dErr, lastError;
//double error, lastError, lastLastError;
double iError, dError;
double lastOutput;
double kp = .5;
double ki = 0.0;
double kd = 0.0;
int max_control = 10;

void setup() {
  Serial.begin(9600);
  //Serial.println("Basic Encoder Test:");

  sumError =0;
  dErr = 0;
  dError =0;
  
  pinMode(8, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(36, OUTPUT);

  //analogWrite(8, 20);
  digitalWrite(37,HIGH);
  digitalWrite(36,LOW);
}

void loop() {
  if(Serial.available() > 0) {
    setpoint = Serial.readString().toDouble();
  }

  now = millis();
  timeChange = now - lastTime;
  if (timeChange >= sampleTime) {
    PID_control();
    lastTime = now;
    analogWrite(8, output);
  }
  
  input = compute_input();
  print_results();
}

double compute_input() {
  pulse = myEnc.readAndReset();
  Serial.print(pulse);
  rpm_right = (pulse * (60000/timeChange) / ENC_COUNT_REV);
  rpm_right = rpm_right / 18.8;
  return double(rpm_right);
}

void PID_control() {
  error = -setpoint + input; //30
  //sumError += error * (60000.0/timeChange); //
  //dErr = (error - lastError) / (60000.0/timeChange);

  output = lastOutput + (kp*error) + (ki*sumError) + (kd*dErr);

//  lastLastError = lastError;
  lastError = error;
  lastOutput = output;
}

void print_results() {
//    Serial.println(" Pulses: " + String(pulse));
//    Serial.println(" Speed: " + String(rpm_right) + " RPM");
//    Serial.println(" Input: " + String(input) + " Output: " + String(output) + " Setpoint: " + String(setpoint));
//    Serial.println();
//    Serial.flush();
  Serial.print(input);
  Serial.print(" ");
  Serial.print(output);
  Serial.print(" ");
  Serial.println(setpoint);

//  Serial.println(String(now) + "," + String(input) + "," + String(setpoint) + "," + String(output));
}
