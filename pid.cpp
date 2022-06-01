// Drive motor by PID controll
#include <Encoder.h>
#include <PID_v1.h>
#include <Math.h>
#define ENA 10 
const int IN1=11;
const int IN2=12;
const int r =0.1;
#define M1 8 // PWM outputs to L298N H-Bridge motor driver module
double kp = 0.1, ki = 0, kd = 0;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
Encoder myEnc(2,4);

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
   pinMode(IN1, OUTPUT);
   pinMode(IN2, OUTPUT);
   pinMode(ENA, OUTPUT);
   myPID.SetMode(AUTOMATIC);
   myPID.SetSampleTime(1);
   myPID.SetOutputLimits(-255, 255);
   Serial.begin (115200);  
}

void loop() {
   setpoint = convert(2);
   long input = myEnc.read();
   myPID.Compute();
   Drive_Motor(output);
}

long int convert(double tocdo) {
   long int pulse = (60*tocdo)/(2*PI*r);
   return pulse;
}

void Drive_Motor(int out) {
   if (out > 0){
      digitalWrite(IN1,HIGH);
      digitalWrite(IN2,LOW);
      analogWrite(ENA, out);
   }else {
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,HIGH);
      analogWrite(ENA, abs(out));
   }
}
