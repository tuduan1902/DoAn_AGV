#include <Encoder.h>
#include <PID_v1.h>
#include <Math.h>
#define ENA 10 // PWM outputs to L298N H-Bridge motor driver module
const int IN1=11;
const int IN2=12;

#define ENB 7 // PWM outputs to L298N H-Bridge motor driver module
const int IN3=8;
const int IN4=9;

double r =0.1;
double kp1 = 0.4, ki1 = 0, kd1 = 0; // modify for optimal performance
double kp2 = 0.3, ki2 = 0, kd2 = 0; // modify for optimal performance
double input1 = 0, output1 = 0, setpoint1 = 0;
double input2 = 0, output2 = 0, setpoint2 = 0;
double starttime;
double encoder_res = 2400; // enconder resolution 
Encoder myEnc1(2,4);
Encoder myEnc2(3,5); // xanh lá là chân số 5  

PID myPID1(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT' 
PID myPID2(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT' 

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(1);
  myPID1.SetOutputLimits(-240, 240);
  
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetSampleTime(1);
  myPID2.SetOutputLimits(-255, 255);
  Serial.begin (115200);
  starttime = 0; 
  input1 = myEnc1.read(); // read value of original position
  input2 = myEnc2.read(); // read value of original position
}

void loop() {
  double nowtime = millis();
  //Serial.println(nowtime);
  if((nowtime - starttime) >= 10) {
    //Serial.println("abc");
    //Serial.println(delta_time);
    setpoint1 += convert(0);
    setpoint2 += convert(0);
    setpoint1 = 2400;
    setpoint2 = 2400;  
    input1 = myEnc1.read();
    input2 = myEnc2.read();
    Serial.println(input2);
    myPID1.Compute();
    myPID2.Compute();
    //Serial.println(output1);
    Drive_MotorA(output1);
    Drive_MotorB(output2);
 
    starttime = millis();
  }
}

double convert(double tocdo) {
  double y = ((60*tocdo))/((2*3.14*r)); // vòng/phút
  //Serial.println(y);
  double pulse = (y*encoder_res)/(60*100); // convert to pulse/10ms
  //Serial.println(pulse);
  return pulse;
}

void Drive_MotorA(int out) {
  //Serial.println(out);
  if (out > 0){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA, out);
    
  }else {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(ENA, abs(out));
  }
}

void Drive_MotorB(int out) {
  //Serial.println(out);
  if (out > 0){
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(ENB, out);
    
  }else {
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(ENB, abs(out));
  }
}



// Control 1 engine 
#include <Encoder.h>
#include <PID_v1.h>
#include <Math.h>
#define ENA 10 // PWM outputs to L298N H-Bridge motor driver module
const int IN1=11;
const int IN2=12;
double r =0.1;
double kp = 0.2, ki = 0, kd = 0;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
double starttime;
double encoder_res = 4800; // enconder resolution 
Encoder myEnc(2,4);

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(ENA, HIGH);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  Serial.begin (115200);
  starttime = 0;
  input = myEnc.read(); // read value of original position 
}

void loop() {
  double nowtime = millis();
  //Serial.println(nowtime);
  if((nowtime - starttime) >= 10) {
    //Serial.println("abc");
    //Serial.println(delta_time);
    setpoint += convert(0);
    input = myEnc.read();
    Serial.println(input);
    myPID.Compute();
    Drive_Motor(output);
    starttime = millis();
  }
}

double convert(double tocdo) {
  double y = ((60*tocdo))/((2*3.14*r)); // vòng/phút
  //Serial.println(y);
  double pulse = (y*encoder_res)/(60*100); // convert to pulse/10ms
  //Serial.println(pulse);
  return pulse;
}

void Drive_Motor(int out) {
  //Serial.println(out);
  if (out > 0){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA, out);
  }else {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(ENA, abs(out));
  }
}
