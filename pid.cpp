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
