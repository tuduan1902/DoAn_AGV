  #include <PID_v1.h> //
  double kp1 = 0.4;
  double ki1 = 0;
  double kd1 = 0;

  double Setpoint1 = 0, Input1 = 0, Output1 = 0, Output1a; // PID variables
  PID my_PID1(&Input1, &Setpoint1, kp1, ki1, kd1, DIRECT); // PID Setup

  double kp2 = 0.3;
  double ki2 = 0;
  double kd2 = 0;

  double Setpoint2 = 0, Input2 = 0, Output2 = 0, Output2a; // PID variables
  PID my_PID2(&Input2, &Setpoint2, kp2, ki2, kd2, DIRECT); // PID Setup

  double r=0.05; // bán kính bánh xe 
  double encoder_res = 2400; // encoder resolution

  #define ENA 10 // PWM outputs to L298N H-Bridge motor driver module
  const int IN1=11;
  const int IN2=12;

  #define ENB 7 // PWM outputs to L298N H-Bridge motor driver module
  const int IN3=8;
  const int IN4=9;
   
  Encoder myEnc1(2, 4);
  Encoder myEnc2(3, 5);

  // Wheel encoder interrupts
  #define encoder0PinA 2    // encoder 1
  #define encoder0PinB 3

  #define encoder1PinA 18   // encoder 2 
  #define encoder1PinA 19

  volatile long encoder0Pos = 0; // encoder 1
  volatile long encoder1Pos = 0; // encoder 2


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

   double convert(double tocdo) {
      double y = ((60*tocdo))/((2*3.14*r)); // vòng/phút
      //Serial.println(y);
      double pulse = (y*encoder_res)/(60*100); // convert to pulse/10ms
      //Serial.println(pulse);
      return pulse;
   }

   void setup() {
      pinMode(IN1, OUTPUT);  // motor PWM pins
      pinMode(IN2, OUTPUT);
      pinMode(IN3, OUTPUT);
      pinMode(IN4, OUTPUT);

      digitalWrite(ENA, HIGH);
      digitalWrite(ENB, HIGH);

      my_PID1.SetMode(AUTOMATIC);
      my_PID1.SetOutputTime(1);
      my_PID1.SetOutputLimits(-255, 250);
      

      my_PID1.SetMode(AUTOMATIC);
      my_PID1.SetOutputTime(1);
      my_PID1.SetOutputLimits(-252, 255);
      
      Serial.begin(115200);
      starrtime = 0;
      input1 = myEnc1.read(); // read value of original position
      input2 = myEnc2.read(); // read value of original position
      
   }

   void loop() {
      double nowtime = millis();
      if((nowtime - starttime) >= 10) { // start timed loop for everything else

         if(Serial.available() > 0) { //manual control of wheesls via terminal
            char c = Serial.read();

            if(c == 'a') {
               Setpoint1 += convert(1);
               Setpoint2 += convert(1);
            } else if( c == 'b') {
               Setpoint1 += convert(-1);
               Setpoint2 += convert(-1);
            } else if( c == 'l') {
               Setpoint1 += convert(1);
               Setpoint2 += convert(0);
            } else if( c == 'r') {
               Setpoint1 += convert(0);
               Setpoint2 += convert(1);
            } else if (c == 'p') {
               Setpoint1 += convert(0);
               Setpoint2 += convert(0);
            }
         }
      input1 = myEnc1.read();
      myPID1.Compute();

      input2 = myEnc2.read();
      //Serial.println(input2);
      myPID2.Compute();
      //Serial.println(output1);
      Drive_MotorA(output1);
      Drive_MotorB(output2);
   
      starttime = millis();

      }
   }
