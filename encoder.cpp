#include <Encoder.h>
Encoder Enc1(2,4);
Encoder Enc2(3,5);


void setup(){
   Serial.begin(9600);
}
long int Position1  = -999;
long int Position2 = -999;


void loop(){
   long enc1, enc2;
   enc1 = Enc1.read();
   enc2 = Enc2.read();
   if(enc1 != Position1 || enc2 != Position2){
      Serial.print("Encoder1: ");
      Serial.print(enc1);
      Serial.print("Encoder2: ");
      Serial.println(enc2);
      Position1 = enc1;
      Position2 = enc2;
   }
}