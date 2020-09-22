#include<ESP8266WiFi.h>
#include<espnow.h>
//#include <SparkFun_TB6612.h>  //  library for the TB6612 with Motor class & functions
 
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch

#define AIN1 4 // D2
#define BIN1 5 // D1
#define AIN2 0 // D3
#define BIN2 12 //   (was GPIO3 (RX) changes to GPIO12  D6
#define PWMA 2 // D4
#define PWMB 16 // TX - changed
#define STBY 13 // TBD - not connected in the module used in the tank



 
void setup()
{
 //Nothing here
// pinMode(LED_BUILTIN, OUTPUT);  


pinMode(AIN1, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(AIN2, OUTPUT);
pinMode(BIN2, OUTPUT);
pinMode(PWMA, OUTPUT);
pinMode(PWMB, OUTPUT);
pinMode(STBY, OUTPUT);



 Serial.begin(9600);
 Serial.print("");
 Serial.println("starting setup");
}
 
 
void loop()
{
 
   
bool doit=true;

while (doit) {
   Serial.println("1");
   digitalWrite(AIN1,HIGH);
   delay(1000);
   Serial.println("2");
   digitalWrite(AIN1,LOW);
   delay(1000);
   Serial.println("3");
   digitalWrite(AIN2,HIGH);
   delay(1000);
   Serial.println("4");
   digitalWrite(AIN2,LOW);
   delay(1000);
   Serial.println("5");
   digitalWrite(BIN1,HIGH);
   delay(1000);
   Serial.println("6");
   digitalWrite(BIN1,LOW);
   delay(1000);
   Serial.println("7");
   digitalWrite(BIN2,HIGH);
   delay(1000);
   Serial.println("8");
   
   
   digitalWrite(BIN2,LOW);
   delay(1000);
   Serial.println("9");
   
   analogWrite(PWMA, 500);
   
   Serial.println("10");
   delay(1000);
   Serial.println("......");
   delay(1000);
   analogWrite(PWMB, 500);
   delay(2000);
   Serial.println("11");
   
}


   Serial.println("Forward");
   // Set Motor A forward
   digitalWrite(AIN1, HIGH);
   digitalWrite(AIN2, LOW);
 
 // Set Motor B forward
   //digitalWrite(BIN1, HIGH);
  //digitalWrite(BIN1, LOW);
  int speedA = 500;
  //int speedB = 500;
  //analogWrite(PWMA, speedA);
  //analogWrite(PWMB, speedB);



  // Read potentiometers and convert to range of 0-255
  
  //MotorSpeed1 = map(analogRead(SpeedControl1), 0, 1023, 0, 255);
  //MotorSpeed2 = map(analogRead(SpeedControl2), 0, 1023, 0, 255);  
     
  
  // Set the motor speeds
  
  
   


   delay(1000);
   
  Serial.println("Stop");
   digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

   
      analogWrite(PWMA, 500);
      delay(2000);
   
   
  speedA = 1023;
  //speedB = 0;
  analogWrite(PWMA, speedA);
  //analogWrite(PWMB, speedB);

   delay(2000);

} // of LOOP