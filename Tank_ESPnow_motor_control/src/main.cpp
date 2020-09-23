#include<ESP8266WiFi.h>
#include<espnow.h>
#include<tank_classes.cpp>
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


Tank my_tank;
Motor motor_left;
Motor motor_right;
int rc_x = 0;
int rc_y = 0;

void test_hw();
 
void setup()
{
 //Nothing here
// pinMode(LED_BUILTIN, OUTPUT);  

my_tank.tank_init(AIN1,AIN2,PWMA,BIN1,BIN2,PWMB);
//motor_left.init(AIN1,AIN2,PWMA);
//motor_right.init(BIN1,BIN2,PWMB);
//motor_left.stop();
//motor_right.stop();

//pinMode(AIN1, OUTPUT);
//pinMode(BIN1, OUTPUT);
//pinMode(AIN2, OUTPUT);
//pinMode(BIN2, OUTPUT);
//pinMode(PWMA, OUTPUT);
//pinMode(PWMB, OUTPUT);
//pinMode(STBY, OUTPUT);



 Serial.begin(9600);
 Serial.print("");
 Serial.println("finish setup");
 test_hw();
} // of SETUP
 
 
void loop()
{
   // rc_x,rc_y are the readings from the joystick in the remote control
   // ranging from -1023 to 0 to 1023
   // is this IF necessary? probably not

 
   

   my_tank.tank_move(rc_x,rc_y);
   



} // of LOOP


// to delete when all works fine
void test_hw() {
    
   
bool doit=true;
int delay_time = 500;

while (doit) {
   Serial.println("1");
   digitalWrite(AIN1,HIGH);
   delay(delay_time);
   Serial.println("2");
   digitalWrite(AIN1,LOW);
   delay(delay_time);
   Serial.println("3");
   digitalWrite(AIN2,HIGH);
   delay(delay_time);
   Serial.println("4");
   digitalWrite(AIN2,LOW);
   delay(delay_time);
   Serial.println("5");
   digitalWrite(BIN1,HIGH);
   delay(delay_time);
   Serial.println("6");
   digitalWrite(BIN1,LOW);
   delay(delay_time);
   Serial.println("7");
   digitalWrite(BIN2,HIGH);
   delay(delay_time);
   Serial.println("8");  
   digitalWrite(BIN2,LOW);
   delay(delay_time);
   Serial.println("9");
   analogWrite(PWMA, 500);
   Serial.println("10");
   delay(delay_time);
   Serial.println("......");
   delay(delay_time);
   analogWrite(PWMB, 500);
   Serial.println("11");
   delay(delay_time);  
   doit = false;
}

}