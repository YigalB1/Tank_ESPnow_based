#include<ESP8266WiFi.h>
#include<espnow.h>
#include<tank_classes.cpp>
//#include <SparkFun_TB6612.h>  //  library for the TB6612 with Motor class & functions
 
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch

#define AIN1_pin 4 // D2
#define BIN1_pin 5 // D1
#define AIN2_pin 0 // D3
#define BIN2_pin 12 //   (was GPIO3 (RX) changes to GPIO12  D6
#define PWMA_pin 2 // D4
#define PWMB_pin 16 // TX - changed
#define STBY_pin 14 // D5 , swhen low, all stop, low current consumption


Tank my_tank;
Motor motor_left;
Motor motor_right;
int rc_x = 0;
int rc_y = 0;

void test_hw();
 
void setup()
{
// pinMode(LED_BUILTIN, OUTPUT);  

my_tank.tank_init(AIN1_pin,AIN2_pin,PWMA_pin,BIN1_pin,BIN2_pin,PWMB_pin,STBY_pin);

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
   digitalWrite(AIN1_pin,HIGH);
   delay(delay_time);
   Serial.println("2");
   digitalWrite(AIN1_pin,LOW);
   delay(delay_time);
   Serial.println("3");
   digitalWrite(AIN2_pin,HIGH);
   delay(delay_time);
   Serial.println("4");
   digitalWrite(AIN2_pin,LOW);
   delay(delay_time);
   Serial.println("5");
   digitalWrite(BIN1_pin,HIGH);
   delay(delay_time);
   Serial.println("6");
   digitalWrite(BIN1_pin,LOW);
   delay(delay_time);
   Serial.println("7");
   digitalWrite(BIN2_pin,HIGH);
   delay(delay_time);
   Serial.println("8");  
   digitalWrite(BIN2_pin,LOW);
   delay(delay_time);
   Serial.println("9");
   analogWrite(PWMA_pin, 500);
   Serial.println("10");
   delay(delay_time);
   Serial.println("......");
   delay(delay_time);
   analogWrite(PWMB_pin, 500);
   Serial.println("11");
   delay(delay_time);  
   doit = false;
}

}