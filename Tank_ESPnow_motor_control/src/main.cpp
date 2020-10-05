#include "WiFi.h"
#include <esp_now.h>
//#include<ESP8266WiFi.h>
//#include <espnow.h>
#include<tank_classes.cpp>
//#include <SparkFun_TB6612.h>  //  library for the TB6612 with Motor class & functions
 
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch

#define AIN1_pin     26 // D2
#define BIN1_pin     14 // D1
#define AIN2_pin     25 // D3
#define BIN2_pin     12 //   (was GPIO3 (RX) changes to GPIO12  D6
#define PWMA_pin     33 // D7      was: 2 // D4
#define PWMB_pin     13 // TX - changed
#define STBY_pin     27 // D5 , when low, all stop, low current consumption
#define LED_MOV_pin  15 // D4 , when the tank is moving, on board LED
//#define Spare_LED    15 // D8, currently not in use


Tank my_tank;
Motor motor_left;
Motor motor_right;
int rc_x = 0;
int rc_y = 0;

void test_hw();
 
void setup()
{
// pinMode(LED_BUILTIN, OUTPUT);  
pinMode(AIN1_pin, OUTPUT);
pinMode(AIN2_pin, OUTPUT);
pinMode(PWMA_pin, OUTPUT);
pinMode(BIN1_pin, OUTPUT);
pinMode(BIN2_pin, OUTPUT);
pinMode(PWMB_pin, OUTPUT);
//pinMode(Spare_LED, OUTPUT);

pinMode(LED_MOV_pin, OUTPUT);

// for ESP32 - different than ESp8266/WEMOS
// example from: https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
// configure LED PWM functionalitites
  ledcSetup(L_PWM_Channel, freq, resolution);
  ledcSetup(R_PWM_Channel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWMA_pin, L_PWM_Channel);
  ledcAttachPin(PWMA_pin, R_PWM_Channel);



my_tank.tank_init(AIN1_pin,AIN2_pin,PWMA_pin, L_PWM_Channel, BIN1_pin,BIN2_pin,PWMB_pin, R_PWM_Channel, STBY_pin);

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

   my_tank.test_moves();

/*
      
   for (int i=0;i<1023;i+=300) {
      Serial.print(i);
      Serial.print("..");
      Serial.print(rc_x);
      Serial.print("..");
      Serial.print(rc_y);
      Serial.println("");
      
      rc_x = i;
      rc_y = i;
      Serial.print(i);
      Serial.print("..");
      Serial.print(rc_x);
      Serial.print("..");
      Serial.print(rc_y);
      Serial.println("");



      my_tank.tank_move(rc_x,rc_y);
      delay(2000);
   }
   
   for (int i=-1023;i<0;i+=300) {
      Serial.println(i);
      Serial.println("..");
      rc_x = i;
      rc_y = i;
      my_tank.tank_move(rc_x,rc_y);
      delay(2000);
   }

   */



} // of LOOP





// to delete when all works fine
void test_hw() {

int delay_time = 500;

   digitalWrite(LED_MOV_pin,HIGH);
   //digitalWrite(Spare_LED,HIGH);
   delay(1000);   
   Serial.print("1.");
   digitalWrite(AIN1_pin,HIGH);
   delay(delay_time);
   Serial.print("2.");
   digitalWrite(AIN1_pin,LOW);
   delay(delay_time);
   Serial.print("3.");
   digitalWrite(AIN2_pin,HIGH);
   delay(delay_time);
   Serial.print("4.");
   digitalWrite(AIN2_pin,LOW);
   delay(delay_time);
   Serial.print("5.");
   digitalWrite(BIN1_pin,HIGH);
   delay(delay_time);
   Serial.print("6.");
   digitalWrite(BIN1_pin,LOW);
   delay(delay_time);
   Serial.print("7.");
   digitalWrite(BIN2_pin,HIGH);
   delay(delay_time);
   Serial.print("8.");  
   digitalWrite(BIN2_pin,LOW);
   delay(delay_time);
   Serial.print("9.");
   //analogWrite(PWMA_pin, 500);
   //Serial.print("10.");
   //delay(delay_time);
   Serial.println("......");
   delay(delay_time);
   //analogWrite(PWMB_pin, 500);
   //Serial.println("11");
   //delay(delay_time);  
 
   digitalWrite(LED_MOV_pin,LOW);
   //digitalWrite(Spare_LED,LOW);

}