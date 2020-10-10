#include "WiFi.h"
#include <esp_now.h>
//#include<ESP8266WiFi.h>
//#include <espnow.h>
#include<tank_classes.cpp>
//#include <SparkFun_TB6612.h>  //  library for the TB6612 with Motor class & functions

//#include <ESP32Servo.h>
 
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch





Tank my_tank;
Motor motor_left;
Motor motor_right;
int rc_x = 0;
int rc_y = 0;

//void test_hw();
 
void setup()
{
   Serial.begin(9600);
   Serial.print("");
   Serial.println("finish setup");


   WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  while (true) {
     
  }

   // pinMode(LED_BUILTIN, OUTPUT);  
   pinMode(AIN1_pin, OUTPUT);
   pinMode(AIN2_pin, OUTPUT);
   //pinMode(PWMA_pin, OUTPUT);  // done later as ledcAttachPin
   pinMode(BIN1_pin, OUTPUT);    // done later as ledcAttachPin
   pinMode(BIN2_pin, OUTPUT);
   //pinMode(PWMB_pin, OUTPUT);
   //pinMode(Spare_LED, OUTPUT);

   pinMode(F_TRIG_PIN, OUTPUT);
   pinMode(B_TRIG_PIN, OUTPUT);
   pinMode(R_TRIG_PIN, OUTPUT);
   pinMode(L_TRIG_PIN, OUTPUT);
   pinMode(F_ECHO_PIN, INPUT);
   pinMode(B_ECHO_PIN, INPUT);
   pinMode(R_ECHO_PIN, INPUT);
   pinMode(L_ECHO_PIN, INPUT);
   
   pinMode(STBY_pin, OUTPUT);
   pinMode(LED_MOV_pin, OUTPUT);

// For the tank DC motors
   // for ESP32 - different than ESp8266/WEMOS
   // example from: https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
   // configure LED PWM functionalitites
  ledcSetup(L_PWM_Channel, MOTOR_FREQ, PWM_REOLUTION);
  ledcSetup(R_PWM_Channel, MOTOR_FREQ, PWM_REOLUTION);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWMA_pin, L_PWM_Channel);
  ledcAttachPin(PWMB_pin, R_PWM_Channel);



// for the servo motors

 //  ledcSetup(F_SERVO_Channel, SERVO_FREQ, PWM_REOLUTION);
/*   
   ledcSetup(B_SERVO_Channel, freq, PWM_REOLUTION);
   ledcSetup(R_SERVO_Channel, freq, PWM_REOLUTION);
   ledcSetup(L_SERVO_Channel, freq, PWM_REOLUTION);
*/

 //  ledcAttachPin(F_SERVO_PWM_PIN, F_SERVO_Channel);
/*   
   ledcAttachPin(B_SERVO_PWM_PIN, B_SERVO_Channel);
   ledcAttachPin(R_SERVO_PWM_PIN, R_SERVO_Channel);
   ledcAttachPin(L_SERVO_PWM_PIN, L_SERVO_Channel);
*/



  my_tank.tank_init_motors(AIN1_pin,AIN2_pin,PWMA_pin, L_PWM_Channel, BIN1_pin,BIN2_pin,PWMB_pin, R_PWM_Channel, STBY_pin);
  my_tank.tank_init_servos(F_SERVO_PWM_PIN,B_SERVO_PWM_PIN,R_SERVO_PWM_PIN,L_SERVO_PWM_PIN);
  my_tank.tank_init_us_sensors();

   //my_tank.tank_test();
  
} // of SETUP
 
 
void loop() 
{
   my_tank.set_motors_on();
   my_tank.test_moves();
   my_tank.set_motors_off();
   return;

   Serial.println("testing Servo: Front");
   my_tank.test_servos();
   return;  

   my_tank.test_sensor(my_tank.f_sensor,5,200);  // params: num of readings, delay 
   return;



   Serial.println("testing Sensor: Front");
   my_tank.test_sensor(my_tank.f_sensor,5,200);  // params: num of readings, delay 


   return;

   

   




   // rc_x,rc_y are the readings from the joystick in the remote control
   // ranging from -1023 to 0 to 1023
   // is this IF necessary? probably not


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





