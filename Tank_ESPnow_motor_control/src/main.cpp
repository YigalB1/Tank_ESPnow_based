#include "WiFi.h"
#include <esp_now.h>
//#include<ESP8266WiFi.h>
//#include <espnow.h>
#include<tank_classes.cpp>


Tank my_tank;
Motor motor_left;
Motor motor_right;
int rc_x = 0;
int rc_y = 0;
int range=12; // perhaps this should be bessaged from RC, becaue same value here and there



typedef struct struct_message {
  int x_val;
  int y_val;
  bool button_state;
  char ctrl_msg[32];
  //float c;
  //String d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  my_tank.tank_go_vector(myData.x_val,myData.y_val,myData.button_state,range);
}

// ****************** SETUP ************* 
void setup()
{
  Serial.begin(9600);
  Serial.print("");
  Serial.println("Starting setup");
   
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("init ESP-NOW ok");
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

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
   pinMode(BUZZER_pin, OUTPUT);
   

// For the tank DC motors
   // for ESP32 - different than ESp8266/WEMOS
   // example from: https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
   // configure LED PWM functionalitites
  ledcSetup(L_PWM_Channel, MOTOR_FREQ, PWM_REOLUTION);
  ledcSetup(R_PWM_Channel, MOTOR_FREQ, PWM_REOLUTION);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWMA_pin, L_PWM_Channel);
  ledcAttachPin(PWMB_pin, R_PWM_Channel);
 
  my_tank.tank_init_motors(AIN1_pin,AIN2_pin,PWMA_pin, L_PWM_Channel, BIN1_pin,BIN2_pin,PWMB_pin, R_PWM_Channel, STBY_pin);
  my_tank.tank_init_servos(F_SERVO_PWM_PIN,B_SERVO_PWM_PIN,R_SERVO_PWM_PIN,L_SERVO_PWM_PIN);


  
  digitalWrite(BUZZER_pin,HIGH); // we are ready to go !
  delay(1000);
  digitalWrite(BUZZER_pin,LOW); // we are ready to go !
  my_tank.test_hw();  // 

  //my_tank.tank_init_us_sensors();

   //my_tank.tank_test();
   
   // my_tank.test_sensor(my_tank.f_sensor,5,200);  // params: num of readings, delay 
  //    Serial.println("testing Sensor: Front");
//   my_tank.test_sensor(my_tank.f_sensor,5,200);  // params: num of readings, delay 

//   my_tank.set_motors_on();
//   my_tank.test_moves();
//   my_tank.set_motors_off();

   digitalWrite(LED_MOV_pin,HIGH); // we are ready to go !
   my_tank.set_motors_on();
   Serial.println("End of setup");
 
  
} // of SETUP

// ***************** LOOP ************
void loop() 
{
     
} // of LOOP





