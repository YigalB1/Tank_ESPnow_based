//#define MOTOR_PWM       5   // D1   PWM pin: between 0 - 1023 analog, or 0 off 1 full speed digital
//#define MOTOR_DIR       0   // D3   0 forward. 1 backward
//#define ON_BOARD_LED    2   //D4 Or GPIO-2 of nodemcu ESP8266
//#define GREEN_LED       12  //D6 Or GPIO-2 of nodemcu ESP8266
//#define YELLOW_LED      13  //D7 Or GPIO-2 of nodemcu ESP8266
//#define RED_LED         15  //D8 Or GPIO-2 of nodemcu ESP8266

#define AIN1_pin     26 // D2
#define BIN1_pin     14 // D1
#define AIN2_pin     25 // D3
#define BIN2_pin     12 //   (was GPIO3 (RX) changes to GPIO12  D6
#define PWMA_pin     33 // D7      was: 2 // D4
#define PWMB_pin     13 // TX - changed
#define STBY_pin     27 // D5 , when low, all stop, low current consumption
#define LED_MOV_pin  15 // D4 , when the tank is moving, on board LED
//#define Spare_LED    15 // D8, currently not in use

#define F_TRIG_PIN  32
#define F_ECHO_PIN  35
#define B_TRIG_PIN  21
#define B_ECHO_PIN  34
#define R_TRIG_PIN  23
#define R_ECHO_PIN  36
#define L_TRIG_PIN  4
#define L_ECHO_PIN  16

#define F_SERVO_PWM 22
#define B_SERVO_PWM 18
#define R_SERVO_PWM 5
#define L_SERVO_PWM 17

const int trig1 = 32;

const int JUNK_VAL = 7777;
const int DIST_BUFF_SIZE = 20;
const int VERY_CLOSE = 23;
const int CLOSE = 23;
const int IN_RANGE = 40;
const int MIN_SPEED = 120; // was 120
const int MAX_SPEED = 1023; // MAX is 1023
const int SPEED_INC = 50;
const int TIME_IN_STATION = 1500;
const int SAMPLE_TIME = 200; // time for loop to wait between each cycle
const int MAX_DISTANCE = 99;


const int LEFT  = 0;
const int RIGHT = 1;
const int FORWARD = 0;
const int BACKARD = 0;


// for Esp32 (DIFFERENT THAN ESPP8266/WEMOS)
// setting PWM properties
const int freq = 5000;
const int L_PWM_Channel = 0;
const int R_PWM_Channel = 1;
const int resolution = 10; // 10 bit resulatio: speed until 1023 . Maybe 8? 



const int ZERO  = 0;
//const int LEFT_DIR  = 1; // it should have been like LEFT & RIGHT, but sto match the  real engine direction.
//const int RIGHT_DIR = 0;

const bool fixed_speed = false; // True: speed is fixed or stopped. False: speed changes (slower or faster)
const bool DEBUG_MODE = true; // when true, printing debug statemens