//#define MOTOR_PWM       5   // D1   PWM pin: between 0 - 1023 analog, or 0 off 1 full speed digital
//#define MOTOR_DIR       0   // D3   0 forward. 1 backward
#define ON_BOARD_LED    2   //D4 Or GPIO-2 of nodemcu ESP8266
//#define GREEN_LED       12  //D6 Or GPIO-2 of nodemcu ESP8266
//#define YELLOW_LED      13  //D7 Or GPIO-2 of nodemcu ESP8266
//#define RED_LED         15  //D8 Or GPIO-2 of nodemcu ESP8266

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


const int LEFT  = 0;
const int RIGHT = 1;
const int FORWARD = 0;
const int BACKARD = 0;

const int ZERO  = 0;
//const int LEFT_DIR  = 1; // it should have been like LEFT & RIGHT, but sto match the  real engine direction.
//const int RIGHT_DIR = 0;

const bool fixed_speed = false; // True: speed is fixed or stopped. False: speed changes (slower or faster)
const bool DEBUG_MODE = true; // when true, printing debug statemens