#include<tank_headers.h>
#include "WiFi.h"
//#include "analogWrite.h"
//#include<ESP8266WiFi.h>



class Motor {
  public:
  int motor_en1;
  int motor_en2;
  int motor_pwm;    // for ESP8266 style
  int pwm_channel;  // for ESP32 style
  int speed = 0; // in work, to use as the speed of the train instead of global variable
  int direction = FORWARD;
  int distance = 0;
  
  
  void init(int _in1, int _in2, int _pwm, int _pwm_channel) {
    // _pwm for ESp8266 style
    // _pwm_channel for esp32 style
    motor_en1 = _in1;
    motor_en2 = _in2;
    motor_pwm = _pwm;
    pwm_channel = _pwm_channel;






    speed=0;
  } // of INIT routine

/*
  String debug_msg() {
    String msg;
    msg.concat("dir: ");
    //Serial.print("dir: ");
    if (direction==LEFT) {
      msg.concat("LEFT  ");
      //Serial.print("LEFT ");
    }
    else{
      msg.concat("RIGHT ");
      //Serial.print("RIGHT");
    }
      
    msg.concat("Speed: ");
    msg.concat(speed);
    msg.concat("  Distance:");
    msg.concat(distance);
    msg.concat("   LEFT  Distance: ");
    msg.concat(left_distance);
    msg.concat("   RIGHT Distance: ");  
    msg.concat(right_distance);
    return msg;
  }
*/

  void Go_left(int _l_speed, int _r_speed) {

  }

    void Go_right(int _l_speed, int _r_speed) {
    
  }

  void Go_pivot_left(int _l_speed, int _r_speed) {

  }

  public: void Go_pivot_right(int _speed) {

  }


    void Go_forward ( int _speed_) {
      digitalWrite(motor_en1, HIGH);
      digitalWrite(motor_en2, LOW );
      //analogWrite(motor_pwm, _speed_);
      ledcWrite(pwm_channel, _speed_);



    } // of GO LEFT routine


    void Go_backward ( int _speed_) {
      digitalWrite(motor_en1, LOW);
      digitalWrite(motor_en2, HIGH );
      //analogWrite(motor_pwm, _speed_);
      ledcWrite(pwm_channel, _speed_);
    } // of GO RIGHT routine


    void stop () {
      speed = ZERO;
      //analogWrite(motor_pwm,  ZERO);
      ledcWrite(pwm_channel, ZERO);
      digitalWrite(motor_en1, LOW);
      digitalWrite(motor_en2, LOW);
      
    } // of STOP routine


    // ****************** increase_speed **********************
    void increase_speed() {

      if (fixed_speed) {
        speed = MAX_SPEED;
        return; // speed is not changing
      }
        
      // speed is not fixed

      speed += SPEED_INC;
      if (speed > MAX_SPEED)
        speed = MAX_SPEED;
    }
    // ****************** decrease_speed **********************
    void decrease_speed() {
      if (fixed_speed) {
        speed = MAX_SPEED;
        return; // speed is not changing
      }
        

      speed -= SPEED_INC;
      if (speed < MIN_SPEED)
        speed = MIN_SPEED;
    }

// ****************** SLOW_DOWN **********************
void slow_down() {
  if (fixed_speed)
    return; // speed is not changing

  speed = MIN_SPEED;
} // of SLOW DOWN





};  // of Motor class



class Sensor {

}; // of Sensor class





class Tank {
  bool STDBY = false; // power consumption mode: TBD
  public:
  Motor left_motor;
  Motor right_motor;

  void tank_init(int _l_int1_pin,int _l_int2_pin, int _l_pwm_pin,int _l_pwm_channel, int _r_int1_pin,int _r_int2_pin, int _r_pwm_pin, int _r_pwm_channel,int _stby_pin) {  
    // the pwm_pin is for ESp8266/WEMOS style, pwm_channel is for ESP32 style
    
    
    pinMode(_stby_pin, OUTPUT);
    STDBY = false; // starting with stdby mode
    digitalWrite(_stby_pin,HIGH);
    left_motor.init(_l_int1_pin,_l_int2_pin,_l_pwm_pin,_l_pwm_channel);
    right_motor.init(_r_int1_pin,_r_int2_pin,_r_pwm_pin,_r_pwm_channel);
  }


// XXXXXXXXXXXXX
  void tank_stop() {
    left_motor.stop();
    right_motor.stop();
  }


  void Tank_forward(int _speed) {
    left_motor.Go_forward(_speed);
    right_motor.Go_forward(_speed);    
  }

  void Tank_backward(int _speed) {
      left_motor.Go_backward(_speed);
      right_motor.Go_backward(_speed);    
    }

  void Tank_forward_turn(int _l_speed,int _r_speed) {
    left_motor.Go_forward(_l_speed);
    right_motor.Go_forward(_r_speed);    
  }  

  void Tank_backward_turn(int _l_speed,int _r_speed) {
    left_motor.Go_backward(_l_speed);
    right_motor.Go_backward(_r_speed);    
  }

  void Tank_left_pivot(int _speed) {
    left_motor.Go_backward(_speed);
    right_motor.Go_forward(_speed);    
  }

  void Tank_right_pivot(int _speed) {
    left_motor.Go_forward(_speed);
    right_motor.Go_backward(_speed);    
  }

  void tank_move(int _x,int _y) {    

      if (0 == _x && 0 == _y) {
        tank_stop();
        Serial.println("stopping  ");
        return;
      }

    
      int ratio;
      if (0 == _y || 0 == _x)
        ratio=1;      
      else
        ratio = abs(int(_x/_y));

/*
      Serial.print("x / y / ratio : ");
      Serial.print(_x);
      Serial.print("  ");
      Serial.print(_y);
      Serial.print("  ");
      Serial.println(ratio);
*/      


      if (_x>=0 && _y>0) {
        // 1Q
        // forward and right, right motor slowing
        // if x=0 it is fully FWD
        left_motor.Go_forward(_y);
        right_motor.Go_forward(abs(_y*ratio));
        Serial.println(" 1Q ");
        return;
      }
      if (_x>=0 && _y<0) {
        // 2Q
        // backward and left, left engine slowing
        left_motor.Go_backward(abs(_y));
        right_motor.Go_backward(abs(_y*ratio));
        Serial.println(" 2Q ");
        return;
      }
      if (_x<=0 && _y<0) {
        // 3Q
        // bacward and right, right engine slowing
        left_motor.Go_backward(abs(_y));
        right_motor.Go_backward(abs(_y*ratio));
        Serial.println(" 3Q ");
        return;
      }
      if (_x<0 && _y>0) {
        // 4Q
        // forward and left, left motor slowing
        left_motor.Go_forward(abs(int(_y*ratio)));
        right_motor.Go_forward(abs(_y));
        Serial.println(" 4Q ");
        return;
      }

      if (_y==0 && _x>0 ) {
        // pivot Right
        left_motor.Go_forward(_x);
        right_motor.Go_backward(_x);
        return;
      }

      if (_y==0 && _x<0 ) {
        // pivot Left
        left_motor.Go_backward(abs(_x));
        right_motor.Go_forward(abs(_x));
        return;
      }

      Serial.println("ERROR - should not be here, at TANK_MOVE");


   }  // end of tank_move

  

void test_moves() {
  int test_speed = 500;
  int l_speed = 500;
  int r_speed = 500;
  int test_time = 1000;
  int stop_time = 500;
  
  // test Foward
  Serial.println("in test_moves: Forward");
  Tank_forward(test_speed);
  delay(test_time);
  tank_stop();
  delay(stop_time);

  // test Backward
  Serial.println("in test_moves: Backward");
  Tank_backward(test_speed);
  delay(test_time);
  delay(stop_time);

  // test Left turn
  Serial.println("in test_moves: Left turn");
  Tank_forward_turn(l_speed,r_speed);
  delay(test_time);
  delay(stop_time);

  // test Right turn
  Serial.println("in test_moves: Right turn");
  Tank_forward_turn(l_speed,r_speed);
  delay(test_time);
  delay(stop_time);

  // test right pivot
  Serial.println("in test_moves: Right Pivot");
  Tank_right_pivot(test_speed);
  delay(test_time);
  delay(stop_time);

  // test left pivot
  Serial.println("in test_moves: LKeft Pivot");
  Tank_left_pivot(test_speed);
  delay(test_time);
  delay(stop_time);


} // of test_moves








}; // of TANK class