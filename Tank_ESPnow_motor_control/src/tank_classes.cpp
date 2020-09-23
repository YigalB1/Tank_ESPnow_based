#include<tank_headers.h>
#include<ESP8266WiFi.h>



class Motor {
  public:
  int motor_en1;
  int motor_en2;
  int motor_pwm;
  int speed = 0; // in work, to use as the speed of the train instead of global variable
  int direction = FORWARD;
  int distance = 0;
  
  void init(int _in1, int _in2, int _pwm) {
    motor_en1 = _in1;
    motor_en2 = _in2;
    motor_pwm = _pwm;
    pinMode(motor_en1, OUTPUT);
    pinMode(motor_en2, OUTPUT);
    pinMode(motor_pwm, OUTPUT);
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

  void Go_left_motor(int _speed) {

  }

    void Go_right_motor(int _speed) {
    
  }



    void Go_forward ( int _speed_) {
      digitalWrite(motor_en1, HIGH);
      digitalWrite(motor_en2, LOW );
      analogWrite(motor_pwm, _speed_);
    } // of GO LEFT routine


    void Go_backward ( int _speed_) {
      digitalWrite(motor_en1, LOW);
      digitalWrite(motor_en2, HIGH );
      analogWrite(motor_pwm, _speed_);
    } // of GO RIGHT routine


    void stop () {
      speed = ZERO;
      analogWrite(motor_pwm,  ZERO);
      digitalWrite(motor_en1, LOW);
      digitalWrite(motor_en2, LOW);
      
      digitalWrite(GREEN_LED, LOW );
      digitalWrite(YELLOW_LED,HIGH);
      digitalWrite(RED_LED,   LOW );
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
  public:
  Motor left_motor;
  Motor right_motor;
  void tank_init(int _l_int1,int _l_int2, int _l_pwm,int _r_int1,int _r_int2, int _r_pwm) {  
    left_motor.init(_l_int1,_l_int2,_l_pwm);
    right_motor.init(_r_int1,_r_int2,_r_pwm);
  }

  void tank_stop() {
    left_motor.stop();
    right_motor.stop();
  }

  void tank_move(int _x,int _y) {
      if (0 == _x && 0 == _y) {
      tank_stop();
      return;

      //int loc_x = _x;
      //int loc_y = _y;
      int ratio;
      if (0 == _y || 0 == _x)
        ratio=1;      
      else
        ratio = abs(int(_x/_y));

      if (_x>=0 && _y>0) {
        // forward and right, right motor slowing
        // if x=0 it is fully FWD
        left_motor.Go_forward(_y);
        right_motor.Go_forward(abs(_y*ratio));
        return;
      }
      if (_x>=0 && _y<0) {
        // backward and left, left engine slowing
        left_motor.Go_backward(abs(_y));
        right_motor.Go_backward(abs(_y*ratio));
        return;
      }
      if (_x<=0 && _y<0) {
        // bacward and right, right engine slowing
        left_motor.Go_backward(_y);
        right_motor.Go_backward(abs(_y*ratio));
        return;
      }
      if (_x<0 && _y>0) {
        // forward and left, left motor slowing
        left_motor.Go_forward(int(_y*ratio));
        right_motor.Go_forward(_y);
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

  }
}; // of TANK class