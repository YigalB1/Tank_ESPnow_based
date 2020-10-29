#include<tank_headers.h>
#include "WiFi.h"
//#include <ESP32Servo.h>
//#include <NewPing.h>
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

  void Go_forward ( int _speed_) {
/*
Serial.println("");
Serial.print("  In go _FWD . _speed_:");
Serial.print(_speed_);
Serial.print("  pwm_channel:  ");
Serial.print(pwm_channel);
Serial.print("  motor_en1:  ");
Serial.print(motor_en1);
Serial.print("  motor_en2:  ");
Serial.println(motor_en2);
*/


      speed = _speed_; // set the class global value
      digitalWrite(motor_en1, HIGH);
      digitalWrite(motor_en2, LOW );
      //analogWrite(motor_pwm, _speed_);
      ledcWrite(pwm_channel, speed);
    } // of Go_forward

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


    


    void Go_backward ( int _speed_) {
      speed = _speed_; // set the class global value
      digitalWrite(motor_en1, LOW);
      digitalWrite(motor_en2, HIGH );
      //analogWrite(motor_pwm, _speed_);
      ledcWrite(pwm_channel, speed);
    } // of Go_backward


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
      if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
      }
        
    }
    // ****************** decrease_speed **********************
    void decrease_speed() {
      if (fixed_speed) {
        speed = MAX_SPEED;
        return; // speed is not changing
      }
        

      speed -= SPEED_INC;
      if (speed < MIN_SPEED) {
        speed = MIN_SPEED;
      }
        
    }

// ****************** SLOW_DOWN **********************
void slow_down() {
  if (fixed_speed)
    return; // speed is not changing
  speed = MIN_SPEED;
} // of SLOW DOWN

};  // of Motor class



// no need for this class? there is a servo class builtin
class us_Servo {
  public:
  int s_channel = 77;
  int s_pin = 777; 
  int s_freq = 77777;
  int s_resolution = 777777;
  

  void init(int _channel, int _pin, int _freq, int _resolution) {
    s_channel = _channel;
    s_pin = _pin;
    s_freq = _freq;
    s_resolution = _resolution;

    ledcSetup(s_channel, s_freq, s_resolution);
    ledcAttachPin(s_pin, s_channel);
  }

  void write_angle(int _value) {
     ledcWrite(s_channel, _value);
  }


  
}; // of class us_servo


class US_Sensor {
  public:
  int trig_pin;
  int echo_pin;

  void dum() {

  }

  void init(int _trig, int _echo) {
    trig_pin = _trig;
    echo_pin = _echo;
  } 

  int read_dist() {
    float duration_us, distance_cm;

    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);

    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trig_pin, LOW);
    duration_us  = pulseIn(echo_pin, HIGH);
     distance_cm = 0.017 * duration_us;

    return(distance_cm);
  }
}; // of Sensor class





class Tank {
  

  public:
  Motor left_motor;
  Motor right_motor;
  //Servo f_servo; //initialize a servo object
  //Servo b_servo; //initialize a servo object
  //Servo r_servo; //initialize a servo object
  //Servo l_servo; //initialize a servo object
  
  US_Sensor f_sensor;
  US_Sensor b_sensor;
  US_Sensor r_sensor;
  US_Sensor l_sensor;

  us_Servo f_servo; 
  us_Servo b_servo; 
  us_Servo r_servo; 
  us_Servo l_servo; 
  

  bool STDBY = false; // power consumption mode: TBD
  bool motors_on = false;

  void tank_init_motors(int _l_int1_pin,int _l_int2_pin, int _l_pwm_pin,int _l_pwm_channel, int _r_int1_pin,int _r_int2_pin, int _r_pwm_pin, int _r_pwm_channel,int _stby_pin) {  
    // the pwm_pin is for ESp8266/WEMOS style, pwm_channel is for ESP32 style    
    //pinMode(_stby_pin, OUTPUT);
    set_motors_off();
    //digitalWrite(_stby_pin,STDBY);
    left_motor.init(_l_int1_pin,_l_int2_pin,_l_pwm_pin,_l_pwm_channel);
    right_motor.init(_r_int1_pin,_r_int2_pin,_r_pwm_pin,_r_pwm_channel);
  }

   void set_motors_on() {
    motors_on = true;
    STDBY = false; // starting with stdby mode
    digitalWrite(STBY_pin,HIGH);
  }

  void set_motors_off() {
    motors_on = false;
    STDBY = true; // starting with stdby mode
    digitalWrite(STBY_pin,LOW);
  }

  void tank_init_servos(int _F_SERVO_PWM, int _B_SERVO_PWM, int _R_SERVO_PWM, int _L_SERVO_PWM) {
    f_servo.init(F_SERVO_Channel,F_SERVO_PWM_PIN,SERVO_FREQ,PWM_REOLUTION);
    b_servo.init(B_SERVO_Channel,B_SERVO_PWM_PIN,SERVO_FREQ,PWM_REOLUTION);
    r_servo.init(R_SERVO_Channel,R_SERVO_PWM_PIN,SERVO_FREQ,PWM_REOLUTION);
    l_servo.init(L_SERVO_Channel,L_SERVO_PWM_PIN,SERVO_FREQ,PWM_REOLUTION);


    //ledcSetup(F_SERVO_Channel, freq, resolution);
    //ledcAttachPin(F_SERVO_PWM_PIN, F_SERVO_Channel);
    //ledcWrite(F_SERVO_Channel, 255);

    //f_servo.attach(F_SERVO_PWM); // connect the servo with GPIO
    // b_servo.attach(B_SERVO_PWM); // connect the servo with GPIO
    // r_servo.attach(R_SERVO_PWM); // connect the servo with GPIO
    // l_servo.attach(L_SERVO_PWM); // connect the servo with GPIO
  }

  void tank_init_us_sensors() {
    f_sensor.init(F_TRIG_PIN,F_ECHO_PIN);
    b_sensor.init(B_TRIG_PIN,B_ECHO_PIN);
    r_sensor.init(R_TRIG_PIN,R_ECHO_PIN);
    l_sensor.init(L_TRIG_PIN,L_ECHO_PIN);
  }



// XXXXXXXXXXXXX
  void tank_stop() {
    left_motor.stop();
    right_motor.stop();
  }

  void tank_go_vector(int _x, int _y,int _button,int _range) {

    Serial.print("        in tank_go_vector: x/y/button ");
    Serial.print(_x);
    Serial.print("..  ");
    Serial.print(_y);
    Serial.print("..  ");
    Serial.print(_button);
    Serial.print("..  ");
    
    // go by vector recieved from jpystick;
    
    
    int x_speed = abs(_x);
    int y_speed = abs(_y);

    x_speed = map(x_speed,0,_range,0,255);
    y_speed = map(y_speed,0,_range,0,255);

    Serial.print("   x_speed / y_speed: x/y ");
    Serial.print(x_speed);
    Serial.print("..");
    Serial.print(y_speed);

    if (_x==0 && _y==0) {
      Serial.println("  stop");
      left_motor.stop();
      right_motor.stop();
      return;
    }

    if (_x==0 && _y>0) {
      Serial.println("  forward");
      Tank_forward(y_speed);      
      return;
    }

    if (_x==0 && _y<0) {
      Serial.println("  backward");
      Tank_backward(abs(y_speed));
      return;
    }

    if (_x>0 && _y==0) {
      Serial.println("  Right Pivot");
      Tank_right_pivot(x_speed);
      return;
    }

    if (_x<0 && _y==0) {
      Serial.println("  Left Pivot");
      Tank_left_pivot(abs(x_speed));
      return;
    }

  // now deal with vector moves
    if (_x>0 && _y>0) {
      Serial.println("  FWD R");
      // go FWD right
      Tank_forward_turn(x_speed+y_speed,y_speed);
      return;
    }

      if (_x<0 && _y>0) {
        Serial.println("  FWD L");
      // go FWD left
      Tank_forward_turn(y_speed,y_speed+x_speed);
      return;
    }

        if (_x>0 && _y<0) {
          Serial.println("  BWD R");
      // go BWD right
      Tank_backward_turn(x_speed+y_speed,y_speed);
      return;
    }

      if (_x<0 && _y<0) {
        Serial.println("  BWD L");
      // go BWD left
      Tank_backward_turn(y_speed,y_speed+x_speed);
      return;
    }

  Serial.println("  should NOT be here");

  } // of tank_go_vector


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
  int test_speed = 100;
  int l_speed = 100;
  int r_speed = 100;
  int test_time = 1000;
  int stop_time = 500;
  
  
  // test Forward
  Serial.println("in test_moves: Forward");
  Tank_forward(test_speed);
  delay(test_time);
  tank_stop();
  delay(stop_time);

  // test Backward
  Serial.println("in test_moves: Backward");
  Tank_backward(test_speed);
  delay(test_time);
  tank_stop();
  delay(stop_time);

  // test Left turn
  r_speed = 200;
  Serial.println("in test_moves: Left turn");
  Tank_forward_turn(l_speed,r_speed);
  delay(test_time);
  tank_stop();
  delay(stop_time);

  // test Right turn
  r_speed = 100;
  l_speed = 200;
  Serial.println("in test_moves: Right turn");
  Tank_forward_turn(l_speed,r_speed);
  delay(test_time);
  tank_stop();
  delay(stop_time);

  // test right pivot
  Serial.println("in test_moves: Right Pivot");
  Tank_right_pivot(test_speed);
  delay(test_time);
  tank_stop();
  delay(stop_time);

  // test left pivot
  Serial.println("in test_moves: Left Pivot");
  Tank_left_pivot(test_speed);
  delay(test_time);
  tank_stop();
  delay(stop_time);

} // of test_moves

// basic HW test, to make sure pins don;t create issues
// to delete when all works fine
void test_hw() {
  for (int i=1;i<4;i++) {
    digitalWrite(LED_MOV_pin,HIGH);
    delay(500);  
    digitalWrite(LED_MOV_pin,LOW);
    delay(400);  
  } // of test_hw()
  
return;


  set_motors_on();
// test the duty cycle - TBD
   Serial.print("In test_hw in TANK class. Testing duty_cycle:   ");
   for (int speed=0;speed<255;speed+=30) {
     Serial.print(speed);
     Serial.print("...");
     Tank_forward(speed);
     delay(2000);
   } // of 


   for (int speed=0;speed<255;speed+=30) {
     Serial.print(speed);
     Serial.print("...");
     Tank_backward(speed);
     delay(2000);
   }



   Serial.println("_____ end speed test");

return;

  int delay_time = 200;

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

void test_servos() {
  //test_servos();
  Serial.print("testing Servo: Front: ");
  test_servo(f_servo);
  Serial.println("testing Servo: Back");
  test_servo(b_servo);
  Serial.println("testing Servo: Right");
  test_servo(r_servo);
  Serial.println("testing Servo: Left");
  test_servo(l_servo);
}

void test_servo(us_Servo _servo_name) {
  for(int i = 3; i <= 30; i += 2) {
    //int val = map(i,0,255,8,25);


       //ledcWrite(F_SERVO_Channel, val);
       Serial.print(i);
       Serial.print("___");


    _servo_name.write_angle(i);
    delay(1500);
    _servo_name.write_angle(ZERO);
    }
    
    
    //_servo_name.write_angle(ZERO);
    //delay(100); // move from 180 to 0 degrees with a negative angle of 5 for(angle = 180; angle>=1; angle-=5)
  }


  void test_sensors() {
     Serial.println("testing Sensor: Front");
  test_sensor(f_sensor,5,200);
  Serial.println("testing Sensor: Back");
  test_sensor(b_sensor,10,200);
  Serial.println("testing Sensor: Right");
  test_sensor(r_sensor,10,200);
  Serial.println("testing Sensor: left");
  test_sensor(l_sensor,10,200);
  }

  void test_sensor(US_Sensor _S_Sensor, int _num,int _delay) {
    for (int i=1;i<=_num;i++) {
      int l_dist = _S_Sensor.read_dist();
      Serial.print("Sensor reading: ");
      Serial.println(l_dist);
      delay(_delay);
    }
    
  }


void  tank_test() {
  Serial.println("test tank moves: FW/BW/R/L/R pivot/L pivot");
  set_motors_on();
  test_moves();
  set_motors_off();
  delay(1000);

  test_sensors();
  test_servos();
  return;


  


}






}; // of TANK class