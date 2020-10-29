#include "WiFi.h"

class joystic {
  int range = 20;

  public:
  int x_min_zero = 7777; // the min x still considered zero (becasue joystic is not 100% calibrated)
  int x_max_zero = -777; // the max x still considered zero
  int y_min_zero = 7777; // the min y still considered zero
  int y_max_zero = -777; // the max y still considered zero
  int Xval = 0; // the final calibrated X Y (from -10 to +10) - range value
  int Yval = 0;
  int prev_x ; // keep previous joystic state, transmit only changes
  int prev_y ;
  bool prev_butt;
  //bool Push_button_state = false;
  bool ON_state = false ;
  bool change_occured; // is this read different than previous?
  int Xpin,Ypin,PushButtpin; // the hardwrae pins of the ESP32
  int mouse_mov_range;
  bool butt_pressed = false; // indicates if joystic's button was pressed


  void set_hw(int _xpin,int _ypin, int _pushbutton, int _mouse_range) {
    Xpin = _xpin;
    Ypin = _ypin;
    PushButtpin = _pushbutton;
    mouse_mov_range = _mouse_range;
  }
  
  

  void estimate_joystic_zeros() {
    for (size_t i = 0; i < 100; i++)
    {
      
      int rdX = analogRead(Xpin);
      int rdY = analogRead(Ypin);
      if (rdX > x_max_zero)
        x_max_zero = rdX;
      if (rdY > y_max_zero)
        y_max_zero = rdY;
      if (rdX < x_min_zero)
        x_min_zero = rdX;
      if (rdY < y_min_zero)
        y_min_zero = rdY;
    } // of for loop
    
    // add/sub 10 for safety
    x_min_zero -= range; 
    x_max_zero += range; 
    y_min_zero -= range; 
    y_max_zero += range;

  } // of estimate_joystic_zeros

  void read_jostick() {
    prev_x = Xval;
    prev_y = Yval;

    int t_rdX = analogRead(Xpin);
    int t_rdY = analogRead(Ypin);
   
   // read of putton is done by interrupt
   /*
    int butt_tmp =  digitalRead(PushButtpin);

    if (butt_tmp == HIGH) {
      if (ON_state == true )
        ON_state = false;
      else
        ON_state = false;
    }
    */ 

   Serial.println("  ");
   Serial.print("t_rdX: ");
   Serial.print(t_rdX);
   Serial.print("  t_rdY: ");
   Serial.print(t_rdY);
   Serial.print("  Button: ");
   Serial.print(butt_pressed);


    Xval = 0;    
    if (t_rdX < x_min_zero ) {
      Xval = map(t_rdX,0,x_min_zero,-mouse_mov_range,0); }
      else if (t_rdX > x_max_zero ) {
        Xval = map(t_rdX,x_max_zero,4095,0,mouse_mov_range); }

    Yval = 0;
    if (t_rdY < y_min_zero ) {
      Yval = map(t_rdY,0,y_min_zero,-mouse_mov_range,0); }
      else if (t_rdY > y_max_zero ) {
        Yval = map(t_rdY,y_max_zero,4095,0,mouse_mov_range); }



  
   Serial.print("        Xval: ");
   Serial.print(Xval);
   Serial.print("  Yval: ");
   Serial.print(Yval);
   Serial.print("  Button: ");
   Serial.print(butt_pressed);
   

    
    if ((Xval != prev_x) || (Yval != prev_y) || (butt_pressed)) {
      change_occured = true;
      Serial.print("  ------   change occured ...... in class, read_jostick    ");
      //print_it();
    }      
    else
      change_occured = false;
  }
}; // of class JOYSTICK

