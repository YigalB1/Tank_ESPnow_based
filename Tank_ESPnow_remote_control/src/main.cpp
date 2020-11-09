#include "WiFi.h"
#include <esp_now.h>
#include <classes.h>

// GPIOs numbers on ESP32
const int XPin    = 36;
const int YPin    = 39;
const int PushButton_PIN  = 34;
const int LED1 = 18;
const int LED2 = 5;
const int LED3 = 4;

int mouse_range = 12; // the max of the mouse change


// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0x24,0x0A,0xC4,0x59,0x41, 0x70};
//uint8_t broadcastAddress[] = {0x10,0x52,0x1c,0x66,0xc4, 0x4c}; // pcb tank
uint8_t broadcastAddress[] = {0x24,0x0A,0xC4,0x59,0x41, 0x70}; // pcb 2

// Structure to send data, must match the receiver structure
typedef struct struct_message {
  int x_val;
  int y_val;
  bool button_state;
  char ctrl_msg[32];
  //float c;
  //String d;
} struct_message;

struct_message myData;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

joystic tank_joystick;

void test_joystick();   // debug routine
void print_it();        // debug prints
void test_hw();


void IRAM_ATTR Butt_pressed_isr() {
  tank_joystick.butt_pressed = true;
  //Serial.println("...Pressed...");
  }

// *************** SETUP ****************************
void setup() {
  Serial.begin(9600);
  //delay(1000);
  // pinMode(PushButton, INPUT_PULLUP); // GPIO as input

  pinMode(PushButton_PIN, INPUT); // GPIO as input // Problem - not a good GPIO, no pullup. 
                                  // solution: add pullup on PCB. mext version: change pin?
  pinMode(LED1, OUTPUT); 
  pinMode(LED2, OUTPUT); 
  pinMode(LED3, OUTPUT); 

  test_hw();


  attachInterrupt(PushButton_PIN, Butt_pressed_isr, FALLING);

  // init ESPnow 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }



  // Before start, get X & Y values to be considered as zero,
  // due to non exact readings
  // then translate X & Y  into values from -10 to +10
  tank_joystick.set_hw(XPin,YPin,PushButton_PIN,mouse_range);
  tank_joystick.estimate_joystic_zeros();
} // of SETUP



void loop() {

// test_joystick(); // just for testing

  tank_joystick.read_jostick();
  // transmit because joystick reading has been changed
    // sens the data to the tank
    strcpy(myData.ctrl_msg, "I am the joystic");
    myData.x_val = tank_joystick.Xval;
    myData.y_val = tank_joystick.Yval;
    myData.button_state = tank_joystick.butt_pressed;

    Serial.print("  ");
    Serial.print(" Sending-  X: ");
    Serial.print(tank_joystick.Xval);
    Serial.print(" Y: ");
    Serial.print(tank_joystick.Yval);
    Serial.print(" Button: ");
    Serial.print(tank_joystick.butt_pressed);
    Serial.println("   ^^^^^   ");

      
    // Send message via ESP-NOW, only if x & y are both not zero
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.print(".. ..");
    }
    else {
      Serial.println("");
      Serial.println("Error sending the data");
    }

    tank_joystick.butt_pressed = false;

    delay(100);

    return;





  // print_it();

  //if ((tank_joystick.Xval!=tank_joystick.prev_x) || (tank_joystick.Yval!=tank_joystick.prev_y)) {
  if (tank_joystick.change_occured) {
    Serial.print(" in loop, change occured ------>    ");
    print_it();



    // transmit because joystick reading has been changed
    // sens the data to the tank
    strcpy(myData.ctrl_msg, "I am the joystic");
    myData.x_val = tank_joystick.Xval;
    myData.y_val = tank_joystick.Yval;
    myData.button_state = tank_joystick.butt_pressed;

      
    // Send message via ESP-NOW, only if x & y are both not zero
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
      Serial.println("");
      Serial.println("Sent with success   ");
    }
    else {
      Serial.println("");
      Serial.println("Error sending the data");
    }
  } // of long if


  //delay(300);
  unsigned long time_now = 0;
  int period = 0;
  time_now = millis();      
  while(millis() < time_now + period){
        //wait approx. [period] ms
  }
  


} // of loop



void test_joystick() {
  // read endlessly joystick input, for testing
  int mid = 2047;
  int max_delta_x = 0;
  int max_delta_y = 0;

  Serial.println("don't touch at first - to calibrate drift ");

  while (true) {
    int t_rdX = analogRead(XPin);
    int t_rdY = analogRead(YPin);
    int butt_tmp =  digitalRead(PushButton_PIN);

    if (abs(t_rdX-mid) > max_delta_x )
      max_delta_x = abs(t_rdX-mid);

    if (abs(t_rdY-mid) > max_delta_y )
      max_delta_y = abs(t_rdY-mid);

    Serial.print("   X: ");
    Serial.print(t_rdX);
    Serial.print("   Y: ");
    Serial.print(t_rdY);
    Serial.print("  SW: ");
    Serial.print(butt_tmp);
    Serial.print("       Max X drift:  ");
    Serial.print(max_delta_x);
    Serial.print("       Max Y drift:  ");
    Serial.println(max_delta_y);
    delay(300);
  }
} // of test_joystick()





void print_it() { // DEBUG

  if (tank_joystick.change_occured) {
    Serial.print(" change!   ");  
  }
  else {
    Serial.print("           ");  
  }
  Serial.print("X/Y     ");
  Serial.print(tank_joystick.Xval);
  Serial.print(" / ");
  Serial.print(tank_joystick.Yval);
   Serial.print("      prevX/PrevY   ");
  Serial.print(tank_joystick.prev_x);
  Serial.print(" / ");
  Serial.println(tank_joystick.prev_y);

} // of print_it()

void test_hw() {
  for (int i=0;i<3;i++) {
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    digitalWrite(LED3,LOW);
    delay(400);
    digitalWrite(LED1,HIGH);
    delay(400);
    digitalWrite(LED2,HIGH);
    digitalWrite(LED1,LOW);
    delay(400);
    digitalWrite(LED3,HIGH);
    digitalWrite(LED2,LOW);
    delay(400);
  }
  digitalWrite(LED3,LOW);
  

}