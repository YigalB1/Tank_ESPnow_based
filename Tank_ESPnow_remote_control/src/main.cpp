#include "WiFi.h"
#include <esp_now.h>

// GPIOs numbers on ESP32
const int XPin    = 36;
const int YPin    = 39;
const int PushButton  = 34;


class joystic {
  int range =15;
  public:
  int x_min_zero = 7777; // the min x still considered zero (becasue joystic is not 100% calibrated)
  int x_max_zero = -777; // the max x still considered zero
  int y_min_zero = 7777; // the min y still considered zero
  int y_max_zero = -777; // the max y still considered zero
  int Xval,Yval; // the final calibrated X Y (from -10 to +10) - range value
  bool Push_button_state = false;
  bool ON_state = false ;

  void estimate_joystic_zeros() {
    for (size_t i = 0; i < 100; i++)
    {
      int rdX = analogRead(XPin);
      int rdY = analogRead(YPin);
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
    int rdX = analogRead(XPin);
    int rdY = analogRead(YPin);

    int butt_tmp =  digitalRead(PushButton);

    if (butt_tmp == HIGH) {
      if (ON_state == true )
        ON_state = false;
      else
        ON_state = false;
    }

    Xval = 0;
    Yval = 0;
    
    if (rdX < x_min_zero ) {
      Xval = map(rdX,0,x_min_zero,-10,0); }
      else if (rdX > x_max_zero ) {
        Xval = map(rdX,x_max_zero,4095,0,10); }

    if (rdY < x_min_zero ) {
      Yval = map(rdY,0,y_min_zero,-10,0); }
      else if (rdY > y_max_zero ) {
        Yval = map(rdY,y_max_zero,4095,0,10); }

  }

}; // of class JOYSTICK




// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x24,0x0A,0xC4,0x59,0x41, 0x70};

// Structure example to send data
// Must match the receiver structure
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

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

joystic tank_joystick;


// *** start SETUP
void setup() {
  Serial.begin(9600);
  delay(1000);
  pinMode(PushButton, INPUT); // GPIO as input




 
  // *******************  init ESPnow *****************
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


  tank_joystick.estimate_joystic_zeros();

} // of SETUP

void loop() {


  tank_joystick.read_jostick();
  Serial.print("   button:  ");
  Serial.print(tank_joystick.ON_state);
  Serial.print(" X/Y:   ");
  Serial.print(tank_joystick.Xval);
  Serial.print(" / ");
  Serial.println(tank_joystick.Yval);

  delay(300);
  return;

  // sens the data to the tank
  strcpy(myData.ctrl_msg, "I am the joystic");
  myData.x_val = tank_joystick.Xval;
  myData.y_val = tank_joystick.Yval;
  myData.button_state = tank_joystick.Push_button_state;
    
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }


  delay(300);
  


}