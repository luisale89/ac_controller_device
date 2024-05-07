#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>

// Set your Board and Server ID 
#define BOARD_ID 1
#define MAX_CHANNEL 11  // for North America
#define BUILTIN_LED 2

uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

enum MessageType {PAIRING, DATA,};
enum SenderID {SERVER, CONTROLLER, MONITOR,};
enum SystemModes {SYS_OFF, SYS_FAN, SYS_COOL, SYS_AUTO_CL,};
enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
PairingStatus pairingStatus = NOT_PAIRED;
MessageType espnow_msg_type;
SystemModes espnow_system_mode;
SenderID espnow_peer_id;

//Structure to send data
//Must match the receiver structure
// Structure example to receive data
// Must match the sender structure
typedef struct controller_data_struct {
  uint8_t msg_type;             // (1 byte)
  uint8_t sender_id;            // (1 byte)
  uint8_t active_system_mode;   // (1 byte)
  uint8_t fault_code;           // (1 byte) 0-no_fault; 1..255 controller_fault_codes.
  float evap_vapor_line_temp;   // (4 bytes)
  float evap_air_in_temp;       // (4 bytes)
  float evap_air_out_temp;      // (4 bytes)
} controller_data_struct;       // TOTAL = 21 bytes

typedef struct incoming_settings_struct {
  uint8_t msg_type;             // (1 byte)
  uint8_t sender_id;            // (1 byte)
  uint8_t fault_restart;        // (1 byte)
  uint8_t system_setpoint;      // (1 byte)
  uint8_t system_mode;          // (1 byte)
  uint8_t system_state;         // (1 byte)
  uint8_t room_temp;            // (1 byte)
} incoming_settings_struct;     // TOTAL = 7 bytes

typedef struct pairing_data_struct {
  uint8_t msg_type;             // (1 byte)
  uint8_t sender_id;            // (1 byte)
  uint8_t macAddr[6];           // (6 bytes)
  uint8_t channel;              // (1 byte)
} pairing_data_struct;          // TOTAL = 9 bytes

//Create 2 struct_message 
controller_data_struct outgoing_data;  // data to send
incoming_settings_struct incoming_data;  // data received
pairing_data_struct pairing_data;

#ifdef SAVE_CHANNEL
  int lastChannel;
#endif  
int channel = 1;
 
// simulate temperature and humidity data
float t = 0;
float h = 0;

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;   // Stores last time temperature was published
const unsigned long interval = 5000;        // Interval at which to publish sensor readings - 5' seconds
unsigned long start;                // used to measure Pairing time

// simulate temperature reading
float readDHTTemperature() {
  t = random(0,40);
  return t;
}

// simulate humidity reading
float readDHTHumidity() {
  h = random(0,100);
  return h;
}

void addPeer(const uint8_t * mac_addr, uint8_t chan){
  esp_now_peer_info_t peer;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print("Packet received from: ");
  printMAC(mac_addr);
  Serial.println();
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
  uint8_t type = incomingData[0];
  switch (type) {
  case DATA :      // we received data from server
    memcpy(&incoming_data, incomingData, sizeof(incoming_data));
    Serial.print("ID  = ");
    Serial.println(incoming_data.sender_id);
    Serial.print("fault restart = ");
    Serial.println(incoming_data.fault_restart);
    Serial.print("Setpoint temp = ");
    Serial.println(incoming_data.system_setpoint);
    Serial.println("System mode = ");
    Serial.println(incoming_data.system_mode);

  case PAIRING:    // we received pairing data from server
    memcpy(&pairing_data, incomingData, sizeof(pairing_data));
    if (pairing_data.sender_id == 0) {              // the message comes from server
      printMAC(mac_addr);
      Serial.print("Pairing done for ");
      printMAC(pairing_data.macAddr);
      Serial.print(" on channel " );
      Serial.print(pairing_data.channel);    // channel used by the server
      Serial.print(" in ");
      Serial.print(millis()-start);
      Serial.println("ms");
      addPeer(pairing_data.macAddr, pairing_data.channel); // add the server  to the peer list 
      #ifdef SAVE_CHANNEL
        lastChannel = pairing_data.channel;
        EEPROM.write(0, pairing_data.channel);
        EEPROM.commit();
      #endif  
      pairingStatus = PAIR_PAIRED;             // set the pairing status
    }
    break;
  }
}

PairingStatus autoPairing(){
  switch(pairingStatus) {
    case PAIR_REQUEST:
    Serial.print("Pairing request on channel "  );
    Serial.println(channel);

    // set WiFi channel   
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
    }

    // set callback routines
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
  
    // set pairing data to send to the server
    pairing_data.msg_type = PAIRING;
    pairing_data.sender_id = BOARD_ID;     
    pairing_data.channel = channel;

    // add peer and send request
    addPeer(serverAddress, channel);
    esp_now_send(serverAddress, (uint8_t *) &pairing_data, sizeof(pairing_data));
    previousMillis = millis();
    pairingStatus = PAIR_REQUESTED;
    break;

    case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis();
    if(currentMillis - previousMillis > 1000) { // 1 second for server response.
      previousMillis = currentMillis;
      // time out expired,  try next channel
      channel ++;
      if (channel > MAX_CHANNEL){
         channel = 1;
      }   
      pairingStatus = PAIR_REQUEST;
    }
    break;

    case PAIR_PAIRED:
      // nothing to do here 
    break;
  }
  return pairingStatus;
}  

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  start = millis();

  #ifdef SAVE_CHANNEL 
    EEPROM.begin(10);
    lastChannel = EEPROM.read(0);
    Serial.println(lastChannel);
    if (lastChannel >= 1 && lastChannel <= MAX_CHANNEL) {
      channel = lastChannel; 
    }
    Serial.println(channel);
  #endif  
  pairingStatus = PAIR_REQUEST;
}  

void loop() {
  if (autoPairing() == PAIR_PAIRED) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // Save the last time a new reading was published
      previousMillis = currentMillis;
      //Set values to send
      outgoing_data.msg_type = DATA;
      outgoing_data.sender_id = BOARD_ID;
      outgoing_data.evap_air_in_temp = readDHTTemperature();
      outgoing_data.evap_air_out_temp = readDHTHumidity();
      esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
    }
  }
}