// logging
#include <esp_log.h>
static const char* TAG = "main";
// libraries
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>
#include <SPIFFS.h>
#include <secrets.h>
#include <ArduinoJson.h>

//pinout
#define AUTO_RELAY 18
#define VENT_RELAY 17
#define COMP_RELAY 16
#define NETWORK_LED 2
#define Q3_INPUT 14
#define Q2_INPUT 27
#define Q1_INPUT 26
#define RETURN_TEMP 25
#define SUPPLY_TEMP 33
#define FLOAT_SWTCH 32
#define NOW_CNF 35
// Set your Board ID 
#define BOARD_ID 1
#define MAX_CHANNEL 11  // for North America

// oneWire and DallasTemperature
OneWire ow_return_temp(RETURN_TEMP);
OneWire ow_supply_temp(SUPPLY_TEMP);
DallasTemperature air_return_sensor(&ow_return_temp);
DallasTemperature air_supply_sensor(&ow_supply_temp);
int tempSensorResolution = 12; //bits
int tempRequestDelay = 0;
float air_return_temp = 0;
float air_supply_temp = 0;

// esp-now variables
uint8_t server_mac_address[] = { 0 };
int channel = 1; // esp-now communication channel.
String server_id = "";
String deviceID = "";

enum MessageType {PAIRING, DATA,};
enum SenderID {SERVER, CONTROLLER, MONITOR,};
enum SystemModes {SYS_OFF, SYS_FAN, SYS_COOL, SYS_AUTO_CL,};
enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
PairingStatus pairingStatus = NOT_PAIRED;
MessageType espnow_msg_type;
SystemModes espnow_system_mode;
SenderID espnow_peer_id;

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

// time vars.
unsigned long lastTempRequest = 0;
unsigned long currentMillis = millis();
unsigned long previousMillis = 0;   // Stores last time temperature was published
unsigned long start;                // used to measure Pairing time
const unsigned long interval = 5000;// Interval at which to publish sensor readings - 5' seconds

//logger function
void debug_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "%s", message);
}

void info_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "%s", message);
}

void error_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "%s", message);
}

// función que carga una String que contiene toda la información dentro del target_file del SPIFFS.
String load_data_from_fs(const char *target_file) {
  //-
  File f = SPIFFS.open(target_file);
  if (!f) {
    error_logger("Error al abrir el archivo solicitado.");
    return "null";
  }

  String file_string = f.readString();
  f.close();

  //log
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "data loaded from SPIFFS: %s", file_string.c_str());
  return file_string;
}

// función que guarda una String en el target_file del SPIFFS.
void save_data_in_fs(String data_to_save, const char *target_file) {
  //savin data in filesystem.
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "saving in:%s this data:%s", target_file, data_to_save.c_str());

  File f = SPIFFS.open(target_file, "w");
  if (!f){
    error_logger("Error al abrir el archivo solicitado.");
    return;
  }

  f.print(data_to_save);
  f.close();
  info_logger("data saved correctly in SPIFFS.");
  return;
}

//-functions
bool temp_is_valid(float measurement) {
  char message_buffer[40];
  char name[8] = "ds18B20";

  if (measurement == -127) {
    sprintf(message_buffer, "Error -127; [%s]", name);
    error_logger(message_buffer);
    return false;
  }
  return true;
}

void update_temperatures()
{
  if (millis() - lastTempRequest >= tempRequestDelay) {
    //-
    float returnBuffer = air_return_sensor.getTempCByIndex(0);
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Return temperature: %3.2f °C", returnBuffer);
    float supplyBuffer = air_supply_sensor.getTempCByIndex(0);
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Supply temperature: %3.2f °C", returnBuffer);
    if (!temp_is_valid(returnBuffer) || !temp_is_valid(supplyBuffer))
    {
      error_logger("invalid ds18b20 redings.");
      return;
    }
    // update global variable.
    air_return_temp = returnBuffer;
    air_return_sensor.requestTemperatures();
    //-
    air_supply_temp = supplyBuffer;
    air_supply_sensor.requestTemperatures();
    //- set timer
    lastTempRequest = millis();
  }
  return;
}

void update_IO()
{
  // to-do: function tu update inputs and outputs.
  return;
}

//- mac address parse
bool str2mac(const char* mac, uint8_t* values){
    if(6 == sscanf(mac,"%02x:%02x:%02x:%02x:%02x:%02x",&values[0], &values[1], &values[2],&values[3], &values[4], &values[5])){
        return true;
    }else{
        return false;
    }
}

//- load server mac address from filesystem.
void load_server_from_fs() {
  const char test_mac[] = "FF:AA:05:24:33:24";
  channel = 1; // channel also is stored in fs.
  uint8_t mac_buffer[6] = { 0 };
  // fs string: "{server_id: FFFFFFFFFFFF, server_mac: FF:FF:FF:FF:FF:FF, server_chan: 1}"
  //-
  const bool parse_success = str2mac(test_mac, mac_buffer);
  if (parse_success) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "Server Mac Address: %02X:%02X:%02X:%02X:%02X:%02X", 
    mac_buffer[0], mac_buffer[1], mac_buffer[2], mac_buffer[3], mac_buffer[4], mac_buffer[5]);

    //- update server_mac_address variable
    memcpy(server_mac_address, mac_buffer, sizeof(mac_buffer));

  } else {
    error_logger("fail to load server mac address from file system.");
  }
}

//- *esp_now functions
void addPeer(const uint8_t * mac_addr, uint8_t chan){
  debug_logger("adding new peer to peer list.");
  esp_now_peer_info_t peer;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK){
    error_logger("esp peer could'n be added");
    return;
  }
  memcpy(server_mac_address, mac_addr, sizeof(uint8_t[6]));
}

//get device id from mac address.
String get_device_id(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x%02x%02x%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

// print mac address.
String print_mac(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  info_logger("Last Packet Send Status:");
  info_logger(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print("Packet received from: ");
  get_device_id(mac_addr);
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
      get_device_id(mac_addr);
      Serial.print("Pairing done for ");
      get_device_id(pairing_data.macAddr);
      Serial.print(" on channel " );
      Serial.print(pairing_data.channel);    // channel used by the server
      Serial.print(" in ");
      Serial.print(millis()-start);
      Serial.println("ms");
      addPeer(pairing_data.macAddr, pairing_data.channel); // add the server  to the peer list 
      pairingStatus = PAIR_PAIRED;             // set the pairing status
    }
    break;
  }
}

PairingStatus autoPairing(){
  switch(pairingStatus) {
    case PAIR_REQUEST:
    ESP_LOG_LEVEL(LOG_LOCAL_LEVEL, TAG, "Pairing request on channel: %d", channel);
    // set WiFi channel   
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
  
    // set pairing data to send to the server
    pairing_data.msg_type = PAIRING;
    pairing_data.sender_id = BOARD_ID;     
    pairing_data.channel = channel;

    // add peer and send request
    addPeer(server_mac_address, channel);
    esp_now_send(server_mac_address, (uint8_t *) &pairing_data, sizeof(pairing_data));
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

// -setup
void setup() {
  // SETUP BEGIN
  esp_log_level_set("*", ESP_LOG_DEBUG); 
  Serial.begin(115200); 
  debug_logger("** setup start. **");

  //PINS SETUP
  info_logger("pins settings");
  pinMode(AUTO_RELAY, OUTPUT);
  pinMode(VENT_RELAY, OUTPUT);
  pinMode(COMP_RELAY, OUTPUT);
  pinMode(NETWORK_LED, OUTPUT);
  pinMode(Q3_INPUT, INPUT_PULLUP);
  pinMode(Q2_INPUT, INPUT_PULLUP);
  pinMode(Q1_INPUT, INPUT_PULLUP);
  pinMode(NOW_CNF, INPUT);
  info_logger("pins settings done.");

  // SPIFFS SETUP
  if(!SPIFFS.begin(true)) {
    error_logger("Ocurrió un error al iniciar SPIFFS., reboot.");
    while (1) {;}
  }

  // OneWire SETUP
  info_logger("OneWire sensors settings!");
  air_return_sensor.begin();
  air_return_sensor.setResolution(tempSensorResolution);
  air_return_sensor.setWaitForConversion(false);
  air_supply_sensor.begin();
  air_supply_sensor.setResolution(tempSensorResolution);
  air_supply_sensor.setWaitForConversion(false);
  //first temp. request.
  air_return_sensor.requestTemperatures();
  air_supply_sensor.requestTemperatures();
  lastTempRequest = millis();
  tempRequestDelay = 750 / (1 << (12 - tempSensorResolution));
  info_logger("OneWire sensors settings done.");

  //- WiFi SETUP
  info_logger("WiFi settings.");
  WiFi.mode(WIFI_AP);
  info_logger("MAC Address:  ->");
  info_logger(WiFi.macAddress().c_str());
  info_logger("WiFi settings done.");

  //- ESP-NOW SETUP
  info_logger("esp-now settings");
  uint8_t mac_buffer[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_AP, mac_buffer);
  if (ret != ESP_OK) {
    error_logger("could not read mac address.");
  }
  deviceID = get_device_id(mac_buffer);
  //-
  if (esp_now_init() != ESP_OK) {
    error_logger("-- Error initializing ESP-NOW, please reboot --");
    while (1){;}
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  info_logger("esp-now settings done.");
  start = millis();
  pairingStatus = PAIR_REQUEST;

  //LOAD DATA FROM FS
  // load_server_from_fs();

  // SETUP FINISHED
  info_logger("setup finished --!.");
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
      outgoing_data.evap_air_in_temp = air_return_temp;
      outgoing_data.evap_air_out_temp = air_supply_temp;
      esp_err_t result = esp_now_send(server_mac_address, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
    }
  }
}