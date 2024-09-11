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
int radio_channel = 1; // esp-now communication channel.
String server_serial = "";
String self_serial = "";

enum PeerRoleID {SERVER, CONTROLLER, MONITOR_A, MONITOR_B, ROLE_UNSET};
enum PairingStatusEnum {PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED, NOT_PAIRED};
enum MessageTypeEnum {PAIRING, DATA,};
enum SysModeEnum {AUTO_MODE, FAN_MODE, COOL_MODE};
enum SysStateEnum {SYSTEM_ON, SYSTEM_OFF, SYSTEM_SLEEP, UNKN};
//-vars
PairingStatusEnum pairingStatus = NOT_PAIRED;
MessageTypeEnum espnow_msg_type;
SysModeEnum espnow_system_mode;

typedef struct controller_data_struct {
  MessageTypeEnum msg_type;// (1 byte)
  PeerRoleID sender_role;  // (1 byte)
  uint8_t fault_code;      // (1 byte)
  float air_return_temp;   // (4 bytes) [°C]
  float air_supply_temp;   // (4 bytes) [°C]
  bool drain_switch;       // (1 byte)
  bool cooling_relay;      // (1 byte)
  bool turbine_relay;      // (1 byte)
} controller_data_struct;  // TOTAL = 14 bytes

typedef struct incoming_settings_struct {
  MessageTypeEnum msg_type;     // (1 byte)
  PeerRoleID sender_role;       // (1 byte)
  SysModeEnum system_mode;      // (1 byte)
  SysStateEnum system_state;    // (1 byte)
  float system_temp_sp;         // (4 bytes) [°C]
  float room_temp;              // (4 bytes) [°C]
} incoming_settings_struct;     // TOTAL = 7 bytes

typedef struct pairing_data_struct {
  MessageTypeEnum msg_type;     // (1 byte)
  PeerRoleID sender_role;       // (1 byte)
  PeerRoleID device_new_role;   // (1 byte)
  uint8_t channel;              // (1 byte) - 0 is default, let this value for future changes.
} pairing_data_struct;          // TOTAL = 9 bytes

//Create 2 struct_message 
controller_data_struct outgoing_data;  // data to send
incoming_settings_struct incoming_data;  // data received from server
pairing_data_struct pairing_data;

// time vars.
unsigned long lastTempRequest = 0;
unsigned long lastEspnowPost = 0;   // Stores last time temperature was published
unsigned long lastPairingRequest = 0;
unsigned long currentMillis = millis();
unsigned long start;                // used to measure Pairing time
const unsigned long espnowPostInterval = 5000;// Interval at which to publish sensor readings - 5' seconds

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

// print mac address.
String print_mac(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

//- mac address parser
bool str2mac(const char* mac, uint8_t* values){
    if(6 == sscanf(mac,"%02x:%02x:%02x:%02x:%02x:%02x",&values[0], &values[1], &values[2],&values[3], &values[4], &values[5])){
        return true;
    }else{
        return false;
    }
}

//get device id from mac address.
String get_device_serial(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x%02x%02x%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

void save_server_in_fs(const uint8_t *new_mac_address) {
  //- save server info in spiffs.
  info_logger("[esp-now] saving new server data in the fs.");
  JsonDocument server_json;
  String data;
  String new_server_serial = get_device_serial(new_mac_address);

  //update global variables.
  memcpy(server_mac_address, new_mac_address, sizeof(uint8_t[6]));
  server_serial = new_server_serial;

  server_json["server_serial"] = new_server_serial;
  server_json["server_mac"] = print_mac(new_mac_address);
  server_json["server_chan"] = radio_channel;

  serializeJson(server_json, data);
  save_data_in_fs(data, "now_server.txt");

  return;
}


//- load server mac address from filesystem.
void load_server_from_fs() {
  info_logger("[esp-now] loading server data from fs.");
  JsonDocument server_json;
  String server_data = load_data_from_fs("now_server.txt");
  DeserializationError error = deserializeJson(server_json, server_data);

  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "server_json Deserialization error raised with code: %s", error.c_str());
    return;
  }

  const char *server_ser = server_json["server_serial"] | "null";
  const char *server_mac = server_json["server_mac"] | "null";
  const int server_chan = server_json["server_chan"] | 1;

  if (strcmp(server_ser, "null") == 0) {
    error_logger("server id not found in fs.");
    return;
  }
  if (strcmp(server_mac, "null") == 0) {
    error_logger("server mac addres not found in fs.");
    return;
  }

  uint8_t mac_buffer[6] = { 0 };
  // fs string: "{server_id: ffffffffffff, server_mac: FF:FF:FF:FF:FF:FF, server_chan: 1}"
  //-
  if (str2mac(server_mac, mac_buffer)) {
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "Server Mac Address: %02X:%02X:%02X:%02X:%02X:%02X", 
    mac_buffer[0], mac_buffer[1], mac_buffer[2], mac_buffer[3], mac_buffer[4], mac_buffer[5]);

    //- update server variables.
    memcpy(server_mac_address, mac_buffer, sizeof(mac_buffer));
    server_serial = (char*) server_ser;
    radio_channel = server_chan;

  } else {
    error_logger("fail to parse server mac address from file system.");
  }

  return;
}

//-functions
bool is_valid_temp(float measurement) {
  char message_buffer[40];
  char name[8] = "ds18B20";

  if (measurement == -127) {
    sprintf(message_buffer, "Error -127; [%s]", name);
    error_logger(message_buffer);
    return false;
  }
  return true;
}

void update_temperature_readings()
{
  if (millis() - lastTempRequest >= tempRequestDelay) {
    //-
    float returnBuffer = air_return_sensor.getTempCByIndex(0);
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "Return temperature: %3.2f °C", returnBuffer);
    float supplyBuffer = air_supply_sensor.getTempCByIndex(0);
    ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "Supply temperature: %3.2f °C", supplyBuffer);
    if (!is_valid_temp(returnBuffer) || !is_valid_temp(supplyBuffer))
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

//- *esp_now functions
bool add_peer_to_plist(const uint8_t * mac_addr){
  info_logger("[esp-now] adding new peer to peer list.");
  //-
  esp_now_peer_info_t peerTemplate;
  memset(&peerTemplate, 0, sizeof(esp_now_peer_info_t));
  //create reference to peerTemplate memory loc.
  const esp_now_peer_info_t *peer = &peerTemplate;

  //-set data
  memcpy(peerTemplate.peer_addr, mac_addr, sizeof(uint8_t[6]));
  peerTemplate.channel = radio_channel;
  peerTemplate.encrypt = false;

  // check if the peer exists and remove it from peerlist
  if (esp_now_is_peer_exist(mac_addr)) {
    // Slave already paired.
    info_logger("! peer already exists, deleting existing data.");
    esp_err_t deleteStatus = esp_now_del_peer(mac_addr);
    if (deleteStatus == ESP_OK) {
      info_logger("+ peer deleted!");
    } else {
      error_logger("* error deleting peer!");
      return false;
    }
  }
  // save peer in peerlist
  esp_err_t addPeerResult = esp_now_add_peer(peer);
  switch (addPeerResult)
  {
  case ESP_OK:
    info_logger("new peer added successfully");
    return true;
  
  default:
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "Error: %d, while adding new peer.", addPeerResult);
    return false;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //- callback.
  String device_id = get_device_serial(mac_addr);
  switch (status)
  {
  case ESP_NOW_SEND_SUCCESS:
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] packet to: %s has been sent!", device_id.c_str());
    break;
  
  default:
  ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[esp-now] packet to: %s not sent.", device_id.c_str());
    break;
  }
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  //-
  String device_serial = get_device_serial(mac_addr);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] %d bytes of data received from: %s", len, device_serial.c_str());

  //only accept messages from server device.
  if (strcmp(device_serial.c_str(), server_serial.c_str()) != 0) //don't match
  {
    info_logger("[esp-now] msg from invalid device, receiving from server only.");
    return;
  }

  uint8_t message_type = incomingData[0];
  uint8_t device_id = incomingData[1];
  if (device_id != SERVER) {
    info_logger("[esp-now] invalid device ID. ignoring message");
    return;
  }

  JsonDocument root;
  String payload;

  //get message_type message from first byte.
  switch (message_type) {
  case DATA :      // we received data from server
    memcpy(&incoming_data, incomingData, sizeof(incoming_data));
    root["system_mode"] = incoming_data.system_mode;
    root["system_state"] = incoming_data.system_state;
    root["sts_temp_sp"] = incoming_data.system_temp_sp;
    root["room_temp"] = incoming_data.room_temp;
    serializeJson(root, payload);
    debug_logger(payload.c_str());


  case PAIRING:    // received pairing data from server
    memcpy(&pairing_data, incomingData, sizeof(pairing_data));
    if (!add_peer_to_plist(mac_addr)){
      error_logger("[esp-now] peer couldn't be saved, try again.");
      break;
    } 
    
    // add the server  to the peer list
    save_server_in_fs(mac_addr); // update server info in fs.
    
    // device PAIRED with server.
    pairingStatus = PAIR_PAIRED;  // set the pairing status
    break;
  }
}

void espnow_loop(){

  // switch modes.
  switch(pairingStatus) {
    // PAIR REQUEST
    case PAIR_REQUEST:
    ESP_LOG_LEVEL(LOG_LOCAL_LEVEL, TAG, "Sending pairing request on channel: %d", radio_channel);
  
    // set pairing data to send to the server
    pairing_data.msg_type = PAIRING;
    pairing_data.sender_role = ROLE_UNSET;
    pairing_data.channel = radio_channel;

    // add peer and send request
    if (!add_peer_to_plist(server_mac_address)){
      error_logger("[esp-now] couldn't add peer to peer list.");
      break;
    }

    //- send pair data.
    esp_now_send(NULL, (uint8_t *) &pairing_data, sizeof(pairing_data));
    lastPairingRequest = millis();
    pairingStatus = PAIR_REQUESTED;
    break;

    // PAIR REQUESTED
    case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis();
    if(currentMillis - lastPairingRequest > 1000) { // 1 second for server response.
      lastPairingRequest = currentMillis;
      info_logger("[autopairing] time out expired, try next channel");
      radio_channel ++;
      if (radio_channel > MAX_CHANNEL){
         radio_channel = 1;
      }
      // set WiFi channel   
      ESP_ERROR_CHECK(esp_wifi_set_channel(radio_channel,  WIFI_SECOND_CHAN_NONE));
      pairingStatus = PAIR_REQUEST;
    }
    break;

    // NOT PAIRED
    case NOT_PAIRED:
      // waiting for now button press to begin pair process. 
      // starts with the default address (ffx6) and wait for the response
      // to save the correct mac_address.
    break;
    
    // PAIRED
    case PAIR_PAIRED:
    //- posting data on posting intervals.
    unsigned long currentMillis = millis();
    if (currentMillis - lastEspnowPost >= espnowPostInterval) {
      // Save the last time a new reading was published
      lastEspnowPost = currentMillis;
      //Set values to send
      outgoing_data.msg_type = DATA;
      outgoing_data.sender_role = CONTROLLER;
      outgoing_data.fault_code = 0x16;
      outgoing_data.air_return_temp = air_return_temp;
      outgoing_data.air_supply_temp = air_supply_temp;
      outgoing_data.drain_switch = true;
      outgoing_data.cooling_relay = false;
      outgoing_data.turbine_relay = false;
      esp_err_t result = esp_now_send(NULL, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
    }
    break;

  }
  return;
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
  self_serial = get_device_serial(mac_buffer);
  //-
  if (esp_now_init() != ESP_OK) {
    error_logger("-- Error initializing ESP-NOW, please reboot --");
    while (1){;}
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  //LOAD DATA FROM FS
  load_server_from_fs();

  //- PAIRING SETUP
  if (strcmp(server_serial.c_str(), "ffffffffffff") == 0) {
    //- server address has never been set. configure idle mode for pairing process.
    info_logger("[esp-now] server mac address is the default value, waiting for now button to pair");
    pairingStatus = NOT_PAIRED;
  } else {
    info_logger("[esp-now] ready for esp-now communication with the server");
    pairingStatus = PAIR_REQUEST;
  }

  //-done
  info_logger("[esp-now] settings done.");

  // SETUP FINISHED
  start = millis();
  info_logger("setup finished --!.");
}

void loop() {
  //-
  update_temperature_readings();
  update_IO();
  espnow_loop();
}