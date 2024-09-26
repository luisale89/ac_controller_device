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
#define FAN_RELAY 17
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
float air_return_temp = 24;
float air_supply_temp = 24;

// esp-now variables
esp_now_peer_info_t server_peer; // variable holds the data for esp-now communications.
const int MAX_PAIR_ATTEMPTS = 55; // 5 times on each channel.
const char SERIAL_DEFAULT[] = "ffffffffffff";
uint8_t server_mac_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
int radio_channel = 1; // esp-now communication channel.
int pair_request_attempts = 0;
bool postEspnowFlag = false;

enum PeerRoleID {SERVER, CONTROLLER, MONITOR_A, MONITOR_B, ROLE_UNSET};
enum PairingStatusEnum {PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED, NOT_PAIRED};
enum MessageTypeEnum {PAIRING, DATA,};
enum SysModeEnum {AUTO_MODE, FAN_MODE, COOL_MODE};
enum SysStateEnum {SYSTEM_ON, SYSTEM_OFF, SYSTEM_SLEEP, UNKN};
//-vars
PairingStatusEnum pairingStatus = NOT_PAIRED;

typedef struct controller_data_struct {
  MessageTypeEnum msg_type;// (1 byte)
  PeerRoleID sender_role;  // (1 byte)
  uint8_t fault_code;      // (1 byte)
  float air_return_temp;   // (4 bytes) [°C]
  float air_supply_temp;   // (4 bytes) [°C]
  bool drain_switch;       // (1 byte)
  bool cooling_relay;      // (1 byte)
  bool fan_relay;      // (1 byte)
  unsigned int seconds_since_last_cooling_rq;  // (4 bytes) seconds since last false->true relay change.
} controller_data_struct;  // TOTAL = 18 bytes

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
incoming_settings_struct settings_data = { // initial values.
  DATA, ROLE_UNSET, FAN_MODE, UNKN, 24, 24
  };  // data received from server
pairing_data_struct pairing_data;

// time vars.
unsigned int secondsSinceCR = 0; // seconds since last cooling request.
unsigned long lastTempRequest = 0;
unsigned long lastEspnowReceived = 0;   // Stores last time data was published
unsigned long lastPairingRequest = 0;
unsigned long currentMillis = 0;
unsigned long lastNetworkLedBlink = 0;
unsigned long lastNowBtnChange = 0;
unsigned long lastCompressorTurnOff = 0;
unsigned long lastFanTurnOn = 0;
unsigned long lastSystemLog = 0;
unsigned long lastSecondTick = 0;
const unsigned long SYSTEM_LOG_DELAY = 1000L; // 1 second.
const unsigned long FAN_OFF_DELAY = 5000L; // 5 seconds off-delay.
const unsigned long COMPRESSOR_ON_DELAY = 5000L; // 5 seconds on-delay.
const unsigned long COMPRESSOR_SHORT_CYCLING_DELAY = 5L * 60000L; // 5 minutes for compressor short cycling prevention.
const unsigned long ESP_NOW_POST_INTERVAL = 500; // 0,5 seconds after receiving data from the server.
const unsigned long ESP_NOW_WAIT_SERVER_MSG = 1L * 60000L; // 1 minute for server message to arrive before PAIRING mode is set.
const unsigned long ESP_NOW_WAIT_PAIR_RESPONSE = 2000; // Interval to wait for pairing response from server
const unsigned long BTN_DEBOUNCE_TIME = 75; // 50ms rebound time constant;

// gen vars
bool now_btn_state = false;
bool last_now_btn_state = false;
bool float_sw_state = false;
bool last_float_sw_state = false;
int led_brightness = 0;
int led_fade_amount = 5;
bool compressor_state = false;
bool fan_state = false;

//-
void network_led_pulse_effect() {
  // pulse effect.
  analogWrite(NETWORK_LED, led_brightness);
  led_brightness = led_brightness + led_fade_amount;
  if (led_brightness <= 0 || led_brightness >= 125) {
    led_fade_amount = -led_fade_amount;
  }
}

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
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "loading data from %s", target_file);
  //-
  File f = SPIFFS.open(target_file);
  if (!f) {
    error_logger("Error al abrir el archivo solicitado.");
    return "null";
  }

  String file_string = f.readString();
  f.close();
  delay(100);

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
  debug_logger("data saved in SPIFFS!");
  delay(100);

  return;
}

// print mac address.
String print_device_mac(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

//get device id from mac address.
String print_device_serial(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x%02x%02x%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

//- mac address parser
void parse_mac_address(const char* str, char sep, byte* bytes, int maxBytes, int base) {
    for (int i = 0; i < maxBytes; i++) {
        bytes[i] = strtoul(str, NULL, base);  // Convert byte
        str = strchr(str, sep);               // Find next separator
        if (str == NULL || *str == '\0') {
            break;                            // No more separators, exit
        }
        str++;                                // Point to next character after separator
    }
}

void set_defaults_in_fs(){
  info_logger("saving server default values in the fs.");
  JsonDocument json_doc;
  String data;

  //create json_doc
  json_doc["server_serial"] = "ffffffffffff";
  json_doc["server_mac"] = "FF:FF:FF:FF:FF:FF";
  json_doc["server_chan"] = 1;

  serializeJson(json_doc, data);
  save_data_in_fs(data, "/now_server.txt");

  return;

}

void save_server_in_fs(const uint8_t *new_mac_address, uint8_t new_channel) {
  //- save server info in spiffs.

  info_logger("Saving new server data in the fs.");
  JsonDocument server_json;
  String data;
  
  // create json
  server_json["server_serial"] = print_device_serial(new_mac_address);
  server_json["server_mac"] = print_device_mac(new_mac_address);
  server_json["server_chan"] = new_channel;
  //save json in filesystem.
  serializeJson(server_json, data);
  save_data_in_fs(data, "/now_server.txt");

  return;
}

//- load server mac address from filesystem.
void load_server_from_fs() {
  info_logger("Loading server data from fs.");

  JsonDocument server_json;
  String server_data = load_data_from_fs("/now_server.txt");
  DeserializationError error = deserializeJson(server_json, server_data);
  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "server_json Deserialization error raised with code: %s", error.c_str());
    return;
  }

  const char *server_ser = server_json["server_serial"] | "null";
  const char *server_mac_str = server_json["server_mac"] | "null";
  uint8_t server_channel = server_json["server_chan"] | 1;

  //- validations.
  if (radio_channel < 1 || radio_channel > MAX_CHANNEL) {
    error_logger("Invalid Wifi channel stored in fs. setting channel to default value.");
    server_channel = 1;
  }

  if (strcmp(server_ser, "null") == 0) {
    error_logger("server id not found in fs.");
    return;
  }
  if (strcmp(server_mac_str, "null") == 0) {
    error_logger("server mac addres not found in fs.");
    return;
  }

  //reset server_peer variable.
  memset(&server_peer, 0, sizeof(esp_now_peer_info_t));

  //-
  parse_mac_address(server_mac_str, ':', server_mac_address, 6, 16);

  //- update server_peer variable.
  memcpy(&server_peer.peer_addr, server_mac_address, sizeof(uint8_t[6]));
  server_peer.encrypt = false;
  server_peer.channel = server_channel;
  radio_channel = server_channel;

  //- Set pairingStatus based on the server_serial loaded from fs.
  if (strcmp(server_ser, SERIAL_DEFAULT) == 0) {
    //- server address has never been set. configure idle mode for pairing process.
    info_logger("[esp-now] server mac address is the default value.");
    pairingStatus = NOT_PAIRED;
  } else {
    info_logger("[esp-now] ready for esp-now communication with the server");
    pairingStatus = PAIR_REQUEST;
  }

  info_logger("server data loaded from fs!");
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
    float supplyBuffer = air_supply_sensor.getTempCByIndex(0);
    if (is_valid_temp(returnBuffer)){
      // update global variable.
      air_return_temp = returnBuffer;
    } else {
      error_logger("invalid reading on return sensor.");
    }

    if (is_valid_temp(supplyBuffer))
    {
      // update global variable.
      air_supply_temp = supplyBuffer;
    } else {
      error_logger("invalid reading on supply sensor.");
    }
    
    //request new readings.
    air_return_sensor.requestTemperatures();
    air_supply_sensor.requestTemperatures();
    //- set timer
    lastTempRequest = millis();
  }
  return;
}

void set_compressor_state(bool new_state){

  currentMillis = millis();

  //- turn off the compressor
  if (new_state == false && compressor_state == true){
    info_logger("turning off the compressor");
    digitalWrite(COMP_RELAY, LOW);
    compressor_state = false;
    lastCompressorTurnOff = currentMillis;
    return;
  }

  //- turn on the compressor
  if (new_state == true && compressor_state == false){

    //-SHORT_CYCLING_PREVENTION-
    if (currentMillis - lastCompressorTurnOff < COMPRESSOR_SHORT_CYCLING_DELAY) {
      return;
    }

    // if the fan is off, the compressor wouldn't turn on.
    if (!fan_state) {
      return;
    }

    //- on_delay since Fan last turn ON.
    if (currentMillis - lastFanTurnOn >= COMPRESSOR_ON_DELAY) {
      info_logger("turning on the compressor");
      digitalWrite(COMP_RELAY, HIGH);
      compressor_state = true;
      secondsSinceCR = 0; // restart the counter.
      lastSecondTick = currentMillis;
      return;
    }
  }

  return;
}

void set_fan_state(bool new_state) {

  currentMillis = millis();

  //- turn on the fan.
  if (new_state == true && fan_state == false){
    info_logger("turning on the fan.");
    digitalWrite(FAN_RELAY, HIGH);
    lastFanTurnOn = currentMillis;
    fan_state = true;
    return;
  }
  
  //- turn off the fan.
  if (new_state == false && fan_state == true){

    if (!compressor_state){
      // if the compressor is on, the fan wouldn't turn off..
      return;
    }

    // off delay time after the compressor last turn off.
    if (currentMillis - lastCompressorTurnOff >= FAN_OFF_DELAY) {
      info_logger("turning off the fan.");
      digitalWrite(FAN_RELAY, LOW);
      fan_state = false;
      return;
    }
  }

  return;
}


void update_IO()
{
  currentMillis = millis();
  //inputs
  const bool current_now_btn = digitalRead(NOW_CNF) ? false : true; //button is active low.
  const bool current_float_switch = digitalRead(FLOAT_SWTCH) ? false : true; // float switch is active low.

  if (current_now_btn != last_now_btn_state) {
    lastNowBtnChange = currentMillis;
    last_now_btn_state = current_now_btn;
  }

  if (currentMillis - lastNowBtnChange > BTN_DEBOUNCE_TIME) {
    // btn change.
    now_btn_state = current_now_btn;
  }

  //- outputs.
  // turn off all relays when the device is not PAIRED
  
  if (pairingStatus != PAIR_PAIRED) {
    set_compressor_state(false);
    set_fan_state(false);
    digitalWrite(AUTO_RELAY, LOW);
    return;
  }

  switch (settings_data.system_state) // state received from server.
  {
  case UNKN:
    set_fan_state(false);
    set_compressor_state(false);
    break;;

  case SYSTEM_OFF:
    set_fan_state(false);
    set_compressor_state(false);
    break;

  case SYSTEM_SLEEP:
    set_fan_state(false);
    set_compressor_state(false);
    break;

  case SYSTEM_ON:
    // system is on.
    // turn on the compressor based on the return temp. value.
    const float on_value = settings_data.system_temp_sp + 0.5; // +0.5 deg.
    const float off_value = settings_data.system_temp_sp - 0.5; // -0.5
    //- fan mode function.
    if (settings_data.system_mode == FAN_MODE) {
      // turn on the fan only.
      set_fan_state(true);
      set_compressor_state(false);
      return;
    }

    // cool mode or auto mode.
    if (settings_data.system_mode == COOL_MODE) {
      // cooling mode state.
      // turn on the fan allways.
      set_fan_state(true);
      
      if (air_return_temp <= off_value) {
        set_compressor_state(false);
      } else {
        set_compressor_state(true);
      }

      return;
    }

    if (settings_data.system_mode == AUTO_MODE) {

      if (air_return_temp <= off_value) {
        set_compressor_state(false);
        set_fan_state(false);
      } else {
        set_compressor_state(true);
        set_fan_state(true);
      }

      return;
    }

    //-breaks;
    break;
  }

  return;
}

void update_cr_counter() {

  currentMillis = millis();

  if (!compressor_state) {
    //nothing to count when the compressor is off.
    return;
  }

  if (currentMillis - lastSecondTick >= 1000) {
    //1 second count
    lastSecondTick = currentMillis;
    secondsSinceCR ++; // sum 1 second to the counter.
  }

  return;
}

//- *esp_now functions
bool add_peer_to_plist(const uint8_t * mac_addr, uint8_t channel){
  info_logger("[esp-now] adding new peer to peer list.");

  if (channel <= 0 || channel > MAX_CHANNEL) {
    error_logger("invalid channel value received. peer not added");
    return false;
  }

  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "MAC: %s, channel: %d.", print_device_mac(mac_addr).c_str(), channel);
  //- set to 0 all data of the peerTemplate var.
  memset(&server_peer, 0, sizeof(esp_now_peer_info_t));

  //-set data
  //update global variable.
  memcpy(&server_peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  server_peer.channel = channel;
  server_peer.encrypt = false;

  // delete existing peer.
  if (esp_now_is_peer_exist(server_peer.peer_addr)) {
    debug_logger("peer already exists. deleting old data.");
    esp_now_del_peer(server_peer.peer_addr);
  }
  // save peer in peerlist
  esp_err_t result = esp_now_add_peer(&server_peer);

  //- update WiFi channel.
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
  info_logger("WiFi channel updated!");

  switch (result)
  {
  case ESP_OK:
    info_logger("New peer added successfully!..");
    return true;
  
  default:
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "Error: %d, while adding new peer.", esp_err_to_name(result));
    return false;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //- callback.
  String device_id = print_device_mac(mac_addr);
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
  String mac_str = print_device_mac(mac_addr);
  String sender_serial = print_device_serial(mac_addr);
  String server_serial = print_device_serial(server_peer.peer_addr);

  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] %d bytes of data received from: %s", len, mac_str.c_str());

  //only accept messages from server device.
  uint8_t message_type = incomingData[0];
  uint8_t sender_role = incomingData[1];

  if (sender_role != SERVER) {
    info_logger("[esp-now] invalid device role. ignoring message");
    return;
  }

  lastEspnowReceived = millis(); // time of the last message received from the server.

  //get message_type message from first byte.
  switch (message_type) {
  case DATA :      // we received data from server
    //- DATA type
    //- validations
    if (pairingStatus != PAIR_PAIRED) {
      info_logger("device not paired yet! ignoring data.");
      break;
    }
    if (sender_serial != server_serial) {
      info_logger("message received from an unknown device, ignoring data.");
      break;
    }
    //- ingest incoming data.
    info_logger("[esp-now] message of type DATA received");
    memcpy(&settings_data, incomingData, sizeof(settings_data));
    postEspnowFlag = true; // flag to send a response to the server.
    //-
    break;

  case PAIRING:    // received pairing data from server
    //- PAIRING type
    info_logger("[esp-now] message of type PAIRING received");
    memcpy(&pairing_data, incomingData, sizeof(pairing_data));
    // add peer to peer list.
    if (!add_peer_to_plist(mac_addr, pairing_data.channel)){ // the server decides the channel.
      error_logger("[esp-now] server peer couldn't be saved, try again.");
      break;
    }
    // save server data in fs.
    save_server_in_fs(mac_addr, pairing_data.channel); // update server info in fs.
    
    // device PAIRED with server.
    char buff[100] = "";
    sprintf(buff, "device paired on channel %d after %d attempts", radio_channel, pair_request_attempts);
    info_logger(buff);
    pair_request_attempts = 0;
    pairingStatus = PAIR_PAIRED;  // set the pairing status
    //-
    break;
  }
}

void log_on_result(esp_err_t result) {
    //-swtch
  switch (result)
  {
  case ESP_OK:
    info_logger("[esp-now] message to the server has been sent.");
    break;
  
  default:
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[esp-now] error sending msg to peer, reason: %s",  esp_err_to_name(result));
    break;
  }

  return;
}

void system_logs() {
  currentMillis = millis();
  JsonDocument root;
  String doc;

  if (currentMillis - lastSystemLog >= SYSTEM_LOG_DELAY) {
    lastSystemLog = currentMillis;
    //- data
    root["room_t"] = settings_data.room_temp;
    root["return_t"] = air_return_temp;
    root["supply_t"] = air_supply_temp;
    root["system_sp"] = settings_data.system_temp_sp;
    root["compressor"] = compressor_state;
    root["fan"] = fan_state;
    root["sys_state"] = settings_data.system_state;
    root["sys_mode"] = settings_data.system_mode;
    root["pairing"] = pairingStatus;
    root["last_cr_cnt"] = secondsSinceCR;

    //- output
    serializeJson(root, doc);
    debug_logger(doc.c_str());
  }

  return;
}

void espnow_loop(){
  // current time.
  currentMillis = millis();
  esp_err_t send_result;

  if (now_btn_state && currentMillis - lastNowBtnChange > 10000L) {
    // after 10 seconds of now button pressed, default values will be set.
    lastNowBtnChange = currentMillis;
    //-
    set_defaults_in_fs();
    //-
    info_logger("ESP restart...");
    digitalWrite(NETWORK_LED, HIGH);
    delay(1000);
    ESP.restart();
  }

  // switch modes.
  switch(pairingStatus) {
    // PAIR REQUEST
    case PAIR_REQUEST:
      if (strcmp(SERIAL_DEFAULT, print_device_serial(server_mac_address).c_str()) == 0)
      {
        // check if no more attemps are allowed.
        if (pair_request_attempts >= MAX_PAIR_ATTEMPTS) {
          // ends pairing process.
          pairingStatus = NOT_PAIRED;
          pair_request_attempts = 0;
          info_logger("auto-pairing process couln't find the server.");
          break;
        }
      }
      pair_request_attempts ++;
      ESP_LOG_LEVEL(LOG_LOCAL_LEVEL, TAG, "Sending PR# %d on channel: %d", pair_request_attempts, radio_channel);
    
      // set pairing data to send to the server
      pairing_data.msg_type = PAIRING;
      pairing_data.sender_role = CONTROLLER;
      pairing_data.channel = radio_channel;

      // add peer and send request
      if (!add_peer_to_plist(server_mac_address, radio_channel)){ // mac address stored in global var.
        error_logger("[esp-now] couldn't add peer to peer list.");
        break;
      }

      //- send pair data.
      send_result = esp_now_send(server_peer.peer_addr, (uint8_t *) &pairing_data, sizeof(pairing_data));
      //-log
      log_on_result(send_result);

      lastPairingRequest = currentMillis;
      pairingStatus = PAIR_REQUESTED;
    break;

    // PAIR REQUESTED
    case PAIR_REQUESTED:
      // change wifi channel for continue with the pairing process.
      if (currentMillis - lastNetworkLedBlink > 125L) { // 4hz blink
        lastNetworkLedBlink = currentMillis;
        digitalWrite(NETWORK_LED, !digitalRead(NETWORK_LED));
      }

      // time out to allow receiving response from server
      if(currentMillis - lastPairingRequest > ESP_NOW_WAIT_PAIR_RESPONSE) {
        info_logger("[autopairing] time out expired, try on the next channel");
        radio_channel ++;
        if (radio_channel > MAX_CHANNEL){
          radio_channel = 1;
        }
        // set WiFi channel   
        pairingStatus = PAIR_REQUEST;

      }
    break;

    // NOT PAIRED
    case NOT_PAIRED:
      if (currentMillis - lastNetworkLedBlink > 500L) { // 1hz blink
        lastNetworkLedBlink = currentMillis;
        digitalWrite(NETWORK_LED, !digitalRead(NETWORK_LED));
      }
      if (now_btn_state && currentMillis - lastNowBtnChange > 3000L) {
        // after 3 seconds of now button pressed..
        pairingStatus = PAIR_REQUEST; // begin pair process.
      }
      // waiting for now button press to begin pair process. 
      // starts with the default address (ffx6) and wait for the response
      // to save the correct mac_address.
    break;
    
    // PAIRED
    case PAIR_PAIRED:
      //- posting data on posting intervals.
      if (currentMillis - lastNetworkLedBlink > 50L) {
        lastNetworkLedBlink = currentMillis;
        network_led_pulse_effect();
      }

      if (currentMillis - lastEspnowReceived >= ESP_NOW_WAIT_SERVER_MSG) {
        info_logger("Timeout since last server message received. Setting up PR Mode.");
        pairingStatus = PAIR_REQUEST;
      }

      //- time based message sent.
      if (postEspnowFlag && currentMillis - lastEspnowReceived >= ESP_NOW_POST_INTERVAL) {
        // set flag to false.
        postEspnowFlag = false;
        //Set values to send
        outgoing_data.msg_type = DATA;
        outgoing_data.sender_role = CONTROLLER;
        outgoing_data.fault_code = 0x00;
        outgoing_data.air_return_temp = air_return_temp;
        outgoing_data.air_supply_temp = air_supply_temp;
        outgoing_data.drain_switch = digitalRead(FLOAT_SWTCH); //to-do.. need to update this to the state.
        outgoing_data.cooling_relay = compressor_state;
        outgoing_data.fan_relay = fan_state;
        outgoing_data.seconds_since_last_cooling_rq = secondsSinceCR;

        send_result = esp_now_send(server_peer.peer_addr, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
        log_on_result(send_result);
      }
    break;
  }

  //-end
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
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(COMP_RELAY, OUTPUT);
  pinMode(NETWORK_LED, OUTPUT);
  pinMode(Q3_INPUT, INPUT_PULLUP);
  pinMode(Q2_INPUT, INPUT_PULLUP);
  pinMode(Q1_INPUT, INPUT_PULLUP);
  pinMode(FLOAT_SWTCH, INPUT);
  pinMode(NOW_CNF, INPUT);
  info_logger("pins settings done.");

  // SPIFFS SETUP
  if(!SPIFFS.begin(true)) {
    error_logger("Ocurrió un error al iniciar SPIFFS..");
    while (1){;}
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
  WiFi.mode(WIFI_MODE_STA);
  info_logger("MAC Address:  ->");
  info_logger(WiFi.macAddress().c_str());
  WiFi.disconnect();
  info_logger("WiFi settings done.");

  //- ESP-NOW SETUP
  info_logger("esp-now settings");
  uint8_t mac_buffer[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_AP, mac_buffer);
  if (ret != ESP_OK) {
    error_logger("could not read mac address.");
  }
  //-
  if (esp_now_init() != ESP_OK) {
    error_logger("-- Error initializing ESP-NOW, please reboot --");
    while (1){;}
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  //-done
  info_logger("[esp-now] settings done.");

  //LOAD DATA FROM FS
  load_server_from_fs();

  currentMillis = millis();
  lastCompressorTurnOff = currentMillis;

  // SETUP FINISHED
  info_logger("setup finished --!.");
  delay(500);
}

void loop() {
  //-
  update_temperature_readings();
  update_IO();
  update_cr_counter();
  espnow_loop();
  system_logs();
}