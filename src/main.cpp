// logging
#include <esp_log.h>
static const char *TAG = "main";
// libraries
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>
#include <LittleFS.h>
#include <secrets.h>
#include <ArduinoJson.h>

// pinout
#define FAN_RELAY 17
#define COMPRESSOR_RELAY 16
#define ALARM_RELAY 18
#define NETWORK_LED 2
#define Q3_INPUT 14
#define Q2_INPUT 27
#define Q1_INPUT 26
#define RETURN_TEMP 25
#define SUPPLY_TEMP 33
#define FLOAT_SWTCH 32
#define NOW_CNF 35
#define MAX_CHANNEL 11 // for North America

// oneWire and DallasTemperature
OneWire ow_return_temp(RETURN_TEMP);
OneWire ow_supply_temp(SUPPLY_TEMP);
DallasTemperature air_return_sensor(&ow_return_temp);
DallasTemperature air_supply_sensor(&ow_supply_temp);
int tempSensorResolution = 12; // bits
int tempRequestDelay = 0;
float air_return_temp = 24;
float air_supply_temp = 24;

// esp-now variables
esp_now_peer_info_t server_peer;  // variable holds the data for esp-now communications.
const int MAX_PAIR_ATTEMPTS = 55; // 5 times on each channel.
const char SERIAL_DEFAULT[] = "ffffffffffff";
uint8_t server_mac_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
int radio_channel = 1; // esp-now communication channel.
int pair_request_attempts = 0;
bool postEspnowFlag = false;

enum AlarmCode
{
  UNKNOWN_ALARM = -1,
  NORMAL,
  HIGH_DISCHARGE_TEMP,
  HIGH_LIQUID_TEMP,
  HIGH_EXTERIOR_TEMP,
  HIGH_PRESSURE_SWITCH,
  HIGH_COMPRESSOR_CURRENT,
  HIGH_AC_MAINS_VOLTAGE,
  LOW_VAPOR_TEMP,
  LOW_ENTHALPY,
  LOW_PRESSURE_SWITCH,
  LOW_AC_MAINS_VOLTAGE,
  LOW_EVAP_DELTA_T,
  DRAIN_SWITCH_OPEN,
  COMPRESSOR_STALL,
};

enum PeerRoleID
{
  SERVER,
  CONTROLLER,
  MONITOR,
  UNSET
};
enum PairingStatusEnum
{
  PAIR_REQUEST,
  PAIR_REQUESTED,
  PAIR_PAIRED,
  NOT_PAIRED
};
enum MessageTypeEnum
{
  PAIRING,
  DATA,
};
enum SysModeEnum
{
  AUTO_MODE,
  FAN_MODE,
  COOL_MODE
};
enum SysStateEnum
{
  SYSTEM_ON,
  SYSTEM_OFF,
  SYSTEM_SLEEP,
  UNKN
};
enum SysStatusFlag
{
  STATUS_OK,
  STATUS_WARNING,
  STATUS_ERROR
};
//-vars
PairingStatusEnum pairingStatus = NOT_PAIRED;

typedef struct
{
  MessageTypeEnum msg_type;                   //
  PeerRoleID sender_role;                     //
  AlarmCode alarm_code;                       //  alarm_code_state... enum value... alarm detected in the controller device.
  float air_return_temp;                      //  [°C]
  float air_supply_temp;                      //  [°C]
  bool drain_switch;                          //
  bool cooling_relay;                         //
  bool fan_relay;                             //
  unsigned int seconds_since_last_cooling_rq; //  seconds since last false->true relay change.
} controller_data_struct;                     //

typedef struct
{
  SysModeEnum peers_mode;    // (1 byte)
  SysStateEnum system_state; // (1 byte)
  AlarmCode alarm_code;      // (1 byte)
  float system_temp_sp;      // (4 bytes) [°C]
  float room_temp;           // (4 bytes) [°C] room temp value to be used as control temp if room_temp_control_en is true.
  bool room_temp_control_en; // (1 byte) true if the system temp sp is based on the room temp, false if it's based on the return air temp.
} incoming_settings_struct;  // TOTAL = 13 bytes

typedef struct
{
  MessageTypeEnum msg_type;   // tipo de mensaje (PAIR)
  PeerRoleID sender_role;     //
  PeerRoleID device_new_role; //
  int channel;                //  - 0 is default, let this value for future changes.
} pairing_data_struct;        //

// Create 2 struct_message
controller_data_struct outgoing_data;       // data to send
incoming_settings_struct settings_data = {  // initial values.
    FAN_MODE, UNKN, NORMAL, 24, 24, false}; // data received from server
pairing_data_struct pairing_data;

// time vars.
unsigned int compressorRunningSeconds = 0; // seconds since last cooling request.
unsigned int fanRunningSeconds = 0;        // total cooling request counter.
unsigned long lastTempRequest = 0;
unsigned long lastEspnowReceived = 0; // Stores last time data was published
unsigned long lastPairingRequest = 0;
unsigned long currentMillis = 0;
unsigned long lastNetworkLedBlink = 0;
unsigned long lastNowBtnChange = 0;
unsigned long lastFloatSwitchChange = 0;
unsigned long lastCompressorTurnOff = 0;
unsigned long lastFanTurnOn = 0;
unsigned long lastSystemLog = 0;
unsigned long lastSecondTick = 0;
const unsigned long SYSTEM_LOG_DELAY = 5000UL;                    // 5 second.
const unsigned long FAN_OFF_DELAY = 5000UL;                       // 5 seconds off-delay.
const unsigned long COMPRESSOR_ON_DELAY = 5000UL;                 // 5 seconds on-delay.
const unsigned long COMPRESSOR_SHORT_CYCLING_DELAY = 3 * 60000UL; // 3 minutes for compressor short cycling prevention.
const unsigned long ESP_NOW_POST_INTERVAL = 500UL;                // 0,5 seconds after receiving data from the server.
const unsigned long ESP_NOW_WAIT_SERVER_MSG = 1 * 60000UL;        // 1 minute for server message to arrive before PAIRING mode is set.
const unsigned long ESP_NOW_WAIT_PAIR_RESPONSE = 2000UL;          // Interval to wait for pairing response from server
const unsigned long BTN_DEBOUNCE_TIME = 100UL;                    // 100ms rebound time constant;

// gen vars
bool now_btn_state = false;
bool last_now_btn_state = false;
bool float_sw_state = false;
bool last_float_sw_state = false;
int led_brightness = 0;
int led_fade_amount = 5;
unsigned int hourmeter_count = 0;
bool compressor_state = false;
bool fan_state = false;
AlarmCode calculated_alarm_code = NORMAL;

//-
void network_led_pulse_effect()
{
  // pulse effect.
  analogWrite(NETWORK_LED, led_brightness);
  led_brightness = led_brightness + led_fade_amount;
  if (led_brightness <= 0 || led_brightness >= 125)
  {
    led_fade_amount = -led_fade_amount;
  }
}

// función que carga datos que contiene toda la información dentro del target_file del SPIFFS.
char *load_data_from_fs(const char *target_file)
{
  static char buffer[1024]; // Buffer estático para evitar heap
  //-
  File f = LittleFS.open(target_file);
  if (!f)
  {
    ESP_LOGE(TAG, "Error al abrir el archivo solicitado.");
    strcpy(buffer, "null");
    return buffer;
  }

  size_t len = f.size();
  if (len >= sizeof(buffer))
  {
    ESP_LOGE(TAG, "Archivo demasiado grande para el buffer.");
    f.close();
    strcpy(buffer, "null");
    return buffer;
  }

  f.readBytes(buffer, len);
  buffer[len] = '\0';
  f.close();

  ESP_LOGI(TAG, "data loaded from SPIFFS correctly");
  ESP_LOGD(TAG, "data: %s", buffer);
  return buffer;
}

// función que guarda datos en el target_file del SPIFFS.
void save_data_in_fs(const char *data_to_save, const char *target_file)
{
  // savin data in filesystem.
  ESP_LOGI(TAG, "saving data in SPIFFS");
  ESP_LOGD(TAG, "target_file: %s", target_file);
  ESP_LOGD(TAG, "data: %s", data_to_save);

  File f = LittleFS.open(target_file, "w");
  if (!f)
  {
    ESP_LOGE(TAG, "Error al abrir el archivo solicitado.");
    return;
  }

  f.print(data_to_save);
  f.close();
  ESP_LOGI(TAG, "data saved correctly in SPIFFS.");
  return;
}

// print mac address.
const char *print_device_mac(const uint8_t *mac_addr)
{
  static char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  return mac_str;
}

// get device id from mac address.
const char *print_device_serial(const uint8_t *mac_addr)
{
  static char mac_str[13];
  snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x%02x%02x%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  return mac_str;
}

//- mac address parser
void parse_mac_address(const char *str, char sep, byte *bytes, int base)
{
  const int max_bytes = 6; // MAC addresses have 6 bytes
  for (int i = 0; i < max_bytes; i++)
  {
    bytes[i] = strtoul(str, NULL, base); // Convert byte
    str = strchr(str, sep);              // Find next separator
    if (str == NULL || *str == '\0')
    {
      break; // No more separators, exit
    }
    str++; // Point to next character after separator
  }
}

void set_defaults_in_fs()
{
  ESP_LOGI(TAG, "saving server default values in the fs.");
  JsonDocument json_doc;
  char data[256];

  // create json_doc
  json_doc["server_serial"] = "ffffffffffff";
  json_doc["server_mac"] = "FF:FF:FF:FF:FF:FF";
  json_doc["server_chan"] = 1;

  serializeJson(json_doc, data);
  save_data_in_fs(data, "/now_server.txt");

  return;
}

void save_server_in_fs(const uint8_t *new_mac_address, uint8_t new_channel)
{
  //- save server info in spiffs.

  ESP_LOGI(TAG, "Saving new server data in the fs.");
  JsonDocument server_json;
  char data[256];

  // create json
  server_json["server_serial"] = print_device_serial(new_mac_address);
  server_json["server_mac"] = print_device_mac(new_mac_address);
  server_json["server_chan"] = new_channel;
  // save json in filesystem.
  serializeJson(server_json, data);
  save_data_in_fs(data, "/now_server.txt");

  return;
}

//- load server mac address from filesystem.
void load_server_from_fs()
{
  ESP_LOGI(TAG, "Loading server data from fs.");

  JsonDocument server_json;
  const char *server_data = load_data_from_fs("/now_server.txt");
  DeserializationError error = deserializeJson(server_json, server_data);
  if (error)
  {
    ESP_LOGE(TAG, "server_json Deserialization error raised with code: %s", error.c_str());
    return;
  }

  const char *server_ser = server_json["server_serial"] | "null";
  const char *server_mac_str = server_json["server_mac"] | "null";
  uint8_t server_channel = server_json["server_chan"] | 1;

  //- validations.
  if (radio_channel < 1 || radio_channel > MAX_CHANNEL)
  {
    ESP_LOGE(TAG, "Invalid Wifi channel stored in fs. setting channel to default value.");
    server_channel = 1;
  }

  if (strcmp(server_ser, "null") == 0)
  {
    ESP_LOGE(TAG, "server id not found in fs.");
    return;
  }
  if (strcmp(server_mac_str, "null") == 0)
  {
    ESP_LOGE(TAG, "server mac addres not found in fs.");
    return;
  }

  // reset server_peer variable.
  memset(&server_peer, 0, sizeof(esp_now_peer_info_t));

  //-
  parse_mac_address(server_mac_str, ':', server_mac_address, 16);

  //- update server_peer variable.
  memcpy(&server_peer.peer_addr, server_mac_address, sizeof(uint8_t[6]));
  server_peer.encrypt = false;
  server_peer.channel = server_channel;
  radio_channel = server_channel;

  //- Set pairingStatus based on the server_serial loaded from fs.
  if (strcmp(server_ser, SERIAL_DEFAULT) == 0)
  {
    //- server address has never been set. configure idle mode for pairing process.
    ESP_LOGI(TAG, "[esp-now] server mac address is the default value.");
    pairingStatus = NOT_PAIRED;
  }
  else
  {
    ESP_LOGI(TAG, "[esp-now] ready for esp-now communication with the server");
    pairingStatus = PAIR_REQUEST;
  }

  ESP_LOGI(TAG, "server data loaded from fs!");
  return;
}

//-functions
bool is_valid_temp(float measurement)
{
  char message_buffer[40];
  char name[8] = "ds18B20";

  if (measurement == -127)
  {
    ESP_LOGE(TAG, "Error -127; [%s]", name);
    return false;
  }

  if (measurement == 85.0)
  {
    ESP_LOGE(TAG, "Error 85; [%s]", name);
    return false;
  }

  return true;
}

void update_temperature_readings()
{
  if (millis() - lastTempRequest >= tempRequestDelay)
  {
    //-
    float returnBuffer = air_return_sensor.getTempCByIndex(0);
    float supplyBuffer = air_supply_sensor.getTempCByIndex(0);
    if (is_valid_temp(returnBuffer))
    {
      // update global variable.
      air_return_temp = returnBuffer;
    }
    else
    {
      ESP_LOGE(TAG, "invalid reading on return sensor.");
    }

    if (is_valid_temp(supplyBuffer))
    {
      // update global variable.
      air_supply_temp = supplyBuffer;
    }
    else
    {
      ESP_LOGE(TAG, "invalid reading on supply sensor.");
    }

    // request new readings.
    air_return_sensor.requestTemperatures();
    air_supply_sensor.requestTemperatures();
    //- set timer
    lastTempRequest = millis();
  }
  return;
}

void set_compressor_state(const bool rq_state)
{
  currentMillis = millis();
  const bool new_state = rq_state && float_sw_state;
  // the compressor state request is true only if the float switch is closed and the request is true.

  //- turn off the compressor
  if (new_state == false && compressor_state == true)
  {
    ESP_LOGI(TAG, "turning off the compressor");
    digitalWrite(COMPRESSOR_RELAY, LOW);
    compressor_state = false;
    lastCompressorTurnOff = currentMillis;
    compressorRunningSeconds = 0; // restart the counter.
    return;
  }

  //- turn on the compressor
  if (new_state == true && compressor_state == false)
  {

    //-SHORT_CYCLING_PREVENTION-
    if (currentMillis - lastCompressorTurnOff < COMPRESSOR_SHORT_CYCLING_DELAY)
    {
      return;
    }

    // if the fan is off, the compressor wouldn't turn on.
    if (!fan_state)
    {
      return;
    }

    //- on_delay since Fan last turn ON.
    if (currentMillis - lastFanTurnOn >= COMPRESSOR_ON_DELAY)
    {
      ESP_LOGI(TAG, "turning on the compressor");
      digitalWrite(COMPRESSOR_RELAY, HIGH);
      compressor_state = true;
      return;
    }
  }

  return;
}

void set_fan_state(bool new_state)
{

  currentMillis = millis();

  //- turn on the fan.
  if (new_state == true && fan_state == false)
  {
    ESP_LOGI(TAG, "turning on the fan.");
    digitalWrite(FAN_RELAY, HIGH);
    lastFanTurnOn = currentMillis;
    fan_state = true;
    return;
  }

  //- turn off the fan.
  if (new_state == false && fan_state == true)
  {

    if (compressor_state)
    {
      // if the compressor is on, the fan wouldn't turn off..
      return;
    }

    // off delay time after the compressor last turn off.
    if (currentMillis - lastCompressorTurnOff >= FAN_OFF_DELAY)
    {
      ESP_LOGI(TAG, "turning off the fan.");
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
  // inputs
  const bool current_now_btn = digitalRead(NOW_CNF) ? false : true;          // button is active low.
  const bool current_float_switch = digitalRead(FLOAT_SWTCH) ? false : true; // float switch is active low.

  // now button debounce.
  if (current_now_btn != last_now_btn_state)
  {
    lastNowBtnChange = currentMillis;
    last_now_btn_state = current_now_btn;
  }

  if (currentMillis - lastNowBtnChange >= BTN_DEBOUNCE_TIME)
  {
    // btn change.
    now_btn_state = current_now_btn;
  }

  // float switch debounce.
  if (current_float_switch != last_float_sw_state)
  {
    lastFloatSwitchChange = currentMillis;
    last_float_sw_state = current_float_switch;
  }

  if (currentMillis - lastFloatSwitchChange >= BTN_DEBOUNCE_TIME)
  {
    // float changed value.
    float_sw_state = current_float_switch;
  }

  //- outputs.
  //- for now, ALARM_RELAY will allways be off...
  digitalWrite(ALARM_RELAY, LOW);

  // turn off all relays when the device is not PAIRED
  if (pairingStatus != PAIR_PAIRED && pair_request_attempts >= 150)
  {
    // after 150 attempts of connection with the server. (approx. 5 min)
    set_fan_state(false);
    set_compressor_state(false);
    return;
  }

  if (settings_data.alarm_code != AlarmCode::NORMAL)
  {
    set_fan_state(false);
    set_compressor_state(false);
    return;
  }

  switch (settings_data.system_state) // state received from server.
  {
  case UNKN:
    set_fan_state(false);
    set_compressor_state(false);
    break;
    ;

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
    // turn on the compressor based on the return temp. value. 1 degree of hysteresis is set to avoid short cycling. (0.5 deg. on, 0.5 deg. off)
    const float on_value = settings_data.system_temp_sp + 0.5;  // +0.5 deg.
    const float off_value = settings_data.system_temp_sp - 0.5; // -0.5 deg.
    const float control_temp = settings_data.room_temp_control_en ? settings_data.room_temp : air_return_temp;
    // control temp is based on the room temp or the return temp based on the settings received from the server.

    //- fan mode function.
    if (settings_data.peers_mode == FAN_MODE)
    {
      // turn on the fan only.
      set_fan_state(true);
      set_compressor_state(false);
      return;
    }

    // cool mode or auto mode.
    if (settings_data.peers_mode == COOL_MODE)
    {
      // cooling mode state.
      // turn on the fan allways.
      set_fan_state(true);

      if (control_temp <= off_value)
      {
        set_compressor_state(false);
      }
      else if (control_temp >= on_value)
      {
        set_compressor_state(true);
      }

      return;
    }

    if (settings_data.peers_mode == AUTO_MODE)
    {
      // in auto mode, the fan follows the compressor state
      if (control_temp <= off_value)
      {
        set_compressor_state(false);
        set_fan_state(false);
      }
      else if (control_temp >= on_value)
      {
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

// void load_hourmeter_from_fs()
// {
//   ESP_LOGI(TAG, "-> loading hourmeter from fs");
//   JsonDocument json;
//   const char *hourmeter_data = load_data_from_fs("/hourmeter.txt");

//   DeserializationError error = deserializeJson(json, hourmeter_data);
//   if (error)
//   {
//     ESP_LOGE(TAG, "hourmeter Deserialization error raised with code: %s", error.c_str());
//     return;
//   }

//   hourmeter_count = json["hours"] | 0;

//   return;
// }

// void update_hourmeter_in_fs()
// {

//   // call this function every minute.

//   ESP_LOGI(TAG, "updating hourmeter in fs.");
//   const char *target_file = "/hourmeter.txt";
//   JsonDocument json;
//   JsonDocument new_json;
//   char new_hourmeter[256];
//   const char *hourmeter_data = load_data_from_fs(target_file);

//   DeserializationError error = deserializeJson(json, hourmeter_data);
//   if (error)
//   {
//     ESP_LOGE(TAG, "hourmeter Deserialization error raised with code: %s", error.c_str());
//     return;
//   }

//   const int current_h = json["hours"] | 0;
//   const int current_m = json["minutes"] | 0;

//   if (current_m < 59)
//   {
//     new_json["minutes"] = current_m + 1;
//     new_json["hours"] = current_h;
//   }
//   else
//   {
//     new_json["minutes"] = 0;
//     new_json["hours"] = current_h + 1;
//   }

//   // update global value.
//   hourmeter_count = new_json["hours"];

//   serializeJson(new_json, new_hourmeter);
//   save_data_in_fs(new_hourmeter, target_file);
// }

void update_time_counter()
{

  currentMillis = millis();

  if (!compressor_state)
  {
    // nothing to count when the compressor is in off state.
    lastSecondTick = currentMillis;
    return;
  }

  if (currentMillis - lastSecondTick >= 1000)
  {
    // 1 second count
    compressorRunningSeconds++; // sum 1 second to the counter.
    lastSecondTick = currentMillis;
  }

  return;
}

//- *esp_now functions
esp_err_t add_peer_to_plist(const uint8_t *mac_addr, uint8_t channel)
{
  ESP_LOGI(TAG, "[esp-now] adding new peer to peer list.");

  if (channel <= 0 || channel > MAX_CHANNEL)
  {
    ESP_LOGE(TAG, "invalid channel value received. peer not added");
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "MAC: %s, channel: %d.", print_device_mac(mac_addr), channel);
  //- set to 0 all data of the peerTemplate var.
  memset(&server_peer, 0, sizeof(esp_now_peer_info_t));

  //-set data
  // update global variable.
  memcpy(&server_peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  server_peer.channel = channel;
  server_peer.encrypt = false;

  // delete existing peer.
  if (esp_now_is_peer_exist(server_peer.peer_addr))
  {
    ESP_LOGD(TAG, "peer already exists. deleting old data.");
    esp_err_t deleteStatus = esp_now_del_peer(server_peer.peer_addr);
    if (deleteStatus == ESP_OK)
    {
      ESP_LOGI(TAG, "server-peer deleted!");
    }
    else
    {
      ESP_LOGE(TAG, "error deleting server-peer!");
      return ESP_FAIL;
    }
  }
  // save peer in peerlist
  esp_err_t result = esp_now_add_peer(&server_peer);

  //- update WiFi channel.
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
  ESP_LOGI(TAG, "WiFi channel updated!");

  switch (result)
  {
  case ESP_OK:
    ESP_LOGI(TAG, "New peer added successfully!..");
    return ESP_OK;

  default:
    ESP_LOGE(TAG, "Error: %d, while adding new peer.", esp_err_to_name(result));
    return ESP_FAIL;
  }
}

void OnDataSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
  //- callback.
  const char *device_id = print_device_mac(tx_info->des_addr);
  switch (status)
  {
  case ESP_NOW_SEND_SUCCESS:
    ESP_LOGI(TAG, "[esp-now] packet to: %s has been sent!", device_id);
    break;

  default:
    ESP_LOGE(TAG, "[esp-now] packet to: %s not sent.", device_id);
    break;
  }
}

void OnDataRecv(const esp_now_recv_info_t *rcv_info, const uint8_t *incomingData, int len)
{
  //-
  const char *mac_str = print_device_mac(rcv_info->src_addr);
  const char *sender_serial = print_device_serial(rcv_info->src_addr);
  const char *server_serial = print_device_serial(server_peer.peer_addr);
  JsonDocument json_payload;
  DeserializationError error = deserializeJson(json_payload, incomingData, len);
  if (error)
  {
    ESP_LOGE(TAG, "Deserialization error raised with code: %s", error.c_str());
    return;
  }

  ESP_LOGI(TAG, "[esp-now] %d bytes of data received from: %s", len, mac_str);

  // only accept messages from server device.
  uint8_t message_type = json_payload["msg"] | 255; // 255 is an invalid message type that will be discarded.
  uint8_t sender_role = json_payload["rol"] | 255;  // 255 is an invalid sender role that will be discarded.

  if (sender_role != SERVER)
  {
    ESP_LOGI(TAG, "[esp-now] invalid device role. ignoring message");
    ESP_LOGD(TAG, "sender role: %d", sender_role);
    return;
  }

  if (message_type != PAIRING && message_type != DATA)
  {
    ESP_LOGI(TAG, "[esp-now] invalid message type. ignoring message");
    ESP_LOGD(TAG, "message type: %d", message_type);
    return;
  }

  lastEspnowReceived = millis(); // time of the last message received from the server.

  // get message_type message from first byte.
  switch (message_type)
  {
  case DATA: // we received data from server
    //- DATA type
    //- validations
    if (pairingStatus != PAIR_PAIRED)
    {
      ESP_LOGI(TAG, "device not paired yet! ignoring data.");
      break;
    }
    if (sender_serial != server_serial)
    {
      ESP_LOGI(TAG, "message received from an unknown device, ignoring data.");
      break;
    }
    //- ingest incoming data.
    ESP_LOGI(TAG, "[esp-now] message of type DATA received");

    // update global variable with the data received from the server.
    settings_data.peers_mode = json_payload["glo"][0] | FAN_MODE;
    settings_data.system_state = json_payload["glo"][1] | UNKN;
    settings_data.alarm_code = json_payload["glo"][2] | NORMAL;
    settings_data.system_temp_sp = json_payload["glo"][3] | 24;
    settings_data.room_temp = json_payload["glo"][4] | 24;
    settings_data.room_temp_control_en = json_payload["cnf"][0] | false;

    postEspnowFlag = true; // flag to send a response to the server.
    //-
    break;

  case PAIRING: // received pairing data from server
    //- PAIRING type
    ESP_LOGI(TAG, "[esp-now] message of type PAIRING received");
    pairing_data.channel = json_payload["chan"] | radio_channel;

    // add peer to peer list.
    if (add_peer_to_plist(rcv_info->src_addr, pairing_data.channel) != ESP_OK)
    { // the server decides the channel.
      ESP_LOGE(TAG, "[esp-now] server peer couldn't be saved, try again.");
      break;
    }
    // save server data in fs.
    save_server_in_fs(rcv_info->src_addr, pairing_data.channel); // update server info in fs.

    // device PAIRED with server.
    ESP_LOGI(TAG, "device paired on channel %d after %d attempts", radio_channel, pair_request_attempts);
    pair_request_attempts = 0;
    pairingStatus = PAIR_PAIRED; // set the pairing status
    //-
    break;
  }
}

void log_on_result(esp_err_t result)
{
  //-swtch
  switch (result)
  {
  case ESP_OK:
    ESP_LOGI(TAG, "[esp-now] message to the server has been sent.");
    break;

  default:
    ESP_LOGE(TAG, "[esp-now] error sending msg to peer, reason: %s", esp_err_to_name(result));
    break;
  }

  return;
}

void system_logs()
{
  currentMillis = millis();
  JsonDocument root;
  char doc[256];

  if (currentMillis - lastSystemLog >= SYSTEM_LOG_DELAY)
  {
    lastSystemLog = currentMillis;
    //- data
    root["pair"] = pairingStatus;
    root["ret_t"] = air_return_temp;
    root["sup_t"] = air_supply_temp;
    root["sys_sp"] = settings_data.system_temp_sp;
    root["sys_stt"] = settings_data.system_state;
    root["sys_mod"] = settings_data.peers_mode;
    root["rt_ctrl"] = settings_data.room_temp_control_en;
    root["room_t"] = settings_data.room_temp;
    root["alarm"] = settings_data.alarm_code;
    root["calc_alarm"] = calculated_alarm_code;

    //- output
    serializeJson(root, doc);
    ESP_LOGD(TAG, "%s", doc);
  }

  return;
}

void espnow_loop()
{
  // current time.
  currentMillis = millis();
  esp_err_t send_result;

  if (now_btn_state && currentMillis - lastNowBtnChange > 10000L)
  {
    // after 10 seconds of now button pressed, default values will be set.
    lastNowBtnChange = currentMillis;
    //-
    set_defaults_in_fs();
    //-
    ESP_LOGI(TAG, "ESP restart...");
    digitalWrite(NETWORK_LED, HIGH);
    delay(1000);
    ESP.restart();
  }

  // switch modes.
  switch (pairingStatus)
  {
  // PAIR REQUEST
  case PAIR_REQUEST:
    if (strcmp(SERIAL_DEFAULT, print_device_serial(server_mac_address)) == 0)
    {
      // check if no more attemps are allowed.
      if (pair_request_attempts >= MAX_PAIR_ATTEMPTS)
      {
        // ends pairing process.
        pairingStatus = NOT_PAIRED;
        pair_request_attempts = 0;
        ESP_LOGI(TAG, "auto-pairing process FINISHED... couln't find the server.");
        break;
      }
    }
    pair_request_attempts++;
    ESP_LOGD(TAG, "Sending PR# %d on channel: %d", pair_request_attempts, radio_channel);

    // set pairing data to send to the server
    pairing_data.msg_type = PAIRING;
    pairing_data.sender_role = CONTROLLER;
    pairing_data.channel = radio_channel;

    // add peer and send request
    if (add_peer_to_plist(server_mac_address, radio_channel) != ESP_OK)
    { // mac address stored in global var.
      ESP_LOGE(TAG, "[esp-now] couldn't add peer to peer list.");
      pairingStatus = NOT_PAIRED;
      break;
    }

    //- send pair data.
    send_result = esp_now_send(server_peer.peer_addr, (uint8_t *)&pairing_data, sizeof(pairing_data));
    //-log
    log_on_result(send_result);

    lastPairingRequest = currentMillis;
    pairingStatus = PAIR_REQUESTED;
    break;

  // PAIR REQUESTED
  case PAIR_REQUESTED:
    // change wifi channel for continue with the pairing process.
    if (currentMillis - lastNetworkLedBlink > 125L)
    { // 4hz blink
      lastNetworkLedBlink = currentMillis;
      digitalWrite(NETWORK_LED, !digitalRead(NETWORK_LED));
    }

    // time out to allow receiving response from server
    if (currentMillis - lastPairingRequest > ESP_NOW_WAIT_PAIR_RESPONSE)
    {
      ESP_LOGI(TAG, "[autopairing] time out expired, try on the next channel");
      radio_channel++;
      if (radio_channel > MAX_CHANNEL)
      {
        radio_channel = 1;
      }
      // set WiFi channel
      pairingStatus = PAIR_REQUEST;
    }
    break;

  // NOT PAIRED
  case NOT_PAIRED:
    if (currentMillis - lastNetworkLedBlink > 500L)
    { // 1hz blink
      lastNetworkLedBlink = currentMillis;
      digitalWrite(NETWORK_LED, !digitalRead(NETWORK_LED));
    }
    if (now_btn_state && currentMillis - lastNowBtnChange > 3000L)
    {
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
    if (currentMillis - lastNetworkLedBlink > 50L)
    {
      lastNetworkLedBlink = currentMillis;
      network_led_pulse_effect();
    }

    if (currentMillis - lastEspnowReceived >= ESP_NOW_WAIT_SERVER_MSG)
    {
      ESP_LOGI(TAG, "Timeout since last server message received. Setting up PR Mode.");
      pairingStatus = PAIR_REQUEST;
    }

    //- resonds to the server...
    if (postEspnowFlag && currentMillis - lastEspnowReceived >= ESP_NOW_POST_INTERVAL)
    {
      // set flag to false.
      postEspnowFlag = false;
      // Set values to send
      outgoing_data.msg_type = DATA;
      outgoing_data.sender_role = CONTROLLER;
      outgoing_data.alarm_code = calculated_alarm_code;
      outgoing_data.air_return_temp = air_return_temp;
      outgoing_data.air_supply_temp = air_supply_temp;
      outgoing_data.drain_switch = float_sw_state;
      outgoing_data.cooling_relay = compressor_state;
      outgoing_data.fan_relay = fan_state;
      outgoing_data.seconds_since_last_cooling_rq = compressorRunningSeconds;

      send_result = esp_now_send(server_peer.peer_addr, (uint8_t *)&outgoing_data, sizeof(outgoing_data));
      log_on_result(send_result);
    }
    //-
    break;
  }

  //-end
  return;
}

// -setup
void setup()
{
  // SETUP BEGIN
  Serial.begin(115200);
  esp_log_level_set(TAG, ESP_LOG_INFO); // set all logger on info.
  ESP_LOGD(TAG, "** setup start. **");

#ifndef ESP32
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB
#endif

  // PINS SETUP
  ESP_LOGI(TAG, "pins settings");
  pinMode(ALARM_RELAY, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(COMPRESSOR_RELAY, OUTPUT);
  pinMode(NETWORK_LED, OUTPUT);
  pinMode(Q3_INPUT, INPUT_PULLUP);
  pinMode(Q2_INPUT, INPUT_PULLUP);
  pinMode(Q1_INPUT, INPUT_PULLUP);
  pinMode(FLOAT_SWTCH, INPUT);
  pinMode(NOW_CNF, INPUT);
  ESP_LOGI(TAG, "pins settings done.");

  // SPIFFS SETUP
  if (!LittleFS.begin(true))
  {
    ESP_LOGE(TAG, "Ocurrió un error al iniciar SPIFFS..");
    while (1)
    {
      ;
    }
  }

  // OneWire SETUP
  ESP_LOGI(TAG, "OneWire sensors settings!");
  air_return_sensor.begin();
  air_return_sensor.setResolution(tempSensorResolution);
  air_return_sensor.setWaitForConversion(false);
  air_supply_sensor.begin();
  air_supply_sensor.setResolution(tempSensorResolution);
  air_supply_sensor.setWaitForConversion(false);
  // first temp. request.
  air_return_sensor.requestTemperatures();
  air_supply_sensor.requestTemperatures();
  lastTempRequest = millis();
  tempRequestDelay = 750 / (1 << (12 - tempSensorResolution));
  ESP_LOGI(TAG, "OneWire sensors settings done.");

  //- WiFi SETUP
  ESP_LOGI(TAG, "WiFi settings.");
  WiFi.mode(WIFI_MODE_STA);
  ESP_LOGI(TAG, "MAC Address:  ->");
  ESP_LOGI(TAG, "%s", WiFi.macAddress().c_str());
  WiFi.disconnect();
  ESP_LOGI(TAG, "WiFi settings done.");

  //- ESP-NOW SETUP
  ESP_LOGI(TAG, "esp-now settings");
  uint8_t mac_buffer[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_AP, mac_buffer);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "could not read mac address.");
  }
  //-
  if (esp_now_init() != ESP_OK)
  {
    ESP_LOGE(TAG, "-- Error initializing ESP-NOW, please reboot --");
    while (1)
    {
      ;
    }
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  //-done
  ESP_LOGI(TAG, "[esp-now] settings done.");

  // LOAD DATA FROM FS
  load_server_from_fs();
  // load_hourmeter_from_fs();

  currentMillis = millis();
  lastCompressorTurnOff = currentMillis;

  // SETUP FINISHED
  ESP_LOGI(TAG, "setup finished --!.");
  delay(500);
}

void loop()
{
  //-
  update_temperature_readings();
  update_IO();
  update_time_counter();
  espnow_loop();
  system_logs();
}