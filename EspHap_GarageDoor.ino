
////  !!!! This sketch is not finished yet
///   It just shows how to add/integrate garage door charachteristic with apple
///   garage door functionality should be reviewed and properly implemented


#define ENABLE_WEB_SERVER    //if we want to have built in web server /site
#define ENABLE_OTA  //if Over the air update need  , ENABLE_WEB_SERVER must be defined first


#include <Arduino.h>


#include "simplesensor.h"
#include "config.h"

#ifdef ENABLE_OTA
#include "wifi_ota.h"
#endif

#ifdef ESP32
#include <SPIFFS.h>
#endif
#ifdef ESP8266
#include <ESP8266WiFi.h>

#include <ESP8266mDNS.h>
#include "coredecls.h"
#endif

#ifdef ENABLE_WEB_SERVER
#ifdef ESP8266
#include <ESP8266WebServer.h>
ESP8266WebServer server(80);
#endif

#ifdef ESP32
#include <WebServer.h>
WebServer server(80);
#endif
#endif


#if defined(ESP32) && defined(ENABLE_OTA)
#include <Update.h>
#endif

#ifdef ENABLE_WEB_SERVER
#include "spiffs_webserver.h"
bool isWebserver_started = false;
#endif

#include <WiFiManager.h>        //https://github.com/tzapu/WiFiManager


#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);


// For reporting heap usage on the serial output every 5 seconds
static uint32_t next_heap_millis = 0;
static uint32_t next_led_millis = 0;

extern "C" {
#include "homeintegration.h"
}
#ifdef ESP8266
#include "homekitintegrationcpp.h"
#endif
#include "hapfilestorage/hapfilestorage.hpp"


#include "spiffs_webserver.h"


homekit_service_t* service_garagedoor = NULL;
const int relay_gpio = 5;
const int sensor_open_gpio = 14;
const int sensor_close_gpio = 4;
SimpleSensor Sensor_open(sensor_open_gpio);
SimpleSensor Sensor_close(sensor_close_gpio);

void sensor_callback(uint8_t gpio_num, uint8_t state)
{
  current_door_state_update_from_sensor();
}

uint8_t current_door_state = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN; //HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN;
uint8_t target_door_state = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN;//HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN;

void startwifimanager() {
  WiFiManager wifiManager;
  if (!wifiManager.autoConnect(homekit_name, NULL)) {
    ESP.restart();
    delay(1000);
  }
}


void setup() {
#ifdef ESP8266
  disable_extra4k_at_link_time();
#endif
  Serial.begin(115200);
  delay(10);

  // We start by connecting to a WiFi network
#ifdef ESP32
  if (!SPIFFS.begin(true)) {
    // Serial.print("SPIFFS Mount failed");
  }
#endif
#ifdef ESP8266
  if (!SPIFFS.begin()) {
    Serial.print("SPIFFS Mount failed");
  }
#endif
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(relay_gpio, OUTPUT);
  Sensor_open.start(sensor_callback);
  Sensor_close.start(sensor_callback);


  startwifimanager();

#ifdef ENABLE_OTA
  wifi_ota_setup();
#endif


  /// now will setup homekit device

  //this is for custom storaage usage
  // In given example we are using \pair.dat   file in our spiffs system
  //see implementation below
  Serial.print("Free heap: ");
  Serial.println(system_get_free_heap_size());

  init_hap_storage("/pair.dat");

  /// We will use for this example only one accessory (possible to use a several on the same esp)
  //Our accessory type is light bulb , apple interface will proper show that
  hap_setbase_accessorytype(homekit_accessory_category_door);
  /// init base properties
  hap_initbase_accessory_service(homekit_name, "Arduino Homekit", "123456789", homekit_name, "1.0");


  // for base accessory registering temperature
  service_garagedoor = hap_add_garagedoor_service("garagedoor", hap_callback_process, 0);

  hap_init_homekit_server();

#ifdef ENABLE_WEB_SERVER
  String strIp = String(WiFi.localIP()[0]) + String(".") + String(WiFi.localIP()[1]) + String(".") +  String(WiFi.localIP()[2]) + String(".") +  String(WiFi.localIP()[3]);
#ifdef ESP8266
  if (hap_homekit_is_paired()) {
#endif
    Serial.println(PSTR("Setting web server"));
    SETUP_FILEHANDLES
//    server.on("/get", handleGetVal);
//    server.on("/set", handleSetVal);
    server.begin();
    Serial.println(String("Web site http://") + strIp);
    Serial.println(String("File system http://") + strIp + String("/browse"));
    Serial.println(String("Update http://") + strIp + String("/update"));
    isWebserver_started = true;
#ifdef ESP8266
  } else
    Serial.println(PSTR("Web server is NOT SET, waiting for pairing"));
#endif

#endif


  obstruction_state_set(0);
  current_door_state_update_from_sensor();

}
void loop() {

#ifdef ESP8266
  hap_homekit_loop();
#endif

  if (isWebserver_started)
    server.handleClient();

#ifdef ENABLE_OTA
  ArduinoOTA.handle();
#endif

  const uint32_t t = millis();

  if (t > next_led_millis) {
    // show heap info every 5 seconds
    digitalWrite(LED_BUILTIN, LOW);  // Change the state of the LED
    delay(5);
    digitalWrite(LED_BUILTIN, HIGH);  // Change the state of the LED
    next_led_millis = t + 1000;
    //LOG_D("WiFi Signal: %d",WiFi.RSSI());
  }


}
bool getSwitchVal() {
  if (service_garagedoor) {
    homekit_characteristic_t * ch = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_ON);
    if (ch) {
      return ch->value.bool_value;
    }
  }
  return false;
}

/*
void handleGetVal() {
  server.send(200, FPSTR(TEXT_PLAIN), getSwitchVal() ? "1" : "0");
}
void handleSetVal() {
  if (server.args() != 2) {
    server.send(505, FPSTR(TEXT_PLAIN), "Bad args");
    return;
  }
  //to do analyze
  if (server.arg("var") == "ch1") {
    if (service_garagedoor) {

      homekit_characteristic_t * ch = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_ON);
      if (ch) {
        set_switch(server.arg("val") == "true");
      }
    }
  }
}*/

/*
void set_switch(bool val) {
  Serial.println(String("set_switch:") + String(val ? "True" : "False"));
  digitalWrite(relay_gpio, val ? HIGH : LOW);
  //we need notify apple about changes

  if (service_garagedoor) {
    Serial.println("notify hap");
    //getting on/off characteristic
    homekit_characteristic_t * ch = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_ON);
    if (ch) {

      if (ch->value.bool_value != val) { //wil notify only if different
        ch->value.bool_value = val;
        homekit_characteristic_notify(ch, ch->value);
      }
    }
  }
}
*/

void notify_hap() {

  if (service_garagedoor) {
    homekit_characteristic_t * ch_currentstate = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE);
    homekit_characteristic_t * ch_targetstate = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE);
    homekit_characteristic_t * ch_obstruction = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_OBSTRUCTION_DETECTED);

  }

}
void relay_write(bool on) {
  digitalWrite(relay_gpio, on ? HIGH : LOW);
}

void current_state_set(uint8_t new_state) {
  homekit_characteristic_t * ch_currentstate = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE);
  // if (ch_currentstate && current_door_state != new_state) {
  current_door_state = new_state;
  HAP_NOTIFY_CHANGES(int, ch_currentstate, new_state, 0)
  // }
}

void target_state_set(uint8_t new_state) {
  homekit_characteristic_t * ch_currentstate = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE);
  //if (ch_currentstate && target_door_state != new_state) {
  target_door_state = new_state;
  HAP_NOTIFY_CHANGES(int, ch_currentstate, new_state, 0)
  //}
}

void obstruction_state_set(uint8_t new_state) {
  homekit_characteristic_t * ch_currentstate = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_OBSTRUCTION_DETECTED);
  HAP_NOTIFY_CHANGES(int, ch_currentstate, new_state, 0)

}

void current_door_state_update_from_sensor() {



  uint8_t state_open = Sensor_open.getstate();
  uint8_t state_close = Sensor_close.getstate();

  if  (  state_open == LOW ) {
    LOG_D("Pin Open closed")
  }
  else {
    LOG_D("Pin Open open");
  }

  if  ( state_close == LOW ) {
    LOG_D("Pin Close closed")
  }
  else {
    LOG_D("Pin Close open");
  }


  uint8_t currentstate;
  uint8_t targetstate;

  currentstate = current_door_state;
  targetstate = target_door_state;

  // Read the sensors and use some logic to determine state
  if ( state_open == LOW ) {
    // If PIN_SENSOR_OPENED is low, it's being pulled to ground, which means the switch at the top of the track is closed, which means the door is open
    targetstate = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN; // NOP, if already set
    currentstate = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN;
  }
  else if ( state_close == LOW ) {
    // If PIN_SENSOR_CLOSED is low, it's being pulled to ground, which means the switch at the bottom of the track is closed, which means the door is closed
    targetstate = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED; // NOP, if already set
    currentstate = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED;
  } else {
    // If neither, then the door is in between switches, so we use the last known state to determine which way it's probably going
    if (currentstate == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED) {
      // Current door state was "closed" so we are probably now "opening"
      currentstate = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING;
    } else if ( currentstate == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN ) {
      // Current door state was "opened" so we are probably now "closing"
      currentstate = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING;
    }

    // If it is traveling, then it might have been started by the button in the garage. Set the new target state:
    if ( current_door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING ) {
      targetstate = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN;
    } else if ( current_door_state = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING ) {
      targetstate = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED;
    }
  }

  current_state_set(currentstate);
  target_state_set(targetstate);

  // ... and then notify HomeKit clients
  LOG_D("Target door state: %i", target_door_state);

  LOG_D("Current door state: %i", current_door_state);


}
void hap_callback_process(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
  Serial.println("hap_callback");
  if (!service_garagedoor) {
    Serial.println("service not defined");
    return;

  }

  homekit_characteristic_t * ch_currentstate = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE);
  homekit_characteristic_t * ch_targetstate = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE);
  homekit_characteristic_t * ch_obstruction = homekit_service_characteristic_by_type(service_garagedoor, HOMEKIT_CHARACTERISTIC_OBSTRUCTION_DETECTED);

  if (ch == ch_targetstate) {
    Serial.println("processing target state");
    if (current_door_state != HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN &&
        current_door_state != HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED) {
      Serial.println("target_state_set  ignored: current state not open or closed.");
      return;
    }
    //target_door_state = new_value.int_value;
    if (current_door_state == value.int_value) {
      Serial.println("target_state_set ignored: target state == current state");
      return;
    }
    Serial.println("Trigger relay");
    relay_write(true);
    // Wait for some time:
    delay(500); ///????
    // Turn OFF GPIO:
    relay_write(false);
    if (current_door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN) {
      current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
    } else {
      current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING);
    }
    LOG_D("Callback Target door state: %i", target_door_state);
    LOG_D("Callback Current door state: %i", current_door_state);

  }


}
