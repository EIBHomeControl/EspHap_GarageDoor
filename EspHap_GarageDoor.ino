
////  !!!! This sketch is not finished yet
///   It just shows how to add/integrate garage door charachteristic with apple
///   garage door functionality should be reviewed and properly implemented




#include <Arduino.h>
#include "simplesensor.h"
#include "config.h"
#include "wifi_ota.h"

#ifdef ESP32
#include <SPIFFS.h>
#endif
#ifdef ESP8266
#include <ESP8266WiFi.h>

#include <ESP8266mDNS.h>
#include "coredecls.h"
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
#include <hapfilestorage/hapfilestorage.hpp>




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

uint8_t current_door_state = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN;
uint8_t target_door_state = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN;

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
  wifi_ota_setup();


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

  obstruction_state_set(0);
  current_door_state_update_from_sensor();

}
void loop() {

#ifdef ESP8266
  hap_homekit_loop();
#endif

  ArduinoOTA.handle();

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
