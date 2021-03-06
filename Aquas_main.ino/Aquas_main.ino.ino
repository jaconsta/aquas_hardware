/**
   Pomelo device connection.

   Connects the esp32 to a wifi network and to a mqtt server

   When orders are received. It will start the sprinkle device.

*/

#include <WiFi.h>
#include <PubSubClient.h>
// I need to update json to 6
#include <ArduinoJson.h>
// Aquas libraries
#include "AquasConstants.h"

const int LED_BUILTIN = 2;
const int SPRINKLE_PUMP = 4;

// Create scheduler
unsigned long heartbeat_time_start = 0;
unsigned long max_heartbeat_time_millis = 300000; // 5 min
// Sprinkle scheduler
bool should_run_sprinkle = false;
bool should_send_sprinkle_response = false;
unsigned long sprinkle_response_time_start = 0;
unsigned long max_sprinkle_response_time_millis = 10000; // 10 sec

struct SprinkleEvents {
  char actuator[50];
  char action[50];
  char code[50];
};
struct SprinkleEvents sprinkle_event;

AquasConstants constants = AquasConstants();
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Blink the built-in led
void blink(int delay_time, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delay_time);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delay_time);
  };
}

// Connect to the Wifi network.
void connect_to_wifi() {

  WiFi.begin(constants.ssid, constants.password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  randomSeed(micros()); // The pseudo-random number generator
  Serial.println("Connected to the WiFi network");
  blink(1000, 2);
}

// Connect to the MQTT broker
void mqtt_connect() {
  Serial.println("try to connect to mqtt");
  while (!mqttClient.connected()) {
    // Mqtt clientID must be random
    String clientId = "ESPomel32-";
    clientId += String(random(0xffff), HEX);
    // Connect
    if (mqttClient.connect(clientId.c_str(), "", "")) {
      Serial.println("MQTT connected");
      mqttClient.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serial.println("MQTT connection failed");
      Serial.println(mqttClient.state());
      delay(500);
    }
  }
}

void sprinkle_controller() {
  if (should_run_sprinkle) {
    unsigned long now = millis();
    unsigned long diff = now - sprinkle_response_time_start;
    if (abs(diff) > max_sprinkle_response_time_millis) {
      digitalWrite(SPRINKLE_PUMP, LOW);
      should_run_sprinkle = false;
      if(should_send_sprinkle_response) {
        send_sprinkle_response();
        should_send_sprinkle_response = false;
      }
    }
  }
}

void runSprinkle(int sprinkle_millis) {
    // Trigger the pump
    digitalWrite(SPRINKLE_PUMP, HIGH);
    // Setup the async response
    should_run_sprinkle = true;
    sprinkle_response_time_start = millis();  // now
    max_sprinkle_response_time_millis = sprinkle_millis;
}

void runActuator(JsonVariant event) {
  const char* actuator = event["actuator"].as<char*>();
  const char* action = event["action"].as<char*>();
  const char* code = event["code"].as<char*>();
  int sprinkleTime = 10000;
  if(strcmp(actuator, "sprinkle") == 0) {
    if (strcmp(action, "now") == 0) {
      sprinkleTime = 3000;
    }
    runSprinkle(sprinkleTime);
    should_send_sprinkle_response = true;
    *stpcpy(sprinkle_event.actuator, actuator);
    *stpcpy(sprinkle_event.action, action);
    if (code != NULL) {
      *stpcpy(sprinkle_event.code, code);
    }
  } else {
      Serial.print("Reived an unknown command ");
      Serial.println(actuator);
  }
}

// When a message is recieved from the MQTT broker
void mqttCallback(char* topic, byte *payload, unsigned int payload_length) {
  Serial.println("New mqtt message");
  Serial.write(payload, payload_length);
  Serial.println("");
  
  // Convert message to Json
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& message = jsonBuffer.parseObject((char *)payload);
  if (!message.success()){
    Serial.println("Could not parse MQTT payload");
    return;
  }
  JsonArray& actions = message["actions"];
  for(int i=0; i<actions.size(); i++){
    runActuator(actions[i]);
  }
  return;
}

void clean_sprinkle_event() {
  *stpcpy(sprinkle_event.actuator, "");
  *stpcpy(sprinkle_event.action, "");
  *stpcpy(sprinkle_event.code, "");
}

void send_boot_message() {
  const int capacity=300;
  StaticJsonBuffer<capacity> json_buffer;
  JsonObject& doc = json_buffer.createObject();
  
  doc["type"] = "device_on";
  doc["device"] = constants.device_code;
  doc["code"] = (char*)0;  // Null

  String outputStr;
  doc.printTo(outputStr);
  char* outputChr = strdup(outputStr.c_str());

  // Send mqtt message
  Serial.println("Sending boot");
  mqttClient.publish(MQTT_SERIAL_HEARTBEAT_CH, outputChr);
  Serial.println("Boot sent");
  free(outputChr);
}

void send_heartbeat() {
  const int capacity=300;
  StaticJsonBuffer<capacity> json_buffer;
  JsonObject& doc = json_buffer.createObject();
  
  doc["type"] = "heartbeat";
  doc["device"] = constants.device_code;
  doc["code"] = (char*)0;  // Null

  // output buffer
  // char output[300];
  // doc.printTo(output, sizeof(output));
  String outputStr;
  doc.printTo(outputStr);
  char* outputChr = strdup(outputStr.c_str());

  // Send mqtt message
  Serial.println("Sending heartbeat");
  mqttClient.publish(MQTT_SERIAL_HEARTBEAT_CH, outputChr);
  Serial.println("Heartbeat sent");
  free(outputChr);
}

void send_sprinkle_response() { // (JsonVariant event) {
  Serial.println("Sending sprinkle response");
  const int capacity=250;
  StaticJsonBuffer<capacity> json_buffer;
  JsonObject& doc = json_buffer.createObject();

  
  doc["action"] = sprinkle_event.action;
  doc["device"] = constants.device_code;
  doc["code"] = sprinkle_event.code;

  // output buffer
  char output[128];
  doc.printTo(output, sizeof(output));

  // Send mqtt message
  mqttClient.publish(MQTT_SERIAL_SPRINKLE_CH, output);
  clean_sprinkle_event();
  Serial.println("Sprinkle response sent");
}

void schedule_heartbeat() {
  unsigned long now = millis();
  unsigned long diff = now - heartbeat_time_start;
  if (abs(diff) > max_heartbeat_time_millis) {
    send_heartbeat();
    Serial.println("Heartbeat scheduled");
    heartbeat_time_start = now;
  }
}

void setup() {
  // --- Configurations ---
  // Serial
  Serial.begin(115200);
  // Built-in led
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SPRINKLE_PUMP, OUTPUT);
  // Wifi
  connect_to_wifi();
  // Mqtt
  mqttClient.setServer(constants.mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqtt_connect();
  send_boot_message();
  runSprinkle(1000);
}

void loop() {
  if (!mqttClient.connected()) {
    Serial.println("Mqtt disconnected");
    mqtt_connect();
  } else {
    mqttClient.loop();
  }
  schedule_heartbeat();
  sprinkle_controller();
}
