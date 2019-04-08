
/**
   Pomelo device connection.

   Connects the esp32 to a wifi network and to a mqtt server

   When orders are received. It will start the sprinkle device.

*/

#include <WiFi.h>
#include <PubSubClient.h>
// I need to update json to 6
#include <ArduinoJson.h>
#include <Scheduler.h>

const char* device_code = "MBBSBG";
// Wifi settings
const char* ssid = "";
const char* password =  "";
// MQTT settings
const char* mqtt_server = "";
#define mqtt_port 1883
// #define MQTT_USER "username"
// #define MQTT_PASSWORD "password"
// #define MQTT_SERIAL_PUBLISH_CH
#define MQTT_SERIAL_RECEIVER_CH "/pomelo/water/DDKCIZ"
#define MQTT_SERIAL_HEARTBEAT_CH "/pomelo/server/heartbeat"
// IO pins
const int LED_BUILTIN = 2;
const int SPRINKLE_PUMP = 4;
// Create scheduler
Scheduler scheduler = Scheduler();
bool heartbeat_scheduled = false;

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

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  randomSeed(micros()); // The pseudo-random number generator
  Serial.println("Connected to the WiFi network");
  blink(1000, 2);
}

// Connect to the MQTT broker
void mqtt_reconnect() {
  Serial.println("try to connect to mqtt");
  while (!mqttClient.connected()) {
    // Mqtt clientID must be random
    String clientId = "ESPomel32-";
    clientId += String(random(0xffff), HEX);
    // Connect
    if (mqttClient.connect(clientId.c_str(), "", "")) {
      Serial.print("MQTT connected");
      mqttClient.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serial.print("MQTT connection failed");
      Serial.println(mqttClient.state());
      delay(500);
    }
  }
}

void runSprinkle(int sprinkle_milli) {
    digitalWrite(SPRINKLE_PUMP, HIGH);
    delay(sprinkle_milli);
    digitalWrite(SPRINKLE_PUMP, LOW);
}

void runActuator(JsonVariant event) {
  const char* actuator = event["actuator"].as<char*>();
  const char* action = event["action"].as<char*>();
  if(strcmp(actuator, "sprinkle") == 0) {
      if (strcmp(action, "now") == 0) {
        runSprinkle(3000);
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
  Serial.println("finished");
  return;
}

void send_heartbeat() {
  const int capacity=JSON_OBJECT_SIZE(4);
  StaticJsonBuffer<capacity> json_buffer;
  JsonObject& doc = json_buffer.createObject();
  
  doc["type"] = "heartbeat";
  doc["device"] = device_code;
  doc["code"] = (char*)0;

  // output buffer
  char output[128];
  doc.printTo(output, sizeof(output));

  // Send mqtt message
  Serial.println("Sending heartbeat");
  mqttClient.publish(MQTT_SERIAL_HEARTBEAT_CH, output);
  heartbeat_scheduled = false;
}

void schedule_heartbeat() {
  if (heartbeat_scheduled==false) {
    scheduler.schedule(send_heartbeat, 240000); // four minutes
    heartbeat_scheduled = true;
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
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqtt_reconnect();
  runSprinkle(1000);
}

void loop() {
  scheduler.update();
  schedule_heartbeat();
  mqttClient.loop();
}
