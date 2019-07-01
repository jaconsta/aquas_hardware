#ifndef AQUAS_CONSTANTS_H
#define AQUAS_CONSTANTS_H

#define mqtt_port 1883
// #define MQTT_USER "username"
// #define MQTT_PASSWORD "password"
// #define MQTT_SERIAL_PUBLISH_CH
#define MQTT_SERIAL_RECEIVER_CH "/pomelo/water/84RV4O"
#define MQTT_SERIAL_HEARTBEAT_CH "/pomelo/server/heartbeat"
#define MQTT_SERIAL_SPRINKLE_CH "/pomelo/server/sprinkle/response"

class AquasConstants {
  public:
    const char* device_code = "XXXAAA";
    const char* ssid = "Wifi_name";
    const char* password =  "wifi_password";

    const char* mqtt_server = "192.168.1.20";
};

#endif
