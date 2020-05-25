// SDA: D1
// SCL: D2

#include <Wire.h>
#include "Adafruit_SI1145.h"
#include <BH1750.h>
#include <Adafruit_BME280.h>
#include "Adafruit_VEML6070.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "config.h"

BH1750 lightMeter;
Adafruit_BME280 bme;
Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_VEML6070 uva = Adafruit_VEML6070();


// Network parameters
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

//MQTT Parameters
const String clientId = "weathersta";
const char* mqtt_server = "192.168.2.10";
const int mqttPort = 1883;
const char* mqttUser = MQTT_USER;
const char* mqttPassword = MQTT_PASSWORD;

// Global
unsigned long delayTime;
#define SEALEVELPRESSURE_HPA (1019.8)

WiFiClient espClient;
PubSubClient client(espClient);

//WIFI Setup code
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//Callback function if we ever decide to implement inputs.
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Logic HERE

}

// Reconection to MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String topic = clientId + "/status";
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqttUser, mqttPassword,topic.c_str(), 0, true, "offline")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      
      client.publish(topic.c_str(), "online", true);
      // ... and resubscribe
      //client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  //WIFI
  setup_wifi();

  //MQTT
  client.setServer(mqtt_server, mqttPort);
  client.setCallback(callback);

  //Sensors I2C Comms
  Serial.println("Adafruit SI1145 test");
  if (! uv.begin()) {
    Serial.println("Didn't find Si1145");
    while (1);
  }
  // Initialize light sensor bus. Might need to init Wire.begin()?
  lightMeter.begin(); //GY-302
  bme.begin(0x76); // BME280
  uva.begin(VEML6070_2_T);  //VEML6070 pass in the integration time constant

  Serial.println("ALL OK!");

  delayTime = 2000;
}

void loop() {

  //Check for connection status.
  if (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    reconnect();
  }
  client.loop();

  Serial.println("######## Measure Cycle ##########################");

  // Sensor param initialization 
  float lux, vis, ir, uvindex, temp, pres, alti, humi, rawuv;

  getLUX(lux);
  //logLUX(lux);
  getAdaUV(vis, ir, uvindex);
  //logAdafruitUV(vis, ir, uvindex);
  getAtmo(temp, pres, alti, humi);
  //logAtmo(temp, pres, alti, humi);
  getRawUV(rawuv);

  DynamicJsonDocument doc = publishPayload(lux, vis, ir, uvindex, temp, pres, alti, humi, rawuv);

  Serial.print("Publish message: ");
  char buffer[512];
  size_t n = serializeJson(doc, buffer);
  
  String topic = clientId + "/data";
  Serial.print(topic); Serial.print(": "); Serial.println(buffer);
  client.publish(topic.c_str(), buffer, n);
  
  delay(delayTime);
}

DynamicJsonDocument publishPayload(float luxval, float vis, float ir, float uvindex, float temp, float pres, float alti, float humi, float rawuv) {
  const size_t capacity = 2*JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(5);
  //const size_t capacity = 512;
  DynamicJsonDocument doc(capacity);

  JsonObject light = doc.createNestedObject("light");
  JsonObject light_data = light.createNestedObject("data");
  light_data["visible"] = vis;
  light_data["infrared"] = ir;
  light_data["uv"] = uvindex;
  light_data["uva"] = rawuv;
  light_data["lux"] = luxval;
  

  JsonObject atmospheric = doc.createNestedObject("atmospheric");
  JsonObject atmospheric_data = atmospheric.createNestedObject("data");
  atmospheric_data["temperature"] = temp;
  atmospheric_data["pressure"] = pres;
  atmospheric_data["elevation"] = alti;
  atmospheric_data["humidity"] = humi;

  //serializeJson(doc, Serial);
  return doc;
}

void getRawUV(float &rawuv){
  rawuv = uva.readUV();
}

void getAtmo(float &temp, float &pres, float &alti, float &humi) {
  temp = bme.readTemperature();
  pres = bme.readPressure();
  alti = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humi = bme.readHumidity();
}

void logAtmo(float temp, float pres, float alti, float humi) {
  Serial.println("===== Atmospheric Data =====");
  Serial.print(F("Temperature: ")); Serial.print(temp); Serial.print(" *C");
  Serial.print(F(" / Pressure: ")); Serial.print(pres); Serial.print(" Pa");
  Serial.print(F(" / Altitude: ")); Serial.print(alti); Serial.print(" m");
  Serial.print(F(" / Humidity: ")); Serial.print(humi); Serial.print(" %");

  Serial.println();
}

void getLUX(float &lux) {
  // GY-302
  // Default I2C addr: 0x23
  lux = lightMeter.readLightLevel();
}

void logLUX(float lux) {
  Serial.println("=====LUX=====");
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
}

void getAdaUV(float &vis, float &ir, float &uvindex) {
  // Adafruit Si1145
  //I2C address: 0x60
  vis = uv.readVisible();
  ir = uv.readIR();
  uvindex = uv.readUV();
  uvindex /= 100.0;
}

void logAdafruitUV(float vis, float ir, float uv) {
  Serial.println("=====ADA VIS/IR/UV=====");
  Serial.print("Vis: "); Serial.print(vis);
  Serial.print(" / IR: "); Serial.print(ir);

  // Uncomment if you have an IR LED attached to LED pin!
  //Serial.print("Prox: "); Serial.println(uv.readProx());

  Serial.print(" / UV: ");  Serial.println(uv);
}
