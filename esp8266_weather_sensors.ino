
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define LED 2     //onboard LED
#define SEALEVELPRESSURE_HPA (1013.25)  //Sea level constant

// Wifi credentials
const char* ssid = "KabelBox-1B6C";
const char* password = "80299875789367562743";

//MQTT broker
const char* mqtt_server = "138.3.246.220";

// MQTT client initialization
WiFiClient espClient;
PubSubClient client(espClient);
#define MQTTQOS1

Adafruit_BME280 bme;  //I2C BME
SHT31 sht;            //I2C sht

// GPIO where the DS18B20 is connected to
const int oneWireBus = 0; 
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
int sensorValue = 0;         // value read from the pot

//wifi initialization
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client", "jezerca", "Password@2")) {
      Serial.println("connected");  
      // Subscribe or resubscribe to a topic
      // You can subscribe to more topics (to control more LEDs in this example)
      //client.subscribe("room/lamp");
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  setup_wifi();
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  client.setServer(mqtt_server, 1883); 
  Wire.begin();
  Serial.println(F("BME280 test"));
  bool status = bme.begin(0x76);   //bme I2C Address
  sht.begin(0x44);            //sht30 I2C Address  
  
  // Start the DS18B20 sensor
  sensors.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Serial.println("-- Default Test --");
  unsigned long delayTime = 5000;

  //sht
  Wire.setClock(100000);
  uint16_t stat = sht.readStatus();
  Serial.print(stat, HEX);
  Serial.println();
}

void loop() { 
  if (!client.connected()) {
    reconnect();
  }
  if(!client.loop())  
  client.connect("ESP8266Client");
  
  //ShT30 sensor
  sht.read();
  delay(2000);
  float sht_t = sht.getTemperature();
  float sht_h = sht.getHumidity();
  digitalWrite(LED, LOW);
  Serial.print("SHT30 Temperature: ");
  Serial.print(sht_t, 1);
  Serial.println("ºC");
  Serial.print("SHT30 Humidity: ");
  Serial.print(sht_h, 1);
  Serial.print("%");
  Serial.println("\n");
  client.publish("esp8266/sht_t", String(sht_t).c_str());
  client.publish("esp8266/sht_h", String(sht_h).c_str());
  delay(1000);
  
  //DS18b20 sensor
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print("DS18B20: ");
  Serial.print(temperatureC);
  Serial.print("ºC");
  Serial.println("\n");
  client.publish("esp8266/ds18b20", String(temperatureC).c_str());
  delay(1000);
  
  // UV sensor
  sensorValue = analogRead(analogInPin);
  Serial.print("UV sensor: ");
  Serial.print(sensorValue);
  Serial.print("mV");
  Serial.println("\n");
  client.publish("esp8266/uv", String(sensorValue).c_str());
  delay(1000);

  float bme_t = bme.readTemperature();
  Serial.print("BME Temperature: ");
  Serial.print(bme_t);
  Serial.println("*C");
  client.publish("esp8266/bme_t", String(bme_t).c_str());

  float bme_p = bme.readPressure() / 100.0;
  Serial.print("BME Pressure: ");
  Serial.print(bme_p);
  Serial.println("hPa");
  client.publish("esp8266/bme_p", String(bme_p).c_str());
  
  float bme_a = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("BME Approx. Altitude: ");
  Serial.print(bme_a);
  Serial.println("m");
  client.publish("esp8266/bme_a", String(bme_a).c_str());

  float bme_h = bme.readHumidity();
  Serial.print("BME Humidity: ");
  Serial.print(bme_h);
  Serial.print("%");
  Serial.println("\n");
  digitalWrite(LED, HIGH);
  client.publish("esp8266/bme_h", String(bme_h).c_str());

}
