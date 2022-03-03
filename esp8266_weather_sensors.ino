
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Wifi credentials
const char* ssid = "KabelBox-1B6C";
const char* password = "80299875789367562743";

// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
const char* mqtt_server = "138.3.246.220";

// Initializes the espClient. You should change the espClient name if you have multiple ESPs running in your home automation system
WiFiClient espClient;
PubSubClient client(espClient);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

uint32_t start;
uint32_t stop;
SHT31 sht;

// GPIO where the DS18B20 is connected to
const int oneWireBus = 2; 
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
int sensorValue = 0;  // value read from the pot

// Don't change the function below. This functions connects your ESP8266 to your router
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
    // Attempt to connect
    /*
     YOU MIGHT NEED TO CHANGE THIS LINE, IF YOU'RE HAVING PROBLEMS WITH MQTT MULTIPLE CONNECTIONS
     To change the ESP device ID, you will have to give a new name to the ESP8266.
     Here's how it looks:
       if (client.connect("ESP8266Client")) {
     You can do it like this:
       if (client.connect("ESP1_Office")) {
     Then, for the other ESP:
       if (client.connect("ESP2_Garage")) {
      That should solve your MQTT multiple connections problem
    */
    if (client.connect("ESP8266Client", "jezerca", "Password@2")) {
      Serial.println("connected");  
      // Subscribe or resubscribe to a topic
      // You can subscribe to more topics (to control more LEDs in this example)
      client.subscribe("room/lamp");
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
  Serial.begin(9600);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  //client.setCallback(callback);
  
  Wire.begin();
  Serial.println(F("BME280 test"));

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);
  
  sht.begin(0x44);    //Sensor I2C Address  
  
  // Start the DS18B20 sensor
  sensors.begin();
  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delayTime = 5000;


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

    
  printValues();
  delay(delayTime);

  //ShT30 sensor
  sht.read();
  Serial.print("SHT30 Temperature: ");
  Serial.print(sht.getTemperature(), 1);
  Serial.println("ºC");
  Serial.print("SHT30 Humidity: ");
  Serial.print(sht.getHumidity(), 1);
  Serial.print("%");
  Serial.println("\n");

  //DS18b20 sensor
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print("DS18B20: ");
  Serial.print(temperatureC);
  Serial.print("ºC");
  Serial.println("\n");

  // UV sensor
  sensorValue = analogRead(analogInPin);
  Serial.print("UV sensor: ");
  Serial.print(sensorValue);
  Serial.print("mV");
  Serial.println("\n");
  
  client.publish("test", "prova123");
  
  delay(5000);
}

void printValues() {
  Serial.print("BME Temperature: ");
  Serial.print(bme.readTemperature());
  Serial.println("*C");
  
  Serial.print("BME Pressure: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println("hPa");

  Serial.print("BME Approx. Altitude: ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println("m");

  Serial.print("BME Humidity: ");
  Serial.print(bme.readHumidity());
  Serial.print("%");
  Serial.println("\n");

  Serial.println();
}
