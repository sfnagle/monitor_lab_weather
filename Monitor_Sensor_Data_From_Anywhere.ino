#include "arduino_secrets.h"

/*
* Monitor Sensor Data from Anywhere
* 
* Rob Reynolds, Mariah Kelly, SparkFun Electronics, 2022
* 
* This sketch will collect data from a BME280, a SGP40, and use a
* SparkFun ESP32 Thing Plus ESP32 WROOM to send the data
* over WiFi to a KaaIoT dashboard.
* https://www.kaaiot.com/
* Want to support open source hardware and software?
* Why not buy a board from us!
* Thing Plus ESP32 WROOM - https://www.sparkfun.com/products/17381
* SparkFun Air Quality Sensor - SGP40 (Qwiic) - https://www.sparkfun.com/products/18345
* SparkFun Atmospheric Sensor Breakout - BME280 (Qwiic) - https://www.sparkfun.com/products/15440 
* 
* License: This code is public domain but you can buy us a beer if you use 
* this and we meet someday at the local (Beerware License).
* 
*/


// First we'll install all of the necessary libraries
#include <Adafruit_SCD30.h>
#include <Wire.h>
#include <WiFi.h>
#include <math.h>

// WifiLocation - Version: Latest 
// #include <WifiLocation.h>
// #include <bingMapsGeocoding.h>

#include "SparkFun_SGP40_Arduino_Library.h" // Go here to get the library:     http://librarymanager/All#SparkFun_SGP40
#include "SparkFunBME280.h"

#include <PubSubClient.h> // Download here: https://github.com/knolleary/pubsubclient/archive/refs/tags/v2.8.zip 
#include <ArduinoJson.h>  // This library can be found in the library manager search bar!
#include "kaa.h" // This one can be found in the LM search bar as well!

#define KAA_SERVER "mqtt.cloud.kaaiot.com"
#define KAA_PORT 1883
#define KAA_TOKEN "BwUGuoiY82"     //Put your KaaIoT Token here (Created in KaaIoT)
#define KAA_APP_VERSION "cedjlpidblakusobpmi0-10"  //Put your auto-generated App Version here

#define RECONNECT_TIME  5000 //ms
#define WINDOW_SIZE     10000 //ms
#define SAMPLE_INTERVAL 2000 //ms
#define SEND_TIME       30000 //ms

// Define output names for our sensor data here
#define COMMAND_TYPE "OUTPUT_SWITCH"
#define OUTPUT_1_NAME "temperature"
#define OUTPUT_2_NAME "humidity"
#define OUTPUT_3_NAME "VOC"
#define OUTPUT_4_NAME "altitude"
#define OUTPUT_5_NAME "pressure"
#define OUTPUT_6_NAME "dewpointC"
#define OUTPUT_7_NAME "CO2"

const char* ssid = "RLE";     //WiFi network goes here
const char* password = "";     //WiFi Password goes here
const float refPressure = 102200; // Metar pressure from nearest airport, in Pa

char mqtt_host[] = KAA_SERVER;
unsigned int mqtt_port = KAA_PORT;

unsigned long now = 0;
unsigned long last_reconnect = 0;
unsigned long last_msg = 0;
unsigned long sample_t0;
unsigned long sample_now;

struct sample_values {
  float  TempF;
  float  Humidity;
  float  VOCindex;
  float  AltitudeFeet;
  float  Pressure;
  float  DewpointC;
  float  CO2;
};

WiFiClient espClient;
PubSubClient client(espClient);
Kaa kaa(&client, KAA_TOKEN, KAA_APP_VERSION);

#define PRINT_DBG(...) printMsg(__VA_ARGS__)

BME280 mySensor;
SGP40  myVOCSensor;
Adafruit_SCD30  myCO2Sensor; 

void setup() {
  Serial.begin(115200);
  Serial.println("Reading basic values from BME280 and SGP40");

  //mySensor.enableDebugging(); // Uncomment this line to print useful debug messages to Serial

  setupWifi();
  client.setServer(mqtt_host, mqtt_port);
  client.setCallback(callback);

  Wire.begin();

  //Initialize Sparkfun sensors
  if (myVOCSensor.begin() == false) {
    Serial.println(F("SGP40 not detected. Check connections. Freezing..."));
    while (1); // Do nothing more
  }
  if (mySensor.beginI2C() == false) {   //Begin communication over I2C
    Serial.println("The sensor did not respond. Please check wiring.");
    while (1); //Freeze
  }
  mySensor.setReferencePressure(refPressure);
  
  
  // Initialize Adafruit SCD-30
  if (!myCO2Sensor.begin()) {
    Serial.println("Failed to find SCD30 chip");
    while (1) { delay(10); }
  }
  Serial.println("SCD30 Found!");

  // if (!scd30.setMeasurementInterval(10)){
  //   Serial.println("Failed to set measurement interval");
  //   while(1){ delay(10);}
  // }
  Serial.print("Measurement Interval: "); 
  Serial.print(myCO2Sensor.getMeasurementInterval()); 
  Serial.println(" seconds");

}

void loop() {
  int max_samples;
  int samples_taken;
  float AvgReadTempC;
  float AvgReadFloatHumidity;
  float AvgGetVOCindex;
  float AvgReadFloatAltitudeFeet;
  float AvgReadFloatPressure;
  float AvgReadDewpointC;
  float AvgReadCO2;
  max_samples = (int)floor((float)WINDOW_SIZE/(float)SAMPLE_INTERVAL) + 1;
  float samples[7][max_samples];
  sample_values outputs;
  
  if (!client.connected()) { //Checking connection
    now = millis();
    if ( ((now - last_reconnect) > RECONNECT_TIME) || (now < last_reconnect) ) {
      last_reconnect = now;
      reconnect();
    }
    return;
  }
  client.loop();

  sample_t0 = millis();
  sample_now = sample_t0;
  samples_taken = 0;
  while (((sample_now - sample_t0) < WINDOW_SIZE) || (sample_now < sample_t0)) {

    //Add samples to the averaging list
    if (samples_taken < max_samples) {
      delay(SAMPLE_INTERVAL);
      outputs = getOutputsState();
      samples[0][samples_taken] = outputs.TempF;
      samples[1][samples_taken] = outputs.Humidity;
      samples[2][samples_taken] = outputs.VOCindex ;
      samples[3][samples_taken] = outputs.AltitudeFeet ;
      samples[4][samples_taken] = outputs.Pressure ;
      samples[5][samples_taken] = outputs.DewpointC;
      samples[6][samples_taken] = outputs.CO2;
      Serial.print("CO2: ");
      Serial.print(outputs.CO2, 1);
      Serial.println(" ppm");
      Serial.println("");
      Serial.print("Samples taken: ");
      Serial.print(samples_taken, 0);
      Serial.println("");
      samples_taken++;
      }
    sample_now = millis();
  }
  samples_taken--;
  Serial.print("Samples taken: ");
  Serial.print(samples_taken, 3);
  Serial.println("");

  // //Sending average to cloud on a different timebase
  // now = millis();
  // if ( ((now - last_msg) > SEND_TIME) || (now < last_msg) ) {
  //   last_msg = now;

  // calculate averages
  AvgReadTempC = 0.;
  AvgReadFloatHumidity = 0.;
  AvgGetVOCindex = 0.;
  AvgReadFloatAltitudeFeet = 0.;
  AvgReadFloatPressure = 0.;
  AvgReadDewpointC = 0;
  AvgReadCO2 = 0;
  for (int ii=0; ii < samples_taken; ii++) {
    AvgReadTempC += samples[0][ii];
    AvgReadFloatHumidity += samples[1][ii];
    AvgGetVOCindex += samples[2][ii];
    AvgReadFloatAltitudeFeet += samples[3][ii];
    AvgReadFloatPressure += samples[4][ii];
    AvgReadDewpointC += samples[5][ii];
    AvgReadCO2 += samples[6][ii];
  }
  AvgReadTempC = (float)AvgReadTempC/(float)samples_taken;
  AvgReadFloatHumidity = (float)AvgReadFloatHumidity/(float)samples_taken;
  AvgGetVOCindex = (float)AvgGetVOCindex/(float)samples_taken;
  AvgReadFloatAltitudeFeet = (float)AvgReadFloatAltitudeFeet/(float)samples_taken;
  AvgReadFloatPressure = (float)AvgReadFloatPressure/(float)samples_taken;
  AvgReadDewpointC = (float)AvgReadDewpointC/(float)samples_taken;
  AvgReadCO2 = (float)AvgReadCO2/(float)samples_taken;
  //Send averages here
  sendAverages(AvgReadTempC, AvgReadFloatHumidity, AvgGetVOCindex, AvgReadFloatAltitudeFeet, AvgReadFloatPressure, AvgReadDewpointC, AvgReadCO2);
}

void printMsg(const char * msg, ...) {
  char buff[256];
  va_list args;
  va_start(args, msg);
  vsnprintf(buff, sizeof(buff) - 2, msg, args);
  buff[sizeof(buff) - 1] = '\0';
  Serial.print(buff);
}

String getChipId() {
  char buf[20];
  uint64_t chipid = ESP.getEfuseMac();
  sprintf(buf, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  return String(buf);
}

void composeAndSendMetadata() {
  StaticJsonDocument<255> doc_data;
  String ipstring = (
                  String(WiFi.localIP()[0]) + "." +
                  String(WiFi.localIP()[1]) + "." +
                  String(WiFi.localIP()[2]) + "." +
                  String(WiFi.localIP()[3])
                );

  // Insert any fixed data you want here - board, location, any other metadata
  doc_data["name"] = "ESP32";
  doc_data["model"] = "SparkFun Thing Plus";
  doc_data["location"] = "Cambridge, MA";
  doc_data["longitude"] = "40Â° 5' 25.5474";
  doc_data["latitude"] = "-105Â° 11' 6.2874";
  doc_data["ip"] = ipstring;
  doc_data["mac"] = String(WiFi.macAddress());
  doc_data["serial"] = String(getChipId());

  kaa.sendMetadata(doc_data.as<String>().c_str());
}

void sendAverages(float AvgReadTempC, float AvgReadFloatHumidity, float AvgGetVOCindex, float AvgReadFloatAltitudeFeet, float AvgReadFloatPressure, float AvgReadDewpointC, float AvgReadCO2) {
  StaticJsonDocument<255> doc_data;

  // This is where we get sensor data, asign it to our outputs, send it to our Dashboard
  doc_data.createNestedObject();
  doc_data[0][OUTPUT_1_NAME] = AvgReadTempC;
  doc_data[1][OUTPUT_2_NAME] = AvgReadFloatHumidity;
  doc_data[2][OUTPUT_3_NAME] = AvgGetVOCindex;
  doc_data[3][OUTPUT_4_NAME] = AvgReadFloatAltitudeFeet;
  doc_data[4][OUTPUT_5_NAME] = AvgReadFloatPressure;
  doc_data[5][OUTPUT_6_NAME] = AvgReadDewpointC;
  doc_data[6][OUTPUT_7_NAME] = AvgReadCO2;
  kaa.sendDataRaw(doc_data.as<String>().c_str());  // send data to Kaa IoT Cloud
}

sample_values getOutputsState() {
  sample_values sampleValues;

  // This is where we send the averages to our Dashboard
  sampleValues.TempF = mySensor.readTempC();  // read sensor data for temperature
  sampleValues.Humidity = mySensor.readFloatHumidity(); // read sensor data for humidity
  sampleValues.VOCindex = myVOCSensor.getVOCindex(); // read VOC sensor data
  sampleValues.AltitudeFeet = mySensor.readFloatAltitudeFeet(); // read sensor data for altitude
  sampleValues.Pressure = mySensor.readFloatPressure(); // read sensor data for pressure
  sampleValues.DewpointC = mySensor.dewPointC(); // read sensor data for dewpoint, in Celsius
  if (myCO2Sensor.dataReady()){
    // Serial.println("Data available!");

    if (!myCO2Sensor.read()){ Serial.println("Error reading sensor data"); sampleValues.CO2 = 1.; }

    // Serial.print("Temperature: ");
    // Serial.print(myCO2Sensor.temperature);
    // Serial.println(" degrees C");
    
    // Serial.print("Relative Humidity: ");
    // Serial.print(myCO2Sensor.relative_humidity);
    // Serial.println(" %");
    
    sampleValues.CO2 = myCO2Sensor.CO2; // read sensor data for OC2, in ppm
    // Serial.print("CO2: ");
    // Serial.print(sampleValues.CO2, 3);
    // Serial.println(" ppm");
    // Serial.println("");
  } else {
    Serial.println("No SCD30 data");
  }
  return sampleValues;
}

void setupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  String ipstring = (
                  String(WiFi.localIP()[0]) + "." +
                  String(WiFi.localIP()[1]) + "." +
                  String(WiFi.localIP()[2]) + "." +
                  String(WiFi.localIP()[3])
                );
  Serial.println();
  PRINT_DBG("WiFi connected\n");
  PRINT_DBG("IP address: %s\n", ipstring.c_str());
}

void callback(char* topic, byte* payload, unsigned int length) {
  PRINT_DBG("Message arrived [%s] ", topic);
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  kaa.messageArrivedCallback(topic, (char*)payload, length);
}

void reconnect() {
  PRINT_DBG("Attempting MQTT connection to %s:%u ...", mqtt_host, mqtt_port);
  // Create client ID
  String clientId = "ESP8266Client-";
  clientId += String(getChipId());
  // Attempt to connect
  if (client.connect(clientId.c_str()))
  {
    PRINT_DBG("connected\n");
    kaa.connect();
    composeAndSendMetadata();
  } else
  {
    PRINT_DBG("failed, rc=%d try again in %d milliseconds\n", client.state(), RECONNECT_TIME);
  }
}
