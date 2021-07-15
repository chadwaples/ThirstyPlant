/*
 * Project L14_04_PlantWater
 * Description: Plant watering system 
 * Author: Chad Waples
 * Date: 7/12/2021
 */


#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include <Grove_Air_quality_Sensor.h>
#include "credentials.h"


// PUMP
const int PUMPPIN = D11; 


String DateTime, TimeOnly;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(-1);

// Moisture Sensor
const int SOILPIN = A0;
int soilValue = 0;

//GroveDustSensor
const int GROVESENSOR = A5;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 2000;//sampe 30s&nbsp;;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

//AirQualitySensor
AirQualitySensor airSensor(A4);

//BME280
Adafruit_BME280 bme;
unsigned status;
float tempC;
float tempF;
float pressPA;
float inHg;
float humidRH;

TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

Adafruit_MQTT_Publish mqttSoil = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soilmoisture");
Adafruit_MQTT_Publish mqttPressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
Adafruit_MQTT_Publish mqttTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish mqttHumid = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish mqttdust = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustsensor");
Adafruit_MQTT_Publish mqttair = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airsensor");
Adafruit_MQTT_Subscribe mqttManual = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pumpwater");

unsigned long last, lastTime;
int manualValue;

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {

waitFor(Serial.isConnected, 15000); //wait for Serial Monitor to startup
 WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }

Serial.begin(9600);
status = bme.begin(0x76);
    // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&mqttManual);
 
display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
pinMode (SOILPIN, INPUT);
pinMode(PUMPPIN, OUTPUT);
pinMode(GROVESENSOR,INPUT);
 starttime = millis();//get the current time;

//airSensor
Serial.println("Waiting on airSensor to init...");
if (airSensor.init()) {
  Serial.println("airSenor ready.");
  }
  else {
    Serial.print("Sensor Error!");
  }

display.display();
}
// Time.zone(-6);
// Particle.syncTime();


void loop() {
  // Validate connected to MQTT Broker
  MQTT_connect();
  AirQualityRead(); 
  OLEDAir(); 
  GetBMEvalues();
  dustSensor();

 
     // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }

  soilValue = analogRead(SOILPIN);
  if (soilValue>3000) {
    initPumpPin();
    }

  manualButton();

    // publish to cloud every 30 seconds
   if((millis()-lastTime > 60000)) {
    if(mqtt.Update()) {
      mqttSoil.publish(soilValue);
      mqttPressure.publish(inHg);
      mqttTemp.publish(tempF);
      mqttHumid.publish(humidRH);
      mqttair.publish(AirQualityRead());
      mqttdust.publish(concentration);
      Serial.printf("Soil moisture is %i \n",soilValue); 
      } 
    lastTime = millis();
  }
  
OLEDdisplay();
 
}
  


// FUNCTIONS (OLEDdisplay, GetBMEvalues, initPumpPin, manualButton, dustSensor, 
// ... MQTT_Connect, OLEDAir, AirQualityRead)

void OLEDdisplay(void) {
  display.clearDisplay();
  display.setRotation(2);  // "display.setRotation();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(10,20);             // Start at top-left corner

  display.printf("Soil moisture is %i\n", soilValue);
  display.printf("Temperature is %f\n", tempF);
  display.printf("Humidity is %f\n", humidRH);
  display.printf("Pressure is %f\n", inHg);
 
  //display.setTextSize(1);             // Draw 2X-scale text
  display.display();
}

void GetBMEvalues() {
  tempC = bme.readTemperature();
  tempF = (tempC * 1.8) + 32;
  pressPA = bme.readPressure();
  inHg = (pressPA * .0002953);
  humidRH = bme.readHumidity();
  if ((millis()-last)>60000) {
      Serial.printf("Temp =%f\n, Pressure = %f\n, Humidity = %f\n", tempF, inHg, humidRH);
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
}

// Function to initialize PumpPin to pump water
void initPumpPin() {
  digitalWrite (PUMPPIN, HIGH);
  delay(500);
  digitalWrite(PUMPPIN, LOW);
  }

void manualButton() {
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &mqttManual) {
      manualValue = atoi((char *)mqttManual.lastread);
          Serial.println(manualValue);
          Serial.printf("Received %i from Adafruit.io feed pumpwater \n",manualValue);
    }
    if (manualValue == 1){
          digitalWrite(PUMPPIN, HIGH);
          } 
    if (manualValue == 0) {
      digitalWrite(PUMPPIN, LOW);
    }
  }
}

void dustSensor(){  
  duration = pulseIn(GROVESENSOR, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;
  if ((millis()-starttime) >= sampletime_ms)//if the sampel time = = 30s
  {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=&gt;100
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    Serial.print("Concentration = ");
    Serial.print(concentration);
    Serial.println(" pcs/0.01cf");
    Serial.println("\n");
    lowpulseoccupancy = 0;
    starttime = millis();
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       }
  Serial.printf("MQTT Connected!\n"); // makes sure we're connected to the server, prevents timeout, good to use at the bottom of code. 
}

void OLEDAir() {
  int _AirQuality;
  _AirQuality= AirQualityRead();
   if ((millis()-last)>2000) {
      last = millis();
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10,20);

  if (_AirQuality==0){
    display.printf("Fresh Air");
  }
  if (_AirQuality==1){
    display.printf("Low \npollution!");
  }
  if ( _AirQuality==2){
    display.printf("High \npollution!");
  }
  if (_AirQuality==3){
    display.printf(" Extreme\npollution!");
  }
  
  display.display();
}


int AirQualityRead() {
  int quality = airSensor.slope();

  Serial.print("Sensor value: ");
  Serial.println(airSensor.getValue());
  
  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    Serial.println("High pollution! Force signal active.");
    return 3;
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    Serial.println("High pollution!");
    return 2;
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION) {
    Serial.println("Low pollution!");
    return 1; 
  }
  else if (quality == AirQualitySensor::FRESH_AIR) {
    Serial.println("Fresh air.");
    return 0;
  }
}