#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
/************************* Analog Sensor Setup *********************************/
const int analogInPin = A0;  // Analog input pin that the sensor is attached to
int sensorValue = 0;  // value read from the sensor
int outputValue = 0;  // value output to the server
#define Choke_1 D7
#define Choke_2 D6
#define Ignition_stop D3
#define Ignition_start D4
#define Fuel D2
#define Alarm D0
/************************* WiFi Access Point *********************************/
#define WLAN_SSID "***********" 
#define WLAN_PASS "************"  

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883                         // use 8883 for SSL
#define AIO_USERNAME "Ayoada"                       
#define AIO_KEY "aio_*************************"

/************ Global State ******************/
// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
// Setup a feed called 'sensor' for publishing sensor values.
Adafruit_MQTT_Publish sensorFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensor");
// Setup a feed called 'onoff' for controlling LED.
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");
void MQTT_connect();
void setup() {
  Serial.begin(9600);
  STAT_UP();
  //delay(10);
  pinMode(Choke_1, OUTPUT);
  pinMode(Choke_2, OUTPUT);
  pinMode(Ignation_stop, OUTPUT);
  pinMode(Ignation_start, OUTPUT);
  pinMode(Fuel, OUTPUT);
  pinMode(Alarm, OUTPUT);

  digitalWrite(Choke_1, LOW);
  digitalWrite(Choke_2, LOW);
  digitalWrite(Ignation_stop, LOW);
  digitalWrite(Ignation_start, LOW);
  digitalWrite(Fuel, LOW);
  digitalWrite(Alarm, LOW);
  Serial.println(F("Adafruit MQTT demo"));
  // Connect to WiFi access point.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&onoffbutton);
}
void loop() {
  // Ensure the connection to the MQTT server is alive
  MQTT_connect();
  // Wait 2000 milliseconds for data from subscription feed.
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(2000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);

      char *value = (char *)onoffbutton.lastread;
      String message = String(value);
      message.trim();
      if (message == "ON") {

        //Fuel Control On
        digitalWrite(Fuel, HIGH);
        delay(3000);

        //Choke control
        digitalWrite(Choke_1, HIGH);
        digitalWrite(Choke_1, HIGH);
        delay(3000);

        //ignition control
        digitalWrite(Ignation_start, HIGH);
        digitalWrite(Ignation_stop, LOW);
        delay(1000);
        digitalWrite(Ignation_start, LOW);
        digitalWrite(Ignation_stop, LOW);
        delay(3000);
        //ignition control
        digitalWrite(Choke_1, LOW);
        digitalWrite(Choke_1, LOW);

      }

      else if (message == "OFF") {
        digitalWrite(Ignation_start, LOW);
        digitalWrite(Ignation_stop, HIGH);
        delay(4000);
        digitalWrite(Ignation_start, LOW);
        digitalWrite(Ignation_stop, LOW);
        delay(2000);
        digitalWrite(Fuel, LOW);
      }
    }
  }
  // Read analog value from the sensor
  sensorValue = analogRead(analogInPin);
  outputValue = map(sensorValue, 112, 921, 0, 100);
  if (outputValue >= 100) {
    outputValue = 100;
  }
  if (outputValue < 0) {
    outputValue = 0;
  }
  if (outputValue < 10) {
FUEL_LOW();
  }
  // Print the results to the Serial Monitor
  Serial.print(F("Sensor value: "));
  Serial.print(sensorValue);
  Serial.print(F("  Output value: "));
  Serial.println(outputValue);

  // Publish sensor data to Adafruit IO
  Serial.print(F("Sending Sensor value "));
  if (!sensorFeed.publish(outputValue)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  // Delay before the next loop iteration
  // delay(500);  // Adjust the delay as needed
}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {  // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 1 seconds...");
    mqtt.disconnect();
    delay(1000);  // wait 1 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1)
        ;
    }
  }
  Serial.println("MQTT Connected!");
}

