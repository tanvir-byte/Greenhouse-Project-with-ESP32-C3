#include <Arduino.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

#define DHTPIN 19      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11
#define SOIL_MOISTURE_PIN 2 // Analog pin connected to the soil moisture sensor
#define TRIGGER_PIN 18 // Pin connected to HC-SR04 trigger
#define ECHO_PIN 12    // Pin connected to HC-SR04 echo
#define RGB_LED_PIN_R 4 // Pin connected to RGB LED - Red
#define RGB_LED_PIN_G 8 // Pin connected to RGB LED - Green
#define RGB_LED_PIN_B 9 // Pin connected to RGB LED - Blue

const char *ssid = "ESP";
const char *password = "bubez123";

const int ledPin2 = 6;
const int ledPin1 = 10; // New LED pin
const int photoresistorPin = 0;
const int WATER_LEVEL_PIN = 1;
const int relayPin1 = 13; // Relay pin for light intensity
const int relayPin2 = 11; // Relay pin for water level
const int servoPin = 3; // Pin connected to the servo motor

int lightIntensity = 0;
int waterLevel = 0;
int soilMoisture = 0;
float temperature = 0.0;
float humidity = 0.0;
int distance = 0;
Servo myservo;
String ledState2;
String ledState1; // State of ledPin


AsyncWebServer server(80);

String processor(const String &var) {
  Serial.println(var);
  if (var == "LIGHT_INTENSITY") {
    return String(lightIntensity);
  } else if (var == "WATER_LEVEL") {
    return String(waterLevel);
  } else if (var == "TEMPERATURE") {
    return String(temperature);
  } else if (var == "HUMIDITY") {
    return String(humidity);
  } else if (var == "SOIL_MOISTURE") {
    return String(soilMoisture);
  } else if (var == "DISTANCE") {
    return String(distance);
 } else if (var == "SERVO_ANGLE") {
    return String(myservo.read());
  } else if (var == "STATE2") {
    if (digitalRead(ledPin2)) {
      ledState2 = "ON";
    } else {
      ledState2 = "OFF";
    }
    Serial.println(ledState2);
    return ledState2;
  } else if (var == "STATE1") { // New block to handle ledPin2
    if (digitalRead(ledPin1)) {
      ledState1 = "ON";
    } else {
      ledState1 = "OFF";
    }
    Serial.println(ledState1);
    return ledState1;
  }
  return String();
}
DHT dht(DHTPIN, DHTTYPE);
Servo servo;
int angle;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(photoresistorPin, INPUT);
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(RGB_LED_PIN_R, OUTPUT);
  pinMode(RGB_LED_PIN_G, OUTPUT);
  pinMode(RGB_LED_PIN_B, OUTPUT);
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(servoPin);
 int redValue = map(temperature, 0, 50, 0, 100); // Map temperature to red color intensity
  int greenValue = map(humidity, 0, 100, 0, 100); // Map humidity to green color intensity
  int blueValue = map(soilMoisture, 0, 4095, 100, 0); // Map soil moisture to blue color intensity
  int red= max(redValue, max(greenValue, blueValue));
  int green= min(redValue, min(greenValue, blueValue));
  int blue = (red+green)/2;
  String blu0 = String(blue);
blu0 = String(redValue) + "," + String(greenValue) + "," + String(blueValue);
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/style.css", "text/css");
  });

  server.on("/on2", HTTP_GET, [](AsyncWebServerRequest *request) {
    digitalWrite(ledPin2, HIGH);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/off2", HTTP_GET, [](AsyncWebServerRequest *request) {
    digitalWrite(ledPin2, LOW);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
    server.on("/on1", HTTP_GET, [](AsyncWebServerRequest *request) {
    digitalWrite(ledPin1, HIGH);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/off1", HTTP_GET, [](AsyncWebServerRequest *request) {
    digitalWrite(ledPin1, LOW);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route to retrieve light intensity
  server.on("/light_intensity", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", String(lightIntensity).c_str());
  });

  // Route to retrieve water level
  server.on("/water_level", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", String(waterLevel).c_str());
  });

  // Route to retrieve temperature
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", String(temperature).c_str());
  });

  // Route to retrieve humidity
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", String(humidity).c_str());
  });

  // Route to retrieve soil moisture
  server.on("/soil_moisture", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", String(soilMoisture).c_str());
  });

  // Route to retrieve ultrasonic distance
  server.on("/distance", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", String(distance).c_str());
  });

  // Route to retrieve servo angle
  server.on("/servo_angle", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", String(servo.read()).c_str());
  });

   // Update web server with RGB LED values
server.on("/rgb_led", HTTP_GET, [blue, blu0](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", String(blue).c_str());
   request->send_P(200, "text/plain", String(blu0).c_str());
});



  server.begin();

  // Initialize DHT sensor
  Serial.println("DHT Initializing...");
  dht.begin();
}

void loop() {

  int sensorValue = analogRead(photoresistorPin);
  lightIntensity = map(sensorValue, 0, 4095, 0, 100);
  delay(100); // wait 10 milliseconds
  waterLevel = analogRead(WATER_LEVEL_PIN); // Read the analog value from sensor
  delay(100); // wait 10 milliseconds
  soilMoisture = analogRead(SOIL_MOISTURE_PIN); // Read the soil moisture sensor value
  delay(100); // wait 10 milliseconds

  // Read ultrasonic distance
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  distance = pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;

 // Read DHT sensor
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
  }
  else
  {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");
  }

  // Control servo based on ultrasonic distance
if (distance >= 0 && distance <= 30)
  {
    // If distance is between 0 and 30 cm, rotate servo to 90 degrees
    servo.write(90);
  }
  else
  {
    // Otherwise, keep servo at 0 degrees
    servo.write(0);
  }
  // Update web server with current servo angle
  server.on("/servo_angle", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", String(servo.read()).c_str());
  });

  // Control relay based on light intensity
  if (lightIntensity <= 50 || digitalRead(ledPin1) == HIGH)
  {
    digitalWrite(relayPin1, HIGH); // Turn on relay
  }
  else if (lightIntensity > 50 || digitalRead(ledPin1) == LOW)
  {
    digitalWrite(relayPin1, LOW); // Turn off relay
  }

  // Control relay based on water level
  if (waterLevel <= 30 || digitalRead(ledPin2) == HIGH)
  {
    digitalWrite(relayPin2, HIGH); // Turn on relay
  }
  else if (waterLevel >30 || digitalRead(ledPin2) == LOW)
  {
    digitalWrite(relayPin2, LOW); // Turn off pump relay
  }
  
  // Change RGB LED color based on sensor values
  int redValue = map(temperature, 0, 50, 0,100); // Map temperature to red color intensity
  int greenValue = map(humidity, 0, 100, 0, 100); // Map humidity to green color intensity
  int blueValue = map(soilMoisture, 0, 4095, 100, 0); // Map soil moisture to blue color intensity
  int red= max(redValue, max(greenValue, blueValue));
  int green= min(redValue, min(greenValue, blueValue));
 
  int blue = (red+green)/2;
   if(red <30 || green<30) {
  digitalWrite(RGB_LED_PIN_R, 1);   // Set red LED intensity
  digitalWrite(RGB_LED_PIN_G, 0); // Set green LED intensity
  digitalWrite(RGB_LED_PIN_B, 0);  // Set blue LED intensity
  }
 else if ( green>80 && red> 30 ) {
  digitalWrite(RGB_LED_PIN_R, 0);   // Set red LED intensity
  digitalWrite(RGB_LED_PIN_G, 0); // Set green LED intensity
  digitalWrite(RGB_LED_PIN_B, 1);  // Set blue LED intensity
 }
 else if (green<80 && red > 30 ) {
  digitalWrite(RGB_LED_PIN_R, 0);   // Set red LED intensity
  digitalWrite(RGB_LED_PIN_G, 1); // Set green LED intensity
  digitalWrite(RGB_LED_PIN_B, 0);  // Set blue LED intensity
 }
  delay(2000);
}

