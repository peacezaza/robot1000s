/*
  Project:robot1000s
  Date:8/7/2024 -> Present
*/

//import Library
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

//End import Library

//Define Global Variable SECTION (preprocessor)
#define MQTT_SERVER "20.2.250.248"
#define MQTT_PORT 1883
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""
#define MQTT_NAME "ESP32_1"
#define MCP23017 0x20

#define trigPin 27
#define echoPin 13
#define trigPinRight 16
#define echoPinRight 17
#define SOUND_SPEED 0.034
#define ENCODER_PIN 25
#define ENCODER_PINBACK 4
#define DISTANCE_THRESHOLD 30


//End Define Vaiable SECTION

//Define MQTT BROKER
WiFiClient client;
PubSubClient mqtt(client);
//End Define MQTT BROKER

//global variable section
hw_timer_t* timer = NULL;
bool isToggle = true;
volatile unsigned long pulseCount = 0;
volatile unsigned long pulseCountBack = 0;
unsigned long lastTime = 0;
float rpm = 0;
float rpmBack = 0;
unsigned long timeInterval = 1000;
unsigned long currentTime;
float distanceCm;
float distanceCmRight;
const char* ssid = "Kanisorn";
const char* password = "Chokkkkk";
int currentTime2 = 0;
int lastSaveTime2 = 0;
int countRotate = 1;
String direction;
int distance;

String ValueTopic = "/ESP32/ultraSonicFront";
String ValueTopic2 = "/ESP32/ultraSonicRight";
String robotDirection = "/ESP32/direction";



//End global variable section
void IRAM_ATTR onTimer();
void IRAM_ATTR encoderISR();
void IRAM_ATTR encoderISRBack();

void setDirectionOutputPortA();
void setDirectionOutputPortB();
void connectWiFi();
void mqttConnect();
void ultraRead_Front();
void ultraRead_Right();
void front_motorForward();
void back_motorForward();
void front_motorStop();
void back_motorStop();
void front_motorTurnleft();
void back_motorTurnleft();
void publishMessage(String Topic, String message);
void callback(char* topic, byte* message, unsigned int length);
void getMotorSpeedFrontLeft();
void getMotorSpeedBackRight();
void onTimer();
void calculateDistanceBack();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!Serial);
  Serial.println("\nI2C Scanner");
  //I2C SETUP
  setDirectionOutputPortA();
  setDirectionOutputPortB();

  pinMode(trigPin, OUTPUT);       // Set TRIG pin as output
  pinMode(echoPin, INPUT);        // Set ECHO pin as input
  pinMode(trigPinRight, OUTPUT);  // Set TRIG pin as output
  pinMode(echoPinRight, INPUT);   // Set ECHO pin as input
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  pinMode(ENCODER_PINBACK, INPUT_PULLUP);


  // attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_PINBACK), encoderISRBack, RISING);

  // timer = timerBegin(10000);
  // timerAttachInterrupt(timer, &onTimer);
  // timerAlarm(timer,500, true, 0);

  // MQTT Setup Section
  connectWiFi();

  //End MQTT Setup Section

  // WiFi.mode(WIFI_STA);
  // WiFi.disconnect();  // Disconnect from any existing WiFi network

  // delay(100);
  // Serial.println("Scanning for available Wi-Fi networks...");

  // int numberOfNetworks = WiFi.scanNetworks();  // Perform the Wi-Fi scan

  // Print results of the scan
  // if (numberOfNetworks == 0) {
  //   Serial.println("No networks found");
  // } else {
  //   Serial.print("Found ");
  //   Serial.print(numberOfNetworks);
  //   Serial.println(" networks:");

  //   for (int i = 0; i < numberOfNetworks; i++) {
  //     // Print details of each network found
  //     Serial.print(i + 1);
  //     Serial.print(": ");
  //     Serial.print(WiFi.SSID(i));            // Print SSID (network name)
  //     Serial.print(" (RSSI: ");
  //     Serial.print(WiFi.RSSI(i));            // Print signal strength
  //     Serial.print(" dBm) ");
  //     Serial.print(" [Channel: ");
  //     Serial.print(WiFi.channel(i));         // Print channel number
  //     Serial.print("] ");

  //     // Check if it's an open or secured network
  //     Serial.print((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured");
  //     Serial.println();
  //   }
  // }
}


//
void loop() {
  mqttConnect();
  // currentTime = millis();
  // if (currentTime - lastTime >= timeInterval) {

  //   getMotorSpeedFrontLeft();
  //   getMotorSpeedBackRight();

  //   lastTime = currentTime;
  // }

  ultraRead_Front();

  // currentTime2 = millis();
  // if (currentTime2 - lastSaveTime2 >= 100) {
  //   lastSaveTime2 = currentTime2;
  //   // ultraRead_Front();
  //   Serial.println("Ultra Read");
  //   if (distanceCm > DISTANCE_THRESHOLD) {
  //     front_motorForward();
  //     back_motorForward();
  //   } else {
  //     front_motorStop();
  //     back_motorStop();
  //     delay(3000);
  //     front_motorTurnleft();
  //     back_motorTurnleft();
  //     delay(2000);
  //     countRotate++;
  //     if (countRotate % 4 == 1) {
  //       direction = "North";
  //     } else if (countRotate % 4 == 2) {
  //       direction = "West";
  //     } else if (countRotate % 4 == 3) {
  //       direction = "South";
  //     } else if (countRotate % 4 == 0) {
  //       direction = "East";
  //     }
  //     // publishMessage(robotDirection, direction);
  //     // publishMessage(robotDirection, direction);
  //   }

    if (distanceCm > DISTANCE_THRESHOLD) {
      front_motorForward();
      back_motorForward();
    }







  // ultraRead_Front();
  // publishMessage(ValueTopic, String(distanceCm));
  // ultraRead_Right();
  // ultraRead_Front();
  // publishMessage(ValueTopic, String(distanceCm));
  // front_motorForward();
  // back_motorForward();


  // if (distanceCm > DISTANCE_THRESHOLD) {
  //   front_motorForward();
  //   back_motorForward();
  // } else {
  //   front_motorStop();
  //   back_motorStop();
  //   delay(200);
  //   front_motorTurnleft();
  //   back_motorTurnleft();
  //   delay(2000);
  //   countRotate++;
  //   if (countRotate % 4 == 1) {
  //     direction = "North";
  //   } else if (countRotate % 4 == 2) {
  //     direction = "West";
  //   } else if (countRotate % 4 == 3) {
  //     direction = "South";
  //   } else if (countRotate % 4 == 0) {
  //     direction = "East";
  //   }
  //   publishMessage(robotDirection, direction);
  //   publishMessage(robotDirection, direction);
  // }

  }



void publishMessage(String Topic, String message) {
  if (mqtt.connected()) {
    mqtt.publish(Topic.c_str(), message.c_str());
    Serial.println("Status is Sent");
  } else {
    Serial.println("MQTT not connected. Cannot publish.");
  }
}

void mqttConnect() {
  if (mqtt.connected() == false) {
    Serial.print("MQTT connection... ");
    if (mqtt.connect(MQTT_NAME, MQTT_USERNAME,
                     MQTT_PASSWORD)) {
      Serial.println("connected");
      mqtt.subscribe("/ESP32_1/STATUS");
    } else {
      Serial.println("failed");
      delay(5000);
    }
  } else {
    mqtt.loop();
  }
}

//callback function for mqttbroker
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  if (String(topic) == "/ESP32_1/STATUS") {
    Serial.print("Changing output to ");
    if (messageTemp == "toggle") {
      Serial.println("Motor is Toggle!");
      isToggle = true;
    } else {
      Serial.println("Motor is Stop!");
      isToggle = false;
    }
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);  //Optional
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);
}

void IRAM_ATTR encoderISR() {
  pulseCount++;
}
void IRAM_ATTR encoderISRBack() {
  pulseCountBack++;
}

void setDirectionOutputPortA() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
}
void setDirectionOutputPortB() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.endTransmission();
}

void front_motorStop() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x12);
  Wire.write(0x00);
  Wire.endTransmission();
}
void back_motorStop() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x13);
  Wire.write(0x00);
  Wire.endTransmission();
}

void front_motorForward() {
  Serial.println("TOGGG");
  Wire.beginTransmission(MCP23017);
  Wire.write(0x12);
  Wire.write(0x0A);  //reverse bit
  Wire.endTransmission();
}
void back_motorForward() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x13);
  Wire.write(0x0A);
  Wire.endTransmission();
}

void front_motorbackForward() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x12);
  Wire.write(0x05);  //reverse bit
  Wire.endTransmission();
}
void back_motorbackForward() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x13);
  Wire.write(0x05);
  Wire.endTransmission();
}

void front_motorTurnleft() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x12);
  Wire.write(0x09);
  Wire.endTransmission();
}

void back_motorTurnleft() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x13);
  Wire.write(0x06);
  Wire.endTransmission();
}

void front_motorTurnRight() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x13);
  Wire.write(0x09);
  Wire.endTransmission();
}

void back_motorTurnRight() {
  Wire.beginTransmission(MCP23017);
  Wire.write(0x12);
  Wire.write(0x06);
  Wire.endTransmission();
}

float distanceCM_cal(long duration) {
  return duration * SOUND_SPEED / 2;
}
void ultraRead_Front() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  distanceCm = distanceCM_cal(duration);


  // Prints the distance in the Serial Monitor
  Serial.print("Distance Front (cm): ");
  Serial.println(distanceCm);
  delay(100);
}

void ultraRead_Right() {
  // Clears the trigPin
  digitalWrite(trigPinRight, LOW);
  delayMicroseconds(10);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long durationRight = pulseIn(echoPinRight, HIGH);

  // Calculate the distance
  distanceCmRight = distanceCM_cal(durationRight);


  // Prints the distance in the Serial Monitor
  Serial.print("Distance Right (cm): ");
  Serial.println(distanceCmRight);
  delay(100);
}

void getMotorSpeedFrontLeft() {
  float revolutions = pulseCount / 20.0;
  rpm = (revolutions / (timeInterval / 1000.0)) * 60.0;  // Calculate RPM
  pulseCount = 0;

  Serial.print("Motor Speed: ");
  Serial.print(rpm);
  Serial.println(" RPM");
}

void getMotorSpeedBackRight() {
  float revolutionsBack = pulseCountBack / 20.0;
  rpmBack = (revolutionsBack / (timeInterval / 1000.0)) * 60.0;  // Calculate RPM

  // Reset pulse count
  pulseCountBack = 0;

  Serial.print("Back Motor Speed: ");
  Serial.print(rpmBack);
  Serial.println(" RPM");
}


void onTimer() {
  // Toggle LED2 state
  ultraRead_Right();
  ultraRead_Front();
  publishMessage(ValueTopic, String(distanceCm));
  publishMessage(ValueTopic2, String(distanceCmRight));
}

void calculateDistanceBack() {
  // float wheel_curcumference = 18.84;
  // distance = rpm * wheel_circumference;
  // printf(distance);
}
