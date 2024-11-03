/*
  Project:robot1000s
  Date:8/7/2024 -> Present
*/



//import Library
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

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
#define distanceCM_cal(x) ((x * SOUND_SPEED) / 2);
WiFiClient client;
PubSubClient mqtt(client);
//End Define MQTT BROKER

//global variable section
bool isToggle = true;
bool messageSent = false;
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
unsigned long timeUse = 0;
bool toggle = false;


String robotDirection = "/ESP32/direction";
String toogleRobot = "/ESP32/toggle";
String statusRobot ="/ESP32/status";


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
void getMotorSpeedFrontLeft();
void getMotorSpeedBackRight();


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

  // MQTT Setup Section
  connectWiFi();
  isToggle = false;

}

//
void loop() {
  mqttConnect();




  if(isToggle){
    if(isToggle != toggle){
      lastSaveTime2 = millis();
      timeUse = millis();
      toggle = true;
    }
    if (millis() - lastSaveTime2 >= 500) {
      ultraRead_Front();
      if (distanceCm > DISTANCE_THRESHOLD) {
        front_motorForward();
        back_motorForward();
        messageSent = false;
      } else {
        Serial.println("STOP!");
        countRotate++;
        if (countRotate % 4 == 1) {
          direction = "North";
        } else if (countRotate % 4 == 2) {
          direction = "West";
        } else if (countRotate % 4 == 3) {
          direction = "South";
        } else if (countRotate % 4 == 0) {
          direction = "East";
        }
        front_motorStop();
        back_motorStop();
        // mqttConnect();

        if(!messageSent && mqtt.connected()){
          DynamicJsonDocument doc(1024);
          doc["direction"] = direction;
          doc["time"] = (millis() - timeUse)/1000;
          // mqtt.publish(robotDirection.c_str(), direction.c_str());
          // mqtt.publish("/ESP32/time", String(.c_str());
          char buffer[256];
          serializeJson(doc,buffer);
          mqtt.publish(robotDirection.c_str(), buffer);
          messageSent = true;
          // publishMessage(robotDirection, direction);
        }

        front_motorTurnleft();
        back_motorTurnleft();
        delay(1700);
        timeUse = millis();
      }
      lastSaveTime2 = millis();
      Serial.print("Direction : ");
      Serial.println(direction);

    }

  }else{
    toggle = false;
  }


}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String topic_str = topic, payload_str = (char*)payload;
  Serial.print("Payload is :");
  Serial.println("[" + topic_str + "]: " + payload_str);
  mqttConnect();
  if(payload_str == "0"){
    isToggle = false;
    publishMessage(statusRobot, "Stopped");
    front_motorStop();
    back_motorStop();
  }
  else if(payload_str == "1"){
    isToggle =true;
      publishMessage(statusRobot, "Running");
  }
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
      mqtt.subscribe("/ESP32/toggle");
    } else {
      Serial.println("failed");
      delay(5000);
    }
  } else {
    mqtt.loop();
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

void ultraRead_Front() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  
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








