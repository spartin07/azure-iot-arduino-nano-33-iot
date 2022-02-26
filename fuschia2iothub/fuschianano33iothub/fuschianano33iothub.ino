#include "arduino_secrets.h"

// ArduinoJson - Version: Latest
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <Arduino_LSM6DS3.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <utility/ECCX08SelfSignedCert.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>

const char broker[]      = SECRET_BROKER;
String     deviceId  =  SECRET_DEVICE_ID;  // Device ID if ECCX08 certificate

#define ECCX08_CERTIFICATE

WiFiClient    wifiClient;            // Used for the TCP socket connection
BearSSLClient sslClient(wifiClient); // Used for SSL/TLS connection, integrates with ECC508
MqttClient    mqttClient(sslClient);

const char ssid[]        = SECRET_WIFI_SSID;
const char pass[]        = SECRET_WIFI_PASS;

float accelX, accelY, accelZ, // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX, gyroY, gyroZ, // units dps (degrees per second)
      gyroDriftX, gyroDriftY, gyroDriftZ, // units dps
      gyroRoll, gyroPitch, gyroYaw, // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw, // units degrees (expect minor drift)
      accRoll, accPitch, accYaw, // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw,
      lastFrequency,
      calibrateRoll, previousRoll, calibrateRollMax;// units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;
unsigned long currentTime;
char data[350];
int counter = 0;


//=========================================================================================


// get the current time from the WiFi module
unsigned long getTime() {
  return WiFi.getTime();
}

void connectWiFi() {
  Serial.print("Attempting to connect to SSID: ");
  Serial.print(ssid);
  Serial.print(" ");

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the network");
  Serial.println();
}

/*
   Establishses connection with the MQTT Broker (IoT Hub)
   Some errors you may receive:
   -- (-.2) Either a connectivity error or an error in the url of the broker
   -- (-.5) Check credentials - has the SAS Token expired? Do you have the right connection string copied into arduino_secrets?
*/
void connectMQTT() {
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");

  while (!mqttClient.connect(broker, 8883)) {
    // failed, retry
    Serial.print(".");
    Serial.println(mqttClient.connectError());
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe("devices/" + deviceId + "/messages/devicebound/#");
}


void publishMessage() {
  Serial.println("Publishing message");
  const int capacity = JSON_ARRAY_SIZE(10) + 10 * JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + 280;  // Calculation of the JSON doc size, as explained in the documentation

  StaticJsonDocument<capacity> doc;
  int userId = 10;
  String date = "2022-02-26";
  char exercise[20];
  sprintf(exercise, "%f", getTime());
  Serial.println(data);
  //  String exercise = "Snow Angel"
  //  sprintf(data, "%f", calibrateRoll);
  doc["userId"] = counter;
  doc["workoutDate"] = date;
  doc["exercise"] = exercise;
  doc["data"] = data;


  //   DEBUG - serialize the document in the serial monitor
  //   serializeJson(doc, Serial);
  //   Serial.println(" ");

  char payload[1024]; // length of the char buffer that contains the JSON file, concretely the number of characters included in one message
  size_t payloadSize = serializeJson(doc, payload);

  //   DEBUG - write the size of the serialized document
  Serial.print("json size:");
  Serial.println(payloadSize);

  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/", static_cast<unsigned long>(payloadSize));
  mqttClient.print(payload);
  mqttClient.endMessage();
  /*

    To replicate the issue, uncomment the following 3 lines and comment the 3 above. This way you'll only be able to send MQTT messages smaller than 256 Bytes.
  */
  //  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
  //  serializeJson(doc, mqttClient);
  //  mqttClient.endMessage();

}


//===================================================================================
//Accel data functions
/*
  the gyro's x,y,z values drift by a steady amount. if we measure this when arduino is still
  we can correct the drift when doing real measurements later
*/
void calibrateIMU(int delayMillis, int calibrationMillis) {
  int calibrationCount = 0;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;
      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;

}

/**
  Read accel and gyro data.
  returns true if value is 'new' and false if IMU is returning old cached data
*/
bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    return true;
  }
  return false;
}
/**
  I'm expecting, over time, the Arduino_LSM6DS3.h will add functions to do most of this,
  but as of 1.0.0 this was missing.
*/
void doCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;



  lastFrequency = 1000000.0 / lastInterval;


  //  Serial.print(" LF: ");
  //  Serial.print(lastFrequency);
  //  Serial.print(" LI: ");
  //  Serial.print(lastInterval);
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);



  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;


  calibrateRoll = complementaryRoll + 90;
  Serial.print(calibrateRoll);
  Serial.println("");
  char cali[6];
  sprintf(cali, "%.2f", calibrateRoll);//make the number into string using sprintf function
  Serial.print(" Cali: ");
  Serial.println(cali);
  strcat(data, cali);
  strcat(data, " ");
  counter++;
}

//=========================================================================================


void setup() {

  Serial.begin(9600);

  pinMode(10, OUTPUT);
  if (!ECCX08.begin()) {
    Serial.println("No ECCX08 present!"); // If no ECCX08 certificate is present, generate one using the "ECCX08SelfSignedCert.ino" sketch from the library examples
    while (1);
  }

  // reconstruct the self signed cert
  ECCX08SelfSignedCert.beginReconstruction(0, 8);
  ECCX08SelfSignedCert.setCommonName(ECCX08.serialNumber());
  ECCX08SelfSignedCert.endReconstruction();

  // Set the ECCX08 slot to use for the private key
  // and the accompanying public certificate for it
  sslClient.setEccSlot(0, ECCX08SelfSignedCert.bytes(), ECCX08SelfSignedCert.length());

  // Set a callback to get the current time
  // used to validate the servers certificate

  ArduinoBearSSL.onGetTime(getTime);

  // Set the username to "<broker>/<device id>/?api-version=2018-06-30"
  String username;

  // Set the client id used for MQTT as the device id
  mqttClient.setId(deviceId);

  username += broker;
  username += "/";
  username += deviceId;
  username += "/api-version=2018-06-30";
  mqttClient.setUsernamePassword(username, "");


  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  calibrateIMU(250, 250);
  lastTime = micros();
}

//=========================================================================================

void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    connectMQTT();
  }

  // poll for new MQTT messages and send keep alives
  if (readIMU()) {
    Serial.print("Reading IMU");
    currentTime = micros();
    lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
    lastTime = currentTime;



    doCalculations();
  }
  mqttClient.poll();
  Serial.println("Polling, Counter: " + counter);
  if (counter == 50) {
    counter == 0;
    publishMessage();
    memset(data,'\0',350);
  }
  delay(100); //  publish a message roughly every 10 seconds
}
