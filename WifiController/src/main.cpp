#include <Arduino.h>

#if defined(ESP8266)
#include <ESP8266WiFi.h>          
#else
#include <WiFi.h>          
#endif

//needed for library
#include <DNSServer.h>
#if defined(ESP8266)
#include <ESP8266WebServer.h>
#else
#include <WebServer.h>
#endif
#include <WiFiManager.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <math.h>
#include <ArduinoJson.h>

#define LED         2
#define INTERVAL    1000

#define SSF_GYRO    65.5
#define FREQ        250

#define PORTUDP     1667
#define PORTUDPSEND 1668
#define BUFFERSIZE  1024

struct Vector3 {
    float x;
    float y;
    float z;
};

struct MPUValue {
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    int16_t temp = 0;
    int16_t gyrX = 0;
    int16_t gyrY = 0;
    int16_t gyrZ = 0;
};

char data;
unsigned long int timer;
bool isConnected = false;
int statusSpeed = 0;

WiFiManager wifiManager;
IPAddress targetIP;
WiFiUDP udp;
WiFiUDP udpSend;
char buffer[BUFFERSIZE];

MPU6050 accelgyro;

Vector3 offset;
Vector3 angle;

// Define function
void calibrateMpu6050();
MPUValue getAngleBrute();
void getAngle();

void setup()
{
    Serial.begin(9600);
    pinMode(LED,OUTPUT);

    //WiFiManager
    statusSpeed = INTERVAL;
    wifiManager.autoConnect("AutoConnectAP");
    isConnected = true;
    statusSpeed = INTERVAL / 4;
    udp.begin(PORTUDP);
    udpSend.begin(PORTUDPSEND);

    //Balais init
    Wire.begin();
    accelgyro.initialize();
    delay(1000);

    //calibrateMpu6050();
}

void loop()
{
    if(millis()-timer>=statusSpeed) {
        timer=millis();
        digitalWrite(LED,!digitalRead(LED));
    } 

    if(Serial.available()) {
        data = Serial.read();
        if (data == '1') {
            WiFi.disconnect(false, true);
            delay(3000);
            Serial.println("Reset wifi setting and restart");
            wifiManager.resetSettings();
            ESP.restart();
        } else if (data == '2') {
            accelgyro.CalibrateGyro(30);
            accelgyro.CalibrateAccel(30);
        }
    }

    if (isConnected) {
        memset(buffer, 0, BUFFERSIZE);
        udp.parsePacket();
        if (udp.read(buffer, BUFFERSIZE) > 0) {
            Serial.println(buffer);
            if (strcmp(buffer, "123456") == 0) {
                targetIP = udp.remoteIP();
            }
        }

        if (targetIP != INADDR_NONE) {
            String jsonString = "";
            statusSpeed = INTERVAL / 8;

            getAngle();

            DynamicJsonDocument tracker(1024);
            tracker["x"] = angle.x;
            tracker["y"] = angle.y;
            tracker["z"] = angle.z;
            
            Serial.print("X: ");
            Serial.print(angle.x);
            Serial.print("\tY: ");
            Serial.print(angle.y);
            Serial.print("\tZ: ");
            Serial.println(angle.z);
            
            serializeJson(tracker, jsonString);

            udpSend.beginPacket(targetIP, PORTUDPSEND);
            
            const char* lineChar = jsonString.c_str();
            int i = 0;
            while (lineChar[i] != 0)
                udpSend.write((uint8_t)lineChar[i++]);
            udpSend.endPacket();
        }
    }
}

void calibrateMpu6050() {
    int max_samples = 2000;

    for (int i = 0; i < max_samples; i++) {
        MPUValue instant = getAngleBrute();

        offset.x += instant.accX;
        offset.y += instant.accY;
        offset.z += instant.accZ;
        
        delay(3);
    }
    offset.x /= max_samples;
    offset.y /= max_samples;
    offset.z /= max_samples;

    Serial.println("--- offset ---");
    Serial.print(offset.x);
    Serial.print("\t");
    Serial.print(offset.y);
    Serial.print("\t");
    Serial.print(offset.z);
    Serial.print("\n");
    Serial.println("--------------");
}

MPUValue getAngleBrute() {
    MPUValue instant;

    accelgyro.getMotion6(&instant.accX, &instant.accY, &instant.accZ, &instant.gyrX, &instant.gyrY, &instant.gyrZ);

    return instant;
}

void getAngle() {
    MPUValue item = getAngleBrute();
    angle.y = (0.98 * (angle.y + float(item.gyrY) * 0.01/50) + 0.02 * atan2((double)item.accX,(double)item.accZ) * 180 / PI); // Y
    angle.x = (0.98 * (angle.x + float(item.gyrX) * 0.01/50) + 0.02 * atan2((double)item.accY,(double)item.accZ) * 180 / PI); // X
    angle.z = (0.98 * (angle.z + float(item.gyrZ) * 0.01/50) + 0.02 * atan2((double)item.accX,(double)item.accY) * 180 / PI); // Z
}
/*
#include <Wire.h>  // Arduino Wire library
#include <I2Cdev.h>
#include <MPU6050.h>
#include <math.h>
 
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
 
int16_t ax, ay, az;  //mesures brutes
int16_t gx, gy, gz;
uint8_t Accel_range;
uint8_t Gyro_range;
float angleX=0;
float angleY=0;
float angleZ=0;

#define BUFFER_LENGTH 1024
 
void setup() {
    Wire.begin();  //I2C bus
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB (LEONARDO)
    }

    // initialize device
    Serial.println("Initialisation I2C...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Test de la conection du dispositif ...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection reussie" : "MPU6050 connection echec");
    delay(1000);
}
 
void loop() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    angleX=0.98*(angleX+float(gy)*0.01/131) + 0.02*atan2((double)ax,(double)az)*180/PI; // Y
    angleY=0.98*(angleY+float(gx)*0.01/131) + 0.02*atan2((double)ay,(double)az)*180/PI; // X
    angleZ=0.98*(angleZ+float(gz)*0.01/131) + 0.02*atan2((double)ax,(double)ay)*180/PI; // Z
    Serial.print("X: ");
    Serial.print(angleX);
    Serial.print("\tY: ");
    Serial.print(angleY);
    Serial.print("\tZ: ");
    Serial.println(angleZ);
    delay(10);
}
*/