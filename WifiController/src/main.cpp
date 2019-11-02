#include <Arduino.h>
/*
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

#include <MPU6050.h>
#include <I2Cdev.h>
#include <math.h>

#include <ArduinoJson.h>

#define LED       2
#define INTERVAL  1000

#define SSF_GYRO    65.5
#define FREQ        250

#define PORTUDP   1667
#define PORTUDPSEND   1668
#define BUFFERSIZE 1024

struct Vector3 {
    double x;
    double y;
    double z;
};

struct MPUValue {
    int accX = 0;
    int accY = 0;
    int accZ = 0;
    int temp = 0;
    int gyrX = 0;
    int gyrY = 0;
    int gyrZ = 0;
};

char data;
unsigned long int timer;
bool isConnected = false;
int statusSpeed = 0;
int period = 0;

WiFiManager wifiManager;
IPAddress targetIP;
WiFiUDP udp;
WiFiUDP udpSend;
char buffer[BUFFERSIZE];

Vector3 offset;
Vector3 result;

float angle = 0;

// Define function
Vector3 getAngles();
MPUValue getAngleBrute();
MPUValue readSensor();
void calibrateMpu6050();
void setupMpu6050();

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

    setupMpu6050();
    calibrateMpu6050();
    period = (1000000/FREQ);
}

void loop()
{
    if(millis()-timer>=statusSpeed) {
        timer=millis();
        digitalWrite(LED,!digitalRead(LED));
    } 

    if(Serial.available()) {
        data=Serial.read();
        if(data=='1')
        {
            WiFi.disconnect(false, true);
            delay(3000);
            Serial.println("Reset wifi setting and restart");
            wifiManager.resetSettings();
            ESP.restart();
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

            DynamicJsonDocument tracker(1024);
            tracker["x"] = result.x;
            tracker["y"] = result.y;
            tracker["z"] = result.z;

            serializeJson(tracker, jsonString);

            udpSend.beginPacket(targetIP, PORTUDPSEND);
            const char* lineChar = jsonString.c_str();
            int i = 0;
            while (lineChar[i] != 0)
                udpSend.write((uint8_t)lineChar[i++]);
            udpSend.endPacket();
        }
    }

    getAngles();
}

MPUValue getAngleBrute() {
    MPUValue instant;

    Wire.beginTransmission(0x68);// Start communicating with the MPU-6050
    Wire.write(0x3B);                   // Send the requested starting register
    Wire.endTransmission();             // End the transmission
    Wire.requestFrom(0x68,14);   // Request 14 bytes from the MPU-6050

    while(Wire.available() < 14);

    instant.accX = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[X] variable
    instant.accY = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Y] variable
    instant.accZ = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Z] variable
    instant.temp = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
    instant.gyrX = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[X] variable
    instant.gyrY = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Y] variable
    instant.gyrZ = Wire.read() << 8 | Wire.read();

    return instant;
}

Vector3 getAngles() { // return Struc Gyroscope containe 3axis
    MPUValue tmp = readSensor();

    result.x = (tmp.gyrX - offset.x) / (250 * SSF_GYRO);
    result.y = (tmp.gyrY - offset.y) / (250 * SSF_GYRO);
    result.z = (tmp.gyrZ - offset.z) / (250 * SSF_GYRO);

    Serial.print("x:");
    Serial.print(result.x);
    Serial.print("\t y:");
    Serial.print(result.y);
    Serial.print("\t z:");
    Serial.println(result.z);
    return result;
}

void setupMpu6050() {
    Wire.begin();

    // Configure power management
    Wire.beginTransmission(0x68); // Start communication with MPU
    Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
    Wire.write(0x00);                    // Apply the desired configuration to the register
    Wire.endTransmission(true);

    // Configure the gyro's sensitivity
    Wire.beginTransmission(0x68); // Start communication with MPU
    Wire.write(0x1B);                    // Request the GYRO_CONFIG register
    Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
    Wire.endTransmission(true);

    // Configure the acceleromter's sensitivity
    Wire.beginTransmission(0x68); // Start communication with MPU
    Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
    Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
    Wire.endTransmission(true);

    // Configure low pass filter
    Wire.beginTransmission(0x68); // Start communication with MPU
    Wire.write(0x1A);                    // Request the CONFIG register
    Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
    Wire.endTransmission(true);
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

/*

#include<Wire.h>

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265;
int maxVal=402;

double x;
double y;
double z;

void setup(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);
}
void loop(){
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);

x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
}
*/