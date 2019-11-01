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
#include <ArduinoJson.h>

#define LED       2
#define INTERVAL  1000

#define PORTUDP   1667
#define PORTUDPSEND   1668
#define BUFFERSIZE 1024

char data;
unsigned long int timer;
bool isConnected = false;
int statusSpeed = 0;
WiFiManager wifiManager;

IPAddress targetIP;
WiFiUDP udp;
WiFiUDP udpSend;
char buffer[BUFFERSIZE];

const int MPU_addr=0x68;

struct gyroscope {
    double x;
    double y;
    double z;
};

gyroscope getAngles();

void setup()
{
// put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(LED,OUTPUT);

    //WiFiManager
    statusSpeed = INTERVAL;
    wifiManager.autoConnect("AutoConnectAP");
    isConnected = true;
    statusSpeed = INTERVAL / 4;
    udp.begin(PORTUDP);
    udpSend.begin(PORTUDPSEND);

    // Init MPU-6050
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
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
            gyroscope value;
            value = getAngles();

            DynamicJsonDocument tracker(1024);
            tracker["x"] = value.x;
            tracker["y"] = value.y;
            tracker["z"] = value.z;

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

gyroscope getAngles() { // return Struc Gyroscope containe 3axis
    gyroscope result;
    
    int16_t AcX,AcY,AcZ;
    int minVal=265;
    int maxVal=402;
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

    result.x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    result.y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    result.z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

    Serial.print("x:");
    Serial.print(result.x);
    Serial.print("\t y:");
    Serial.print(result.y);
    Serial.print("\t z:");
    Serial.println(result.z);

    return result;
}