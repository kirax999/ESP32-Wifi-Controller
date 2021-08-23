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
#include <ArduinoJson.h>

#define INTERVAL    1000

#define LEDRED    2
#define LEDYELLO  4
#define LEDGREEN  5

#define SSF_GYRO    65.5
#define FREQ        250

#define PORTUDP     1667
#define PORTUDPSEND 1668
#define BUFFERSIZE  1024

#define PINACCELERATOR 32
#define PINBRACE 33
#define PINPITCH 34
#define PINROLL 35

struct Data {
    float x;
    float y;
    float a;
    float b;
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

Data angle;

void setup()
{
    digitalWrite(LEDRED, true);
    Serial.begin(9600);
    pinMode(LEDRED,OUTPUT);
    pinMode(LEDYELLO,OUTPUT);
    pinMode(LEDGREEN,OUTPUT);

    //WiFiManager
    statusSpeed = INTERVAL;
    wifiManager.autoConnect("quidditch controller");
    isConnected = true;
    digitalWrite(LEDYELLO, true);
    udp.begin(PORTUDP);
    udpSend.begin(PORTUDPSEND);
}

void loop()
{
    if(millis()-timer>=statusSpeed) {
        timer=millis();
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
            digitalWrite(LEDGREEN, true);

            angle.x = analogRead(PINROLL);
            angle.y = analogRead(PINPITCH);
            angle.a = analogRead(PINACCELERATOR);
            angle.b = analogRead(PINBRACE);

            DynamicJsonDocument tracker(1024);
            tracker["x"] = angle.x;
            tracker["y"] = angle.y;
            tracker["a"] = angle.a;
            tracker["b"] = angle.b;
            /*
            Serial.print("X: ");
            Serial.print(angle.x);
            Serial.print("\tY: ");
            Serial.print(angle.y);
            Serial.print("\tZ: ");
            Serial.println(angle.z);
            */

            Serial.print("X: ");
            Serial.print(angle.x);
            Serial.print("\tY: ");
            Serial.print(angle.y);
            Serial.print("\tA: ");
            Serial.print(angle.a);
            Serial.print("\tB: ");
            Serial.println(angle.b);

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