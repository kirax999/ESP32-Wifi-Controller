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
#include <WiFiManager.h>        //https://github.com/tzapu/WiFiManager

#define LED       2
#define INTERVAL  1000

#define PORTUDP   1667
#define BUFFERSIZE 1024

char data;
unsigned long int timer;
bool isConnected = false;
int statusSpeed = 0;
WiFiManager wifiManager;

WiFiUDP udp;
char buffer[BUFFERSIZE];

IPAddress targetIP;

void configModeCallback (WiFiManager *myWiFiManager) {
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setup()
{
    Serial.begin(9600);
    pinMode(LED,OUTPUT);
    wifiManager.setDebugOutput(true);
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setTimeout(60);
    if(!wifiManager.autoConnect("ESP32 Controller")) {
        Serial.println("failed to connect and hit timeout");
        statusSpeed = INTERVAL;
    } else {
        Serial.println("Successfully connected");
        isConnected = true;
        statusSpeed = INTERVAL / 4;
        udp.begin(PORTUDP);
        Serial.println("Ready !!!");
    }
    
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());    
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
            if (strcmp(buffer, "123456") == 0) {
                targetIP = udp.remoteIP();
            }
        }
    }
}
