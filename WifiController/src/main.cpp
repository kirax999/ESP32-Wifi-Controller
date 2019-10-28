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

char data;
unsigned long int timer;
bool isConnected = false;

int statusSpeed = 0;

WiFiManager wifiManager;

void configModeCallback (WiFiManager *myWiFiManager)
{
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setup()
{
    Serial.begin(9600);
    pinMode(LED,OUTPUT);
    wifiManager.setDebugOutput(false);
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setTimeout(60);
    if(!wifiManager.autoConnect("Controller")) {
        Serial.println("failed to connect and hit timeout");
        statusSpeed = INTERVAL;
    } else {
        Serial.println("Successfully connected");
        isConnected = true;
        statusSpeed = INTERVAL / 4;
    }
    
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());    
}

void loop()
{
    if(millis()-timer>=INTERVAL) {
        timer=millis();
        digitalWrite(LED,!digitalRead(LED));
        if(digitalRead(LED)==LOW)
            Serial.println("LED OFF");
        else
            Serial.println("LED ON");
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

    }
}
