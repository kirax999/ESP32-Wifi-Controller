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

WiFiManager wifiManager;
IPAddress targetIP;
WiFiUDP udp;
WiFiUDP udpSend;
char buffer[BUFFERSIZE];

Vector3 offset;
Vector3 result;

int minVal=265;
int maxVal=402;

float angle = 0;

// Define function
Vector3 getAngles();
MPUValue getAngleBrute();
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
    MPUValue tmp = getAngleBrute();
    double AcX = (tmp.gyrX - offset.x);
    double AcY = (tmp.gyrY - offset.y);
    double AcZ = (tmp.gyrZ - offset.z);

    int x = map(AcX,minVal,maxVal,-90,90);
    int y = map(AcY,minVal,maxVal,-90,90);
    int z = map(AcZ,minVal,maxVal,-90,90);

    result.x = RAD_TO_DEG * (atan2(-y, -z)+PI);
    result.y = RAD_TO_DEG * (atan2(-x, -z)+PI);
    result.z = RAD_TO_DEG * (atan2(-y, -x)+PI);

    Serial.print("x:");
    Serial.print(result.x);
    Serial.print("\t y:");
    Serial.print(result.y);
    Serial.print("\t z:");
    Serial.println(result.z);
    Serial.println("------------------------------");
    return result;
}

void setupMpu6050() {
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
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
*/

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
float angle=0;

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
    //angle=0.98*(angle+float(gy)*0.01/131) + 0.02*atan2((double)ax,(double)az)*180/PI; // Y
    //angle=0.98*(angle+float(gx)*0.01/131) + 0.02*atan2((double)ay,(double)az)*180/PI; // X
    angle=0.98*(angle+float(gz)*0.01/131) + 0.02*atan2((double)ax,(double)ay)*180/PI; // Z
    Serial.println(angle); 
    delay(10);
}