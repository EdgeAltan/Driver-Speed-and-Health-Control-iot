#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30100_PulseOximeter.h"
#include <TinyGPS++.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Servo.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// Pin Definitions
#define SDA_PIN 4
#define SCL_PIN 5
#define GPS_TX_PIN 8
#define GPS_RX_PIN 9
#define SERVO_PIN 7
#define LED_PIN LED_BUILTIN

// OLED Setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX30100 Resources
SemaphoreHandle_t xI2CMutex;  // Dedicated I2C mutex
volatile float sharedBpm = 0;
volatile float sharedSpo2 = 0;
float bpm = 0;
float spo2 = 0;
PulseOximeter pox;

TaskHandle_t max30100TaskHandle = NULL;
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t servoTaskHandle  = NULL;

// GPS
#define GPS_UART Serial2
bool lastGpsStatus = false;
float max_speed = 0;
float speedy = 0;
TinyGPSPlus gps;

// Display Management
uint8_t screenIndex = 0;
uint32_t lastScreenChange = 0;
#define SCREEN_CYCLE_TIME 5000

// Servo Control
Servo myServo;
int currentAngle = 0;
int lastServoUpdate = 0;
int initial_speed_res = 1;
#define SERVO_UPDATE_INTERVAL 500

// Timing Constants
#define REPORTING_PERIOD_MS 500
uint32_t lastReport = 0;
uint32_t lastSensorCheck = 0;
const uint32_t SENSOR_CHECK_INTERVAL = 5000;

// WiFi Credentials
const char* ssid = "xxx";
const char* pass = "xxx";
const char* BLYNK_TOKEN = "xxx";

void onBeatDetected() {
    if(xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100))) {
        Serial.println("♥ Beat!");
        display.fillRect(120, 0, 8, 8, WHITE);
        display.display();
        xSemaphoreGive(xI2CMutex);
    }
    delay(25);
}

void initializeI2C() {
    Wire.end();
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);  // Set to 400kHz I2C speed
    delay(50);
}

void recoverI2C() {
    Serial.println("Attempting I2C recovery...");
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    
    // Reset I2C bus
    initializeI2C();
    
    // Reinitialize devices
    bool oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    bool sensorOK = pox.begin();
    
    if(!oledOK || !sensorOK) {
        Serial.println("Recovery Failed!");
        for(int i=0; i<5; i++) {
            digitalWrite(LED_PIN, HIGH); delay(200);
            digitalWrite(LED_PIN, LOW); delay(200);
        }
    }
    else {
        pox.setIRLedCurrent(MAX30100_LED_CURR_27_1MA);
        pox.setOnBeatDetectedCallback(onBeatDetected);
        Serial.println("Recovery Successful!");
    }
    
    xSemaphoreGive(xI2CMutex);
}

void initializeOLED() {
    if(xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(1000))) {
        if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            Serial.println("OLED Init Failed!");
            while(1) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0,0);
        display.println("System Booting...");
        display.display();
        xSemaphoreGive(xI2CMutex);
    }
}

void checkAndSendGpsStatus() {
    bool currentStatus = gps.location.isValid();

    if (currentStatus != lastGpsStatus) {
        // GPS status changed, send update
        HTTPClient http;
        String value = "";

        if(currentStatus == false){
          value = "0";
        }
        else{
          value = "1";
        }

        String url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                     "&V7=" + value;

        http.begin(url);
        int httpCode = http.GET();
        Serial.print("V7 HTTP Response (GPS Status): ");
        Serial.println(httpCode);

        if (httpCode == HTTP_CODE_OK) {
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(LED_PIN, LOW);
            delay(100);
            digitalWrite(LED_PIN, HIGH);
        }
        http.end();
        delay(500);

        lastGpsStatus = currentStatus;  // Update stored status
    }
}

void MAX30100_Task(void *pvParameters) {
    initializeI2C();
    
    if(xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(1000))) {
        if(!pox.begin()) {
            Serial.println("MAX30100 Init Failed!");
            vTaskDelete(NULL);
        }
        pox.setIRLedCurrent(MAX30100_LED_CURR_17_4MA);
        pox.setOnBeatDetectedCallback(onBeatDetected);
        xSemaphoreGive(xI2CMutex);
    }

    for(;;) {
        if(xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10))) {
            pox.update();
            xSemaphoreGive(xI2CMutex);
        }

        // Update shared data
        if(xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10))) {
            sharedBpm = pox.getHeartRate();
            sharedSpo2 = pox.getSpO2();
            xSemaphoreGive(xI2CMutex);
        }

        // Health check
        if(millis() - lastSensorCheck > SENSOR_CHECK_INTERVAL) {
            lastSensorCheck = millis();
            if(pox.getHeartRate() < 50.00 || pox.getHeartRate() > 250.00 || pox.getSpO2() < 80.00 || pox.getSpO2() >= 100.00) {
                recoverI2C();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void GPS_Task(void *pvParameters) {
    while (1) {
        // Read GPS data non-blocking
        while (GPS_UART.available()) {
            char c = GPS_UART.read();
            gps.encode(c);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent starvation
    }
}

void Servo_Task(void *pvParameters) {
    for(;;) {
        if (gps.speed.isValid()) {
            int newAngle = constrain((int)(gps.speed.kmph() * 180 / 140), 0, 180);
            if (abs(newAngle - currentAngle) > 2) {
                myServo.write(newAngle);
                currentAngle = newAngle;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SERVO_UPDATE_INTERVAL));
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.begin(115200);
    
    // Create I2C mutex first
    xI2CMutex = xSemaphoreCreateMutex();
    
    initializeOLED();

    // Create MAX30100 task on Core 1
    xTaskCreate(
        MAX30100_Task,
        "MAX30100_Task",
        4096,
        NULL,
        2,  // Higher priority
        &max30100TaskHandle
    );

    // GPS Initialization
    GPS_UART.setTX(GPS_TX_PIN);
    GPS_UART.setRX(GPS_RX_PIN);
    GPS_UART.begin(9600);

    // Create GPS task on Core 1
    xTaskCreate(
        GPS_Task,
        "GPS_Task",
        4096,
        NULL,
        1,  // Lower priority than MAX30100
        &gpsTaskHandle
    );

    // WiFi Connection
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, pass);
    uint32_t wifiTimeout = millis() + 20000;
    while(WiFi.status() != WL_CONNECTED && millis() < wifiTimeout) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(250);
        Serial.print(".");
    }
    
    if(WiFi.status() != WL_CONNECTED) {
        display.clearDisplay();
        display.println("WiFi Failed!");
        display.display();
        while(1);
    }

    digitalWrite(LED_PIN, HIGH);
    Serial.println("\nWiFi Connected!");
    Serial.println("IP: " + WiFi.localIP().toString());

    // Servo Initialization
    myServo.attach(SERVO_PIN);
    myServo.write(0);
    delay(500);

    myServo.write(180);
    delay(1000);
    myServo.write(0);
    delay(1000);


    display.clearDisplay();
    display.println("System Ready");
    display.display();

    xTaskCreate(
    Servo_Task,
    "Servo_Task",
    2048,
    NULL,
    1,   // priority lower than MAX30100 task but can be tweaked
    &servoTaskHandle
);

    HTTPClient http;
    String value = "";

    if(lastGpsStatus == false){
      value = "0";
    }
    else{
      value = "1";
    }

    String url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                  "&V7=" + value;

    http.begin(url);
    int httpCode = http.GET();
    Serial.print("V7 HTTP Response (GPS Status): ");
    Serial.println(httpCode);

    if (httpCode == HTTP_CODE_OK) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
        delay(100);
        digitalWrite(LED_PIN, HIGH);
    }
    http.end();
    delay(500);
  

}

void updateDisplay(float bpm, float spo2) {
    static uint32_t lastUpdate = 0;
    if(millis() - lastUpdate < 200) return;  // Limit to 5 FPS

    float lat = gps.location.isValid() ? gps.location.lat() : 0;
    float lng = gps.location.isValid() ? gps.location.lng() : 0;
    float alt = gps.altitude.isValid() ? gps.altitude.meters() : 0;
    float spd = gps.speed.isValid() ? gps.speed.kmph() : 0;

    if(xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(50))) {
        display.clearDisplay();
        display.setCursor(0,0);
        
        switch(screenIndex) {
            case 0:
                display.println("Heart Rate:");
                display.print(bpm, 1);  // 1 decimal place
                display.println(" BPM");
                display.println("SpO2:");
                display.print(spo2, 1);  // 1 decimal place
                display.println(" %");
                break;
                
            case 1:
                if (gps.location.isValid()) {
                    display.println("Latitude:");
                    display.print(lat, 6);
                    display.println();
                    display.println("Longitude:");
                    display.print(lng, 6);
                } else {
                    display.println("No GPS Fix");
                }
                break;
                
            case 2:
                if (gps.altitude.isValid()) {
                    display.println("Altitude:");
                    display.print(alt, 1);
                    display.println(" m");
                    display.println("Speed:");
                    display.print(spd, 1);
                    display.println(" km/h");
                } else {
                    display.println("No GPS Data");
                }
                break;
        }
        display.display();
        xSemaphoreGive(xI2CMutex);
        lastUpdate = millis();
    }
}

void loop() {
    if(millis() - lastReport > REPORTING_PERIOD_MS) {
        lastReport = millis();

        // Get sensor data
        if(sharedBpm > 50 && sharedSpo2 > 80){
          bpm = sharedBpm;
          spo2 = sharedSpo2;
        }

        // Update Display
        if(millis() - lastScreenChange > SCREEN_CYCLE_TIME) {
            screenIndex = (screenIndex + 1) % 3;
            lastScreenChange = millis();
        }
        updateDisplay(bpm, spo2);

        // Blynk Upload with improved handling
        if(WiFi.status() == WL_CONNECTED) {

          checkAndSendGpsStatus();

          if (initial_speed_res == 1){

            initial_speed_res = 0;

            HTTPClient http;
            String url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                       "&V9=140.00";  // Speed

            http.begin(url);
            
            int httpCode = http.GET();
            Serial.print("V9 HTTP Response: ");
            Serial.println(httpCode);
            
            if(httpCode == HTTP_CODE_OK) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
                delay(100);
                digitalWrite(LED_PIN, HIGH);
            }
            http.end();
            delay(1000);

            
            url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                       "&V9=0";  // Speed

            http.begin(url);
            
            httpCode = http.GET();
            Serial.print("V9 HTTP Response: ");
            Serial.println(httpCode);
            
            if(httpCode == HTTP_CODE_OK) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
                delay(100);
                digitalWrite(LED_PIN, HIGH);
            }
            http.end();
            delay(1000);

          }

          if(gps.location.isValid()){

            speedy = gps.speed.kmph();

            if(speedy > max_speed){
              max_speed = speedy;

              HTTPClient http;

              String url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                           "&V3=" + String(max_speed, 1);  // Speed

              http.begin(url);

              int httpCode = http.GET();
              Serial.print("V3 HTTP Response: ");
              Serial.println(httpCode);
              
              if(httpCode == HTTP_CODE_OK) {
                  digitalWrite(LED_PIN, HIGH);
              } else {
                  digitalWrite(LED_PIN, LOW);
                  delay(100);
                  digitalWrite(LED_PIN, HIGH);
              }
              http.end();
              delay(500);

            }

            HTTPClient http;
            String url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                       "&V9=" + String(gps.speed.kmph(), 1);  // Speed

            http.begin(url);
            
            int httpCode = http.GET();
            Serial.print("V9 HTTP Response: ");
            Serial.println(httpCode);
            
            if(httpCode == HTTP_CODE_OK) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
                delay(100);
                digitalWrite(LED_PIN, HIGH);
            }
            http.end();
            delay(500);

            url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                       "&V2=" + String(gps.altitude.meters(), 1); // Altitude

            http.begin(url);
            
            httpCode = http.GET();
            Serial.print("V2 HTTP Response: ");
            Serial.println(httpCode);
            
            if(httpCode == HTTP_CODE_OK) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
                delay(100);
                digitalWrite(LED_PIN, HIGH);
            }
            http.end();
            delay(500);

            url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                       "&V1=" + String(gps.location.lng()*1000000, 1);  // Longitude

            http.begin(url);
            
            httpCode = http.GET();
            Serial.print("V1 HTTP Response: ");
            Serial.println(httpCode);
            
            if(httpCode == HTTP_CODE_OK) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
                delay(100);
                digitalWrite(LED_PIN, HIGH);
            }
            http.end();
            delay(500);

            url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                       "&V0=" + String(gps.location.lat()*1000000, 1);  // Latitude

            http.begin(url);
            
            httpCode = http.GET();
            Serial.print("V0 HTTP Response: ");
            Serial.println(httpCode);
            
            if(httpCode == HTTP_CODE_OK) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
                delay(100);
                digitalWrite(LED_PIN, HIGH);
            }
            http.end();
            delay(500);
          }

          if(sharedBpm > 50 && sharedSpo2 > 80){
            HTTPClient http;
            String url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                      "&V5=" + String(spo2, 1);  // SpO2

            http.begin(url);
            
            int httpCode = http.GET();
            Serial.print("V5 HTTP Response: ");
            Serial.println(httpCode);
            
            if(httpCode == HTTP_CODE_OK) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
                delay(100);
                digitalWrite(LED_PIN, HIGH);
            }
            http.end();
            delay(500);

            url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_TOKEN) +
                      "&V4=" + String(bpm, 1);  // Heart Rate

            http.begin(url);
            
            httpCode = http.GET();
            Serial.print("V4 HTTP Response: ");
            Serial.println(httpCode);
            
            if(httpCode == HTTP_CODE_OK) {
                digitalWrite(LED_PIN, HIGH);
            } else {
                digitalWrite(LED_PIN, LOW);
                delay(100);
                digitalWrite(LED_PIN, HIGH);
            }
            http.end();
            delay(500);
          }
        }
    }

    // Heartbeat LED
    static uint32_t lastBlink = 0;
    if(millis() - lastBlink > 1000) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = millis();
    }
}
