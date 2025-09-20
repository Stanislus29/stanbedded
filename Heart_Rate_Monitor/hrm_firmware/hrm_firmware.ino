/*

  FIRMWARE FOR "VITAPULSE" - a heart rate monitor
  This device monitors the heart rate of the individual it is strapped to, and provides cloud based inference via Blynk

  Author: Somtochukwu Emeka-Onwuneme
  Copyright © 2025 Somtochukwu Stanislus Emeka-Onwuneme

*/

// ------------------- Blynk + WiFi Setup -------------------
#define BLYNK_TEMPLATE_ID "TMPL2fqznONIY"
#define BLYNK_TEMPLATE_NAME "Personal Health Monitoring System"
#define BLYNK_AUTH_TOKEN "QtSGB02k4xZy0tlON2l-jGPzB_ow-9py"

// ---------------Initialize libraries-------------------
#include <EEPROM.h> //Used to enable EEPROM storage 
#include <Wire.h> //Library for I2C communication 
#include "MAX30100_PulseOximeter.h" //MAX30100 library 
#include <WiFi.h> //Adds WIFi compatability to the ESP32
#include <BlynkSimpleEsp32.h> //Blynk library for the ESP32
#include <U8g2lib.h>  //Library for OLED
#include <WiFiManager.h>  //Library for the WiFi manager 
#include <Arduino.h>  
#include <HTTPClient.h> //Library for Over The Air (OTA) firmware updates
#include <Update.h>
#include <ArduinoJson.h>
#include "VitaPulseDisplay.h"

#define FW_VERSION "1.0.1"

// OTA JSON manifest
const char* versionUrl = "https://raw.githubusercontent.com/Stanislus29/stanbedded/refs/heads/main/Heart_Rate_Monitor/version.json";

#define EEPROM_SIZE 512
#define EEPROM_ADDR_HR 0
#define EEPROM_ADDR_SPO2 sizeof(float)

#define SDA_PIN 19
#define SCL_PIN 22
#define LOC_PIN 25

// Button for Wi-Fi bypass
#define DEBOUNCE_DELAY 200  // ms

PulseOximeter pox;
WiFiManager wm;

// State flags
bool wifiBypassed = false;
bool wifiConnected = false;

int yOffset = 0; // or some pixel offset
int btnY = 54 + yOffset;

static uint8_t frame = 0;

// For a 0.91" 128x64 OLED with I2C:
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

bool buttonSelected = digitalRead(LOC_PIN) == LOW;

VitaPulseDisplay display(u8g2);

// ---------------- Blynk Reset Settings Handler ----------------
BLYNK_WRITE(V2) {
  int state = param.asInt(); // 1 = ON, 0 = OFF
  if (state == 1) {
    Serial.println("🔄 Resetting WiFi settings...");
    Blynk.virtualWrite(V2, 0);

    display.showInterrupt();
    delay(2000);

    // Reset settings & disconnect
    wm.resetSettings();
    WiFi.disconnect(true);
    delay(1000);

    // Start config portal (non-blocking)
    wm.setConfigPortalBlocking(false);
    wm.startConfigPortal("ESP32_ConfigAP", "12345678");

    bool buttonSelected = digitalRead(LOC_PIN) == LOW;

    display.showOLEDMessage("Configure New WiFi", "OR", "");

    int btnX = 20, btnY = 46 + yOffset, btnW = 40, btnH = 14;

    u8g2.setDrawColor(1);
    u8g2.drawBox(btnX, btnY, btnW, btnH);
    u8g2.setFont(u8g2_font_8x13B_tr);
    int textWidth = u8g2.getStrWidth("LOC");
    int textX = btnX + (btnW - textWidth)/2;
    int textY = btnY + btnH - 3;
    u8g2.setDrawColor(0);
    u8g2.drawStr(textX, textY, "LOC");
    u8g2.setDrawColor(1);

    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(btnX + btnW + 8, btnY + btnH - 3, "-> Exit");
    u8g2.sendBuffer();

    unsigned long portalStart = millis();
    bool portalSuccess = false;
    bool shouldRestart = false;

while (millis() - portalStart < 60000) 
  {
      wm.process();
      buttonSelected = digitalRead(LOC_PIN) == LOW;

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✅ New WiFi configured and connected!");
        portalSuccess = true;

        for (uint8_t frame = 0; frame <= 25; frame++) {  // 25 frames for animation
                display.showConnected(frame);
                delay(40);  // small delay for smooth animation
            }

        shouldRestart = true;
        break;
      }

      static unsigned long lastPress = 0;   // stores last time button was pressed

      if (buttonSelected && millis() - lastPress > 200) {  // 200 ms debounce
          lastPress = millis();  // update last press time
          wifiBypassed = true;
          Serial.println("⚠️ Bypassed Wi-Fi. Running locally.");

          display.showOLEDMessage("Configure New WiFi", "OR", "");

          u8g2.drawFrame(btnX, btnY, btnW, btnH);
          u8g2.setFont(u8g2_font_8x13B_tr);
          int textWidth = u8g2.getStrWidth("LOC");
          int textX = btnX + (btnW - textWidth)/2;
          int textY = btnY + btnH - 3;
          u8g2.drawStr(textX, textY, "LOC");

          u8g2.setFont(u8g2_font_6x12_tr);
          u8g2.drawStr(btnX + btnW + 8, btnY + btnH - 3, "-> Exit");
          u8g2.sendBuffer();

          delay(1000);

          shouldRestart = true;
          delay(1000);
          return;
      }
    delay(100);
  }

    if (!portalSuccess && !wifiBypassed) {
      pox.update();
      Serial.println("⏳ Config portal timeout. Running locally.");
      wifiBypassed = true;

      display.showOLEDMessage("Portal Timeout", "", "Exiting setup...");

      shouldRestart = true;
    }

    if (shouldRestart) {
      ESP.restart();
    }
  }
}

// ---------------- EEPROM Helpers ----------------
void readFromEEPROM() {
  float hrVal, spo2Val;
  EEPROM.get(EEPROM_ADDR_HR, hrVal);
  EEPROM.get(EEPROM_ADDR_SPO2, spo2Val);

  Serial.println("---- Last Saved Reading ----");
  Serial.printf("Heart rate: %.1f bpm\n", hrVal);
  Serial.printf("SpO2: %.1f %%\n", spo2Val);
  Serial.println("----------------------------");

  display.showVitals("RES", hrVal, spo2Val, 4);

  delay(3000);
}

void saveToEEPROM(float meanHR, float spo2) {
  EEPROM.put(EEPROM_ADDR_HR, meanHR);
  EEPROM.put(EEPROM_ADDR_SPO2, spo2);
  EEPROM.commit();
}

// ---------------- Wi-Fi Setup with Bypass ----------------
#define WIFI_TIMEOUT_MS 60000   // 1 minute timeout
unsigned long wifiStartTime = 0;

void setupWiFi() {
  pinMode(LOC_PIN, INPUT_PULLUP);

  // Read button state
  bool buttonSelected = digitalRead(LOC_PIN) == LOW;

  // Try stored credentials first
  Serial.println("📡 Trying saved Wi-Fi...");

  display.updateConnecting(buttonSelected);

  WiFi.begin();

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) 
    {
      delay(200);

      // Refresh button state inside loop
          buttonSelected = digitalRead(LOC_PIN) == LOW;
      // Update OLED connecting animation
          display.updateConnecting(buttonSelected);

          static unsigned long lastPress = 0;  // store last press time

          if (buttonSelected && millis() - lastPress > 200) {  // 200 ms debounce
              lastPress = millis();  // update last press time

              wifiBypassed = true;
              Serial.println("⚠️ Bypassed Wi-Fi. Running locally.");

              display.showLocalMode();
              delay(1000);
              return;
          }

          if (WiFi.status() == WL_CONNECTED) {
            wifiConnected = true;
            Serial.println("✅ Connected using saved credentials!");

            for (uint8_t frame = 0; frame <= 25; frame++) {  // 20 frames for animation
                display.showConnected(frame);
                delay(40);  // small delay for smooth animation
            }
            return;
        }

} 

  // If not connected → start WiFiManager portal
  Serial.println("❌ No saved Wi-Fi, starting config portal...");

  display.showOLEDMessage("No WiFi Saved", "Config Portal...","");

  int btnX = 20, btnY = 54 + yOffset, btnW = 40, btnH = 14;

  u8g2.setDrawColor(1);
  u8g2.drawBox(btnX, btnY, btnW, btnH);
  u8g2.setFont(u8g2_font_8x13B_tr);
  int textWidth = u8g2.getStrWidth("LOC");
  int textX = btnX + (btnW - textWidth)/2;
  int textY = btnY + btnH - 3;
  u8g2.setDrawColor(0);
  u8g2.drawStr(textX, textY, "LOC");
  u8g2.setDrawColor(1);

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(btnX + btnW + 8, btnY + btnH - 3, "-> Exit");
  u8g2.sendBuffer();

  wm.setConfigPortalBlocking(false);
  wm.startConfigPortal("ESP32_ConfigAP", "12345678");
  wifiStartTime = millis();
  wifiConnected = false;

  while (millis() - wifiStartTime < WIFI_TIMEOUT_MS) {
    wm.process();

    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.println("✅ Connected via WiFiManager!");
      
      display.showConnected(frame);
      delay(1000);

      break;
    }

    if (digitalRead(LOC_PIN) == LOW) {
      wifiBypassed = true;
      Serial.println("⚠️ Wi-Fi bypassed.");
      
      display.showLocalMode();
      delay(1000);

      break;
    }

    delay(100);
  }

  if (!wifiConnected && !wifiBypassed) {
    Serial.println("⏳ Wi-Fi timeout. Running locally.");
    wifiBypassed = true;

    display.showLocalMode();
    delay(400);
  }
}

// ---------------- Wi-Fi Handler ----------------
void handleWiFi() {
  if (wifiBypassed || wifiConnected) return;

  wm.process();

  if (!wifiConnected && (millis() - wifiStartTime >= WIFI_TIMEOUT_MS)) {
    wifiBypassed = true;
    Serial.println("⏰ Wi-Fi setup timed out.");

    display.showLocalMode();
    return;
  }

  if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
    wifiConnected = true;
    Serial.print("✅ Wi-Fi connected! IP: ");
    Serial.println(WiFi.localIP());

    Blynk.config(BLYNK_AUTH_TOKEN);
    if (Blynk.connect(2000)) {
      Serial.println("✅ Blynk connected.");
    } else {
      Serial.println("⚠️ Blynk failed.");
    }
  }
}

// ---------------- Pulse Beat Callback ----------------
void onBeatDetected() {
  Serial.println("Beat!");
}

// ---------------- Measurement Config ----------------
#define NUM_SAMPLES 200
#define SAMPLE_WINDOW   20000UL   // 20 seconds
#define SAMPLE_INTERVAL 100UL     // sample every 100 ms

float hrValues[NUM_SAMPLES];
float spo2Values[NUM_SAMPLES];
uint16_t validSamples = 0;

static bool hrMeasurementRunning = false;
static unsigned long hrMeasurementStart = 0;
static unsigned long lastSampleMillis   = 0;

void startHeartRateMeasurement() {
  validSamples = 0;
  hrMeasurementStart = millis();
  lastSampleMillis   = hrMeasurementStart;
  hrMeasurementRunning = true;
  Serial.println("📡 Starting 20s measurement...");
}

bool pollHeartRateMeasurement(float &meanHR, float &spo2) {
  if (!hrMeasurementRunning) return false;

  unsigned long now = millis();
  pox.update();

  if (now - lastSampleMillis >= SAMPLE_INTERVAL) {
    lastSampleMillis = now;

    float hr = pox.getHeartRate();
    float currentSpO2 = pox.getSpO2();

    if (!isnan(hr) && hr > 40 && hr < 180 &&
        !isnan(currentSpO2) && currentSpO2 > 70 && currentSpO2 <= 100) {
      if (validSamples < NUM_SAMPLES) {
        hrValues[validSamples]   = hr;
        spo2Values[validSamples] = currentSpO2;
        validSamples++;

        Serial.println("Measuring...");
        Serial.printf("Samples: %d\n", validSamples);
        Serial.printf("HR: %.1f  SpO2: %.1f\n", hr, currentSpO2);
      }
    }
  }

  if (now - hrMeasurementStart >= SAMPLE_WINDOW) {
    hrMeasurementRunning = false;

    if (validSamples > 0) {
      float sumHR = 0.0f, sumSPO2 = 0.0f;
      for (uint16_t i = 0; i < validSamples; ++i) {
        sumHR   += hrValues[i];
        sumSPO2 += spo2Values[i];
      }
      meanHR = sumHR / validSamples;
      spo2   = sumSPO2 / validSamples;

      Serial.printf("✅ Complete: HR=%.1f bpm, SpO2=%.1f %%\n", meanHR, spo2);

     if (wifiConnected) {
        Blynk.virtualWrite(0, meanHR);
        Blynk.virtualWrite(1, spo2);
      }
      saveToEEPROM(meanHR, spo2);
    } else {
      Serial.println("❌ No valid samples.");
      meanHR = NAN;
      spo2   = NAN;
    }

    if (!isnan(meanHR) && !isnan(spo2)) {
    display.showVitals("RES", meanHR, spo2, 4);
    } 
    else {
        readFromEEPROM();
    }
    return true;
  }

  return false;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to init EEPROM");
    while(true);
  }
  u8g2.begin(SDA_PIN, SCL_PIN, U8X8_PIN_NONE);

  display.showWelcome();
  delay(2000);
  
  setupWiFi();

  if (wifiConnected) {
    Blynk.config(BLYNK_AUTH_TOKEN);
    if (Blynk.connect(5000)) {
      Serial.println("✅ Blynk connected.");
    } else {
      Serial.println("⚠️ Blynk connect failed.");
    }
  } else {
    Serial.println("⚠️ Local mode (no Wi-Fi).");
  }

  readFromEEPROM();

  Serial.print("Initializing pulse oximeter...");
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

// ---------------- Loop ----------------
void loop() {
  handleWiFi();
  if (wifiConnected) {
    Blynk.run();
  }

  pox.update();

  static float meanHR = 0, spo2 = 0;
  static unsigned long lastFinish = 0;

  if (pollHeartRateMeasurement(meanHR, spo2)) {
    lastFinish = millis();
  }

  if (!hrMeasurementRunning && (millis() - lastFinish >= 10000)) {
    startHeartRateMeasurement();
  }
}