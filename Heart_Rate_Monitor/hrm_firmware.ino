/*
Firmware for a Heart Rate Monitor built with an OLED, MAX30100 sensor and ESP32

Company: Stan's Technologies
Authors: Somtochukwu Emeka-Onwuneme, Kofi Aseda Ayeh-Bampoe
email: somtochukwueo@outlook.com
Copyright © 2025 Somtochukwu Stanislus Emeka-Onwuneme
*/ 


// ------------------- Blynk + WiFi Setup -------------------
#define BLYNK_TEMPLATE_ID "TMPL2fqznONIY"
#define BLYNK_TEMPLATE_NAME "Personal Health Monitoring System"
#define BLYNK_AUTH_TOKEN "QtSGB02k4xZy0tlON2l-jGPzB_ow-9py"

#include <EEPROM.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <U8g2lib.h>
#include <WiFiManager.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>  // install ArduinoJson library

#define FW_VERSION "1.0.0"

// where version.json is hosted
const char* versionUrl = "https://raw.githubusercontent.com/Stanislus29/stanbedded/refs/heads/main/Heart_Rate_Monitor/version.json";

#define EEPROM_SIZE 512
#define EEPROM_ADDR_HR 0
#define EEPROM_ADDR_SPO2 sizeof(float)

#define REPORTING_PERIOD_MS 1000
#define SDA_PIN 19
#define SCL_PIN 22
#define NUM_SAMPLES 5
#define HR_MIN 60
#define HR_MAX 90
#define SPO2_MIN 90
#define SPO2_MAX 100

PulseOximeter pox;
uint32_t tsLastReport = 0;
WiFiManager wm;

// Globals to hold latest readings
float meanHR = 0;
float spo2 = NAN;
bool readingSaved = false;  // flag to stop loop after saving

// For a 0.91" 128x32 OLED with I2C (common on ESP32 boards):
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

BLYNK_WRITE(V2) {   // Virtual pin for the switch widget
  int state = param.asInt(); // 1 = ON, 0 = OFF
  if (state == 1) {
    Serial.println("Resetting WiFi settings...");
    Blynk.virtualWrite(V2, 0); // reset the switch back to OFF in the app

    wm.resetSettings();        // erase stored SSID & password

    // Instead of restarting, disconnect WiFi immediately
    WiFi.disconnect(true);  
    delay(1000);

    // Start config portal right away (blocking until new WiFi is set)
    if (!wm.startConfigPortal("ESP32_ConfigAP", "12345678")) {
      Serial.println("⚠️ Config portal exited without new credentials.");
    } else {
      Serial.println("✅ New WiFi configured!");
    }
  }
}

// Callback when a pulse is detected
void onBeatDetected() {
  Serial.println("Beat!");
}

// Read last saved values from EEPROM
void readFromEEPROM() {
  float hrVal, spo2Val;
  EEPROM.get(EEPROM_ADDR_HR, hrVal);
  EEPROM.get(EEPROM_ADDR_SPO2, spo2Val);

  Serial.println("---- Last Saved Reading ----");
  Serial.print("Heart rate: ");
  Serial.print(hrVal);
  Serial.println(" bpm");

  Serial.print("SpO2: ");
  Serial.print(spo2Val);
  Serial.println(" %");
  Serial.println("----------------------------");

  // ✅ Show on OLED
u8g2.clearBuffer();
u8g2.setFont(u8g2_font_ncenB08_tr);  // medium, readable font

u8g2.setCursor(2, 14);
u8g2.print("Last Saved:");

u8g2.setCursor(2, 32);
u8g2.print("HR: ");
u8g2.print(hrVal, 1);
u8g2.print(" bpm");

u8g2.setCursor(2, 50);
u8g2.print("SpO2: ");
u8g2.print(spo2Val, 1);
u8g2.print(" %");


drawWiFiIcon();

u8g2.sendBuffer();
}

void saveToEEPROM(float meanHR, float spo2) {
  EEPROM.put(EEPROM_ADDR_HR, meanHR);
  EEPROM.put(EEPROM_ADDR_SPO2, spo2);
  EEPROM.commit();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!EEPROM.begin(EEPROM_SIZE)) {
      Serial.println("Failed to initialize EEPROM");
      return;
    }
  u8g2.begin(19, 22, U8X8_PIN_NONE);
  
  // Try WiFiManager auto connect
  if (wm.autoConnect("ESP32_ConfigAP", "12345678")) {
    Serial.println("✅ Connected to WiFi!");
  } else {
    Serial.println("❌ Failed to connect, restarting...");
    ESP.restart();
  }

   // check for updates at boot
  checkForUpdates();

  // Configure Blynk (Wi-Fi is already handled by WiFiManager)
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();

  readFromEEPROM();  

  Serial.print("Initializing pulse oximeter..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }
 
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

// ✅ Draw network bars if connected
void drawWiFiIcon() {
  if (WiFi.status() == WL_CONNECTED) {
    const uint8_t baseX = 108;  // starting x (right side of 128px wide display)
    const uint8_t baseY = 10;   // baseline y position (bottom of bars)
    const uint8_t barWidth = 3;
    const uint8_t spacing = 2;

    // Draw 4 bars with increasing heights
    for (int i = 0; i < 4; i++) {
      uint8_t barHeight = (i + 1) * 3; // heights: 3, 6, 9, 12 px
      u8g2.drawBox(baseX + i * (barWidth + spacing),
                   baseY - barHeight,
                   barWidth,
                   barHeight);
    }
  }
}

void checkForUpdates() {
  Serial.println("🔍 Checking for updates...");

  HTTPClient http;
  http.begin(versionUrl);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println("📥 Got version.json: " + payload);

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      String latestVersion = doc["version"];
      String firmwareUrl   = doc["url"];

      Serial.println("Current FW: " + String(FW_VERSION));
      Serial.println("Latest FW: " + latestVersion);

      if (latestVersion != FW_VERSION) {
        Serial.println("⚡ New firmware available, updating...");
        performOTA(firmwareUrl);
      } else {
        Serial.println("✅ Already on latest firmware.");
      }
    } else {
      Serial.println("❌ Failed to parse JSON");
    }
  } else {
    Serial.printf("⚠️ Failed to fetch version.json, HTTP code: %d\n", httpCode);
  }
  http.end();
}

void performOTA(String firmwareUrl) {
  HTTPClient http;
  http.begin(firmwareUrl);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    bool canBegin = Update.begin(contentLength);

    if (canBegin) {
      WiFiClient& client = http.getStream();
      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        Serial.println("✅ Firmware written successfully.");
      } else {
        Serial.printf("⚠️ Written only %d/%d bytes\n", written, contentLength);
      }

      if (Update.end()) {
        if (Update.isFinished()) {
          Serial.println("🎉 Update complete! Rebooting...");
          ESP.restart();
        } else {
          Serial.println("❌ Update not finished properly.");
        }
      } else {
        Serial.printf("❌ Update error: %d\n", Update.getError());
      }
    } else {
      Serial.println("❌ Not enough space for OTA");
    }
  } else {
    Serial.printf("⚠️ Failed to download firmware, HTTP code: %d\n", httpCode);
  }
  http.end();
}

// --- CONFIG (tweak if needed) ---

#define NUM_SAMPLES 200
#define SAMPLE_WINDOW   20000UL   // 20 seconds
#define SAMPLE_INTERVAL    100UL  // sample every 100 ms

// --- Globals used by the non-blocking measurement ---
float hrValues[NUM_SAMPLES];
float spo2Values[NUM_SAMPLES];
uint16_t validSamples = 0;

static bool     hrMeasurementRunning = false;
static unsigned long hrMeasurementStart = 0;
static unsigned long lastSampleMillis   = 0;

// ------------------------------------------------------------------
// Call this to start a new 20s measurement window
// ------------------------------------------------------------------
void startHeartRateMeasurement() {
  validSamples = 0;
  hrMeasurementStart = millis();
  lastSampleMillis   = hrMeasurementStart;
  hrMeasurementRunning = true;
  Serial.println("📡 Starting 20s measurement...");
}

// ------------------------------------------------------------------
// Non-blocking poll function. Call from loop() as often as possible.
// Returns true once measurement finished and meanHR/spo2 are valid.
// If no valid samples collected, meanHR/spo2 will be NAN.
// ------------------------------------------------------------------
bool pollHeartRateMeasurement(float &meanHR, float &spo2) {
  if (!hrMeasurementRunning) {
    return false; // nothing running
  }

  unsigned long now = millis();

  // Keep the sensor updated frequently
  pox.update();

  // Take a sample at SAMPLE_INTERVAL
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

        Serial.print("Sample ");
        Serial.print(validSamples);
        Serial.print(": HR=");
        Serial.print(hr);
        Serial.print(" bpm, SpO2=");
        Serial.print(currentSpO2);
        Serial.println(" %");
      } else {
        // defensive: buffer full
        Serial.println("⚠️ hr/spo2 buffer full, skipping sample");
      }
    }
  }

  // Check if measurement window ended
  if (now - hrMeasurementStart >= SAMPLE_WINDOW) {
    // stop measuring
    hrMeasurementRunning = false;

    if (validSamples > 0) {
      float sumHR = 0.0f, sumSPO2 = 0.0f;
      for (uint16_t i = 0; i < validSamples; ++i) {
        sumHR   += hrValues[i];
        sumSPO2 += spo2Values[i];
      }
      meanHR = sumHR / validSamples;
      spo2   = sumSPO2 / validSamples;

      Serial.println("✅ 20s Measurement Complete:");
      Serial.print("Average HR: ");
      Serial.print(meanHR);
      Serial.println(" bpm");

      Serial.print("Average SpO2: ");
      Serial.print(spo2);
      Serial.println(" %");

      Blynk.virtualWrite(0, meanHR);
      Blynk.virtualWrite(1, spo2);
      saveToEEPROM(meanHR, spo2);
    } else {
      Serial.println("❌ No valid samples collected in 20s.");
      meanHR = NAN;
      spo2   = NAN;
    }

    // Update OLED snapshot (same UI you had)
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    u8g2.setCursor(2, 14);
    u8g2.print("Current:");

    u8g2.setCursor(2, 32);
    u8g2.print("HR: ");
    if (!isnan(meanHR)) u8g2.print(meanHR, 1); else u8g2.print("--");
    u8g2.print(" bpm");

    u8g2.setCursor(2, 50);
    u8g2.print("SpO2: ");
    if (!isnan(spo2)) u8g2.print(spo2, 1); else u8g2.print("--");
    u8g2.print(" %");

    drawWiFiIcon();
    u8g2.sendBuffer();

    return true; // measurement finished
  }

  return false; // still measuring
}

void loop() {
  Blynk.run();           // keep Blynk/WiFi alive
  pox.update();          // keep sensor backend alive (good to call frequently)

  static float meanHR = 0, spo2 = 0;
  static unsigned long lastFinish = 0;

  // 1) Poll the measurement state machine
  if (pollHeartRateMeasurement(meanHR, spo2)) {
    // ✅ Measurement finished
    lastFinish = millis();  // mark end time
    Serial.println("Loop: Measurement complete, values ready.");
  }

  // 2) Restart automatically after 10s pause
  if (!hrMeasurementRunning && (millis() - lastFinish >= 10000)) {
    startHeartRateMeasurement();
  }

  // ⚠️ No delay() here — keep loop responsive
}