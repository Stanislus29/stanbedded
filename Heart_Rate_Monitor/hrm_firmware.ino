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
#include <ArduinoJson.h>

#define FW_VERSION "1.0.1"

// OTA JSON manifest
const char* versionUrl = "https://raw.githubusercontent.com/Stanislus29/stanbedded/refs/heads/main/Heart_Rate_Monitor/version.json";

#define EEPROM_SIZE 512
#define EEPROM_ADDR_HR 0
#define EEPROM_ADDR_SPO2 sizeof(float)

#define SDA_PIN 19
#define SCL_PIN 22

// Button for Wi-Fi bypass
#define WIFI_BYPASS_PIN 25   // connect button to GPIO0 and GND
#define DEBOUNCE_DELAY 200  // ms

PulseOximeter pox;
WiFiManager wm;

// State flags
bool wifiBypassed = false;
bool wifiConnected = false;

// For a 0.91" 128x64 OLED with I2C:
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// ---------------- Blynk Reset Settings Handler ----------------
BLYNK_WRITE(V2) {
  int state = param.asInt(); // 1 = ON, 0 = OFF
  if (state == 1) {
    Serial.println("🔄 Resetting WiFi settings...");
    Blynk.virtualWrite(V2, 0);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(2, 10);
    u8g2.print("WiFi reset");
    u8g2.setCursor(2, 20);
    u8g2.print("from app.....");
    u8g2.setCursor(2, 30);
    u8g2.print("Configure new WiFi");
    u8g2.setCursor(2, 40);
    u8g2.print("or press 'LOC'");
    u8g2.setCursor(2, 50);
    u8g2.print("to run locally");
    u8g2.sendBuffer();

    // Reset settings & disconnect
    wm.resetSettings();
    WiFi.disconnect(true);
    delay(1000);

    // Start config portal (non-blocking)
    wm.setConfigPortalBlocking(false);
    wm.startConfigPortal("ESP32_ConfigAP", "12345678");

    unsigned long portalStart = millis();
    bool portalSuccess = false;

    while (millis() - portalStart < 60000) {  // ⏳ 1 minute window
      wm.process();

      // Wi-Fi connected
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✅ New WiFi configured and connected!");
        portalSuccess = true;

        u8g2.clearBuffer();
        u8g2.setCursor(2, 10);
        u8g2.print("New WiFi OK!");
        u8g2.setCursor(2, 20);
        u8g2.setCursor(2, 30);
        u8g2.print("Ending Interrupt....");
        u8g2.sendBuffer();
        delay(1000);
        break;
      }

      // Offline bypass button
      if (digitalRead(WIFI_BYPASS_PIN) == LOW) {
        Serial.println("⚠️ Wi-Fi bypassed. Running locally.");
        wifiBypassed = true;

        u8g2.clearBuffer();
        u8g2.setCursor(2, 10);
        u8g2.print("Local mode");
        u8g2.setCursor(2, 30);
        u8g2.print("Ending Interrupt....");
        u8g2.sendBuffer();
        delay(1000);
        break;
      }

      delay(100); // keep responsive
    }

    // Timeout → local mode
    if (!portalSuccess && !wifiBypassed) {
      Serial.println("⏳ Config portal timeout. Running locally.");
      wifiBypassed = true;

      u8g2.clearBuffer();
      u8g2.setCursor(2, 10);
      u8g2.print("Portal timeout");
      u8g2.setCursor(2, 30);
      u8g2.print("Ending Interrupt....");
      u8g2.sendBuffer();
      delay(1000);
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

  // ✅ Show on OLED
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);

  u8g2.setCursor(2, 14);
  u8g2.print("Last Saved:");

  u8g2.setCursor(2, 32);
  u8g2.printf("HR: %.1f bpm", hrVal);

  u8g2.setCursor(2, 50);
  u8g2.printf("SpO2: %.1f %%", spo2Val);

  drawWiFiIcon();
  u8g2.sendBuffer();
  delay(300);
}

void saveToEEPROM(float meanHR, float spo2) {
  EEPROM.put(EEPROM_ADDR_HR, meanHR);
  EEPROM.put(EEPROM_ADDR_SPO2, spo2);
  EEPROM.commit();
}

// ---------------- OLED Wi-Fi Icon ----------------
void drawWiFiIcon() {
  if (WiFi.status() == WL_CONNECTED) {
    const uint8_t baseX = 108;
    const uint8_t baseY = 60;
    const uint8_t barWidth = 3;
    const uint8_t spacing = 2;

    for (int i = 0; i < 4; i++) {
      uint8_t barHeight = (i + 1) * 3;
      u8g2.drawBox(baseX + i * (barWidth + spacing),
                   baseY - barHeight,
                   barWidth,
                   barHeight);
    }
  }
}

// ---------------- Welcome Screen ----------------
void welcomeScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x14B_tr);  // Bold version

  const char* title = "VITAPULSE";
  int16_t x = (128 - u8g2.getStrWidth(title)) / 2; // 128 is typical OLED width
  int16_t y = 32; // You can adjust this vertically (usually 32 or 36 for center)

  u8g2.drawStr(x, y, title);
  u8g2.sendBuffer();
  delay(3000);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

// ---------------- Wi-Fi Setup with Bypass ----------------
#define WIFI_TIMEOUT_MS 60000   // 1 minute timeout
unsigned long wifiStartTime = 0;

void setupWiFi() {
  pinMode(WIFI_BYPASS_PIN, INPUT_PULLUP);

  // Try stored credentials first
  Serial.println("📡 Trying saved Wi-Fi...");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(2, 20);
  u8g2.print("Connecting to WiFi...");
  u8g2.setCursor(2, 30);
  u8g2.print("----------------------");
  u8g2.setCursor(2, 40);
  u8g2.print("Press 'LOC' button");
  u8g2.setCursor(2, 50);
  u8g2.print("for offline");
  u8g2.sendBuffer();

  WiFi.begin();

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    delay(200);
    if (digitalRead(WIFI_BYPASS_PIN) == LOW) {
      wifiBypassed = true;
      Serial.println("⚠️ Bypassed Wi-Fi. Running locally.");
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.setCursor(2, 30);
      u8g2.print("Local Mode Enabled");
      u8g2.sendBuffer();
      delay(300);
      return;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ Connected using saved credentials!");
    wifiConnected = true;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(2, 30);
    u8g2.print("WiFi Connected!");
    u8g2.sendBuffer();
    delay(200);
    return;
  }

  // If not connected → start WiFiManager portal
  Serial.println("❌ No saved Wi-Fi, starting config portal...");
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(2, 10);
  u8g2.print("No WiFi saved");
  u8g2.setCursor(2, 20);
  u8g2.print("Config Portal...");
  u8g2.setCursor(2, 30);
  u8g2.print("----------------------");
  u8g2.setCursor(2, 40);
  u8g2.print("Press 'LOC' button");
  u8g2.setCursor(2, 50);
  u8g2.print("for offline");
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
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.setCursor(2, 30);
      u8g2.print("WiFi Connected!");
      u8g2.sendBuffer();
      delay(200);
      break;
    }

    if (digitalRead(WIFI_BYPASS_PIN) == LOW) {
      wifiBypassed = true;
      Serial.println("⚠️ Wi-Fi bypassed.");
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.setCursor(2, 30);
      u8g2.print("Local Mode Enabled");
      u8g2.sendBuffer();
      delay(300);
      break;
    }

    delay(100);
  }

  if (!wifiConnected && !wifiBypassed) {
    Serial.println("⏳ Wi-Fi timeout. Running locally.");
    wifiBypassed = true;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(2, 20);
    u8g2.print("WiFi Timeout");
    u8g2.setCursor(2, 40);
    u8g2.print("Local Mode Enabled");
    u8g2.sendBuffer();
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

        // Show progress
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tr);
        const char* msg = "Measuring...";
        u8g2.setCursor((128 - u8g2.getStrWidth(msg)) / 2, 14);
        u8g2.print(msg);

        u8g2.setCursor(2, 32);
        u8g2.printf("Samples: %d", validSamples);

        u8g2.setCursor(2, 50);
        u8g2.printf("HR: %.1f  SpO2: %.1f", hr, currentSpO2);

        drawWiFiIcon();
        u8g2.sendBuffer();
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

    // Show final results
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(2, 14);
    u8g2.print("Results:");

    u8g2.setCursor(2, 32);
    if (!isnan(meanHR)) u8g2.printf("HR: %.1f bpm", meanHR);
    else u8g2.print("HR: --");

    u8g2.setCursor(2, 50);
    if (!isnan(spo2)) u8g2.printf("SpO2: %.1f %%", spo2);
    else u8g2.print("SpO2: --");

    drawWiFiIcon();
    u8g2.sendBuffer();

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

  welcomeScreen();
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