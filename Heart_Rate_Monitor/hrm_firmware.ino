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

// For a 0.91" 128x32 OLED with I2C:
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// ---------------- Blynk Reset Settings Handler ----------------
BLYNK_WRITE(V2) {
  int state = param.asInt(); // 1 = ON, 0 = OFF
  if (state == 1) {
    Serial.println("Resetting WiFi settings...");
    Blynk.virtualWrite(V2, 0);

    wm.resetSettings();
    WiFi.disconnect(true);
    delay(1000);

    if (!wm.startConfigPortal("ESP32_ConfigAP", "12345678")) {
      Serial.println("⚠️ Config portal exited without new credentials.");
    } else {
      Serial.println("✅ New WiFi configured!");
    }
  }
}

// ---------------- Pulse Beat Callback ----------------
void onBeatDetected() {
  Serial.println("Beat!");
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
  u8g2.setFont(u8g2_font_ncenB08_tr);

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

// ---------------- Wi-Fi Setup with Bypass ----------------
#define WIFI_TIMEOUT_MS 30000   // 30s timeout
unsigned long wifiStartTime = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200; // 200 ms debounce window

void setupWiFi() {
  pinMode(WIFI_BYPASS_PIN, INPUT_PULLUP);

  // Try stored credentials first
  Serial.println("📡 Trying saved Wi-Fi...");
  WiFi.begin();

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) 
  {
    delay(200);
    Serial.print(".");
    if (digitalRead(WIFI_BYPASS_PIN) == LOW) {
      wifiBypassed = true;
      Serial.println("\n⚠️ Bypassed Wi-Fi. Running locally.");
      return;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected using saved credentials!");
    wifiConnected = true;
    return;
  }

  // If not connected → start WiFiManager portal
Serial.println("\n❌ No saved Wi-Fi, starting config portal...");
wm.setConfigPortalBlocking(false);
wm.startConfigPortal("ESP32_ConfigAP", "12345678");

wifiStartTime = millis();
wifiConnected = false;

// Keep trying for 1 minute
while (millis() - wifiStartTime < 60000) {
  wm.process();  // handle WiFiManager in non-blocking mode

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ Connected via WiFiManager!");
    wifiConnected = true;
    break;
  }

  // Optional: allow bypass during this window
  if (digitalRead(WIFI_BYPASS_PIN) == LOW) {
    wifiBypassed = true;
    Serial.println("⚠️ Wi-Fi bypassed by button press. Running locally.");
    break;
  }

  delay(100); // small delay to keep loop responsive
}

  // After 1 minute, if still not connected → offline mode
  if (!wifiConnected && !wifiBypassed) {
    Serial.println("⏳ Wi-Fi connection timeout. Running locally.");
    wifiBypassed = true;
  }
}

// ---------------- Wi-Fi Handler (call this from loop) ----------------
void handleWiFi() {
  if (wifiBypassed || wifiConnected) return;

  wm.process();  // keep config portal alive

  // Check timeout
  if (!wifiConnected && (millis() - wifiStartTime >= WIFI_TIMEOUT_MS)) {
    wifiBypassed = true;
    Serial.println("⏰ Wi-Fi setup timed out. Switching to offline mode.");
    return;
  }

  // Check Wi-Fi connection
  if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
    wifiConnected = true;
    Serial.print("✅ Wi-Fi connected! IP: ");
    Serial.println(WiFi.localIP());

    Blynk.config(BLYNK_AUTH_TOKEN);
    if (Blynk.connect(2000)) {
      Serial.println("✅ Blynk connected.");
    } else {
      Serial.println("⚠️ Blynk failed to connect.");
    }
  }
}

// ---------------- Firmware Update ----------------
void checkForUpdates() {
  if (!wifiConnected) return;

  Serial.println("🔍 Checking for updates...");
  HTTPClient http;
  http.begin(versionUrl);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println("📥 Got version.json: " + payload);

    StaticJsonDocument<512> doc;
    if (!deserializeJson(doc, payload)) {
      String latestVersion = doc["version"];
      String firmwareUrl   = doc["url"];

      Serial.printf("Current FW: %s\n", FW_VERSION);
      Serial.println("Latest FW: " + latestVersion);

      if (latestVersion != FW_VERSION) {
        Serial.println("⚡ New firmware available, updating...");
        performOTA(firmwareUrl);
      } else {
        Serial.println("✅ Already on latest firmware.");
      }
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
    if (Update.begin(contentLength)) {
      WiFiClient& client = http.getStream();
      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        Serial.println("✅ Firmware written successfully.");
      } else {
        Serial.printf("⚠️ Written only %d/%d bytes\n", written, contentLength);
      }

      if (Update.end() && Update.isFinished()) {
        Serial.println("🎉 Update complete! Rebooting...");
        ESP.restart();
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

// ---------------- OLED Wi-Fi Icon ----------------
void drawWiFiIcon() {
  if (WiFi.status() == WL_CONNECTED) {
    const uint8_t baseX = 108;
    const uint8_t baseY = 10;
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

// ---------------- Measurement Config ----------------
#define NUM_SAMPLES 200
#define SAMPLE_WINDOW   20000UL   // 20 seconds
#define SAMPLE_INTERVAL    100UL  // sample every 100 ms

float hrValues[NUM_SAMPLES];
float spo2Values[NUM_SAMPLES];
uint16_t validSamples = 0;

static bool     hrMeasurementRunning = false;
static unsigned long hrMeasurementStart = 0;
static unsigned long lastSampleMillis   = 0;

// ---------------- Measurement Functions ----------------
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

        Serial.printf("Sample %d: HR=%.1f bpm, SpO2=%.1f %%\n", validSamples, hr, currentSpO2);
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

      Serial.printf("✅ 20s Measurement Complete: HR=%.1f bpm, SpO2=%.1f %%\n", meanHR, spo2);

      if (wifiConnected) {
        Blynk.virtualWrite(0, meanHR);
        Blynk.virtualWrite(1, spo2);
      }
      saveToEEPROM(meanHR, spo2);
    } else {
      Serial.println("❌ No valid samples collected in 20s.");
      meanHR = NAN;
      spo2   = NAN;
    }

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

    return true;
  }

  return false;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialize EEPROM");
    while(true);
  }
  u8g2.begin(SDA_PIN, SCL_PIN, U8X8_PIN_NONE);

  setupWiFi();

  if (wifiConnected) {
    checkForUpdates();
    Blynk.config(BLYNK_AUTH_TOKEN);
    if (Blynk.connect(5000)) {
      Serial.println("✅ Blynk connected.");
    } else {
      Serial.println("⚠️ Blynk connection failed.");
    }
  } else {
    Serial.println("⚠️ Running in local mode (no Wi-Fi).");
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
    Blynk.run();   // ✅ non-blocking: runs only if connected
  }

  pox.update();

  static float meanHR = 0, spo2 = 0;
  static unsigned long lastFinish = 0;

  if (pollHeartRateMeasurement(meanHR, spo2)) {
    lastFinish = millis();
    Serial.println("Loop: Measurement complete, values ready.");
  }

  if (!hrMeasurementRunning && (millis() - lastFinish >= 10000)) {
    startHeartRateMeasurement();
  }
}