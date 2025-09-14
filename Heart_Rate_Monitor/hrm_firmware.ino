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

char ssid[] = "TS9T Stan";      // Your WiFi SSID
char pass[] = "stan29ccco";  // Your WiFi Password

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

// Globals to hold latest readings
float meanHR = 0;
float spo2 = NAN;
bool readingSaved = false;  // flag to stop loop after saving

// For a 0.91" 128x32 OLED with I2C (common on ESP32 boards):
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

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

   // Connect to WiFi + Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  // Show last saved values before running new measurements
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

void showWiFiStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✅ WiFi connected: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("❌ WiFi not connected.");
  }
}

// Globals
float hrValues[NUM_SAMPLES];
uint8_t validSamples = 0;

void getHeartRate(float &meanHR, float &spo2) {
  const unsigned long SAMPLE_WINDOW = 20000;  // 20 seconds
  unsigned long startTime = millis();

  float hrValues[200];    // max ~200 samples over 20s @100ms interval
  float spo2Values[200];
  uint16_t validSamples = 0;

  Serial.println("📡 Starting 20s measurement...");

  while (millis() - startTime < SAMPLE_WINDOW) {
    pox.update();
    float hr = pox.getHeartRate();
    float currentSpO2 = pox.getSpO2();

    if (!isnan(hr) && hr > 40 && hr < 180 &&
        !isnan(currentSpO2) && currentSpO2 > 70 && currentSpO2 <= 100) {
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
    }

    delay(100);  // sample every 100ms
  }

  if (validSamples > 0) {
    float sumHR = 0, sumSPO2 = 0;
    for (int i = 0; i < validSamples; i++) {
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
  }

  // ✅ Show new snapshot on OLED
  u8g2.clearBuffer();
u8g2.setFont(u8g2_font_ncenB08_tr);

u8g2.setCursor(2, 14);
u8g2.print("Current:");

u8g2.setCursor(2, 32);
u8g2.print("HR: ");
u8g2.print(meanHR, 1);
u8g2.print(" bpm");

u8g2.setCursor(2, 50);
u8g2.print("SpO2: ");
u8g2.print(spo2, 1);
u8g2.print(" %");

u8g2.sendBuffer();
}

void loop() {
  Blynk.run();
  showWiFiStatus();       // optional, will print WiFi status every loop
  getHeartRate(meanHR, spo2);  // take a 20s measurement
  delay(20000);           // wait 20s before starting the next measurement
}