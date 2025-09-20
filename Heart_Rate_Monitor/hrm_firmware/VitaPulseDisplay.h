#ifndef VITAPULSE_DISPLAY_H
#define VITAPULSE_DISPLAY_H

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

class VitaPulseDisplay {
  public:
    VitaPulseDisplay(U8G2 &display);  // constructor
    void begin();

    void showOLEDMessage(const char* title, const char* body = "", const char* footer = "");
    // Screens
    void showWelcome();
    void showLocalMode();
    void showInterrupt();
    void showConnected(uint8_t frame);
    void showConnecting(uint8_t strength, uint8_t dotCount, bool buttonSelected);
    void showVitals(const char* label, float heartRate, float spo2, uint8_t signalStrength);
    // Call this every loop to animate the connecting screen
    void updateConnecting(bool buttonSelected);

  private:
    U8G2 &u8g2;

     // Connecting animation state
    uint8_t connStrength = 0;
    uint8_t connDotCount = 0;
    bool connButtonSelected = false;

    // Helpers
    void drawHeartIcon(int x, int y);
    void drawSignalBars(uint8_t x, uint8_t y, uint8_t strength);       // vitals
    void drawSignalBarsConn(uint8_t x, uint8_t y, uint8_t strength);   // connecting
};

#endif