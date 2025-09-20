#include "VitaPulseDisplay.h"

// ---------------- Constructor ----------------
VitaPulseDisplay::VitaPulseDisplay(U8G2 &display) : u8g2(display) {}

// ---------------- Begin ----------------
void VitaPulseDisplay::begin() {
    u8g2.begin();
}

// ---------------- Helpers ----------------
void VitaPulseDisplay::drawHeartIcon(int x, int y) {
    u8g2.drawDisc(x + 4, y - 4, 4, U8G2_DRAW_ALL);
    u8g2.drawDisc(x + 12, y - 4, 4, U8G2_DRAW_ALL);
    for (int i = 0; i < 8; ++i) {
        int sx = x + 1 + i;
        int w = 14 - i * 2;
        if (w > 0) u8g2.drawBox(sx, y - 2 + i, w, 1);
    }
}

void VitaPulseDisplay::drawSignalBars(uint8_t x, uint8_t y, uint8_t strength) {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t barHeight = (i + 1) * 3;
        if (i < strength) u8g2.drawBox(x + i * 4, y - barHeight, 3, barHeight);
        else u8g2.drawFrame(x + i * 4, y - barHeight, 3, barHeight);
    }
}

void VitaPulseDisplay::drawSignalBarsConn(uint8_t x, uint8_t y, uint8_t strength) {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t barHeight = (i + 1) * 4;
        if (i < strength) u8g2.drawBox(x + i * 6, y - barHeight, 5, barHeight);
        else u8g2.drawFrame(x + i * 6, y - barHeight, 5, barHeight);
    }
}

// ---------------- Screens ----------------
void VitaPulseDisplay::showWelcome() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_logisoso16_tr);

    const char *title = "VITAPULSE";
    int titleW = u8g2.getStrWidth(title);
    int iconW = 18;
    int totalW = titleW + iconW + 6;
    int startX = (128 - totalW) / 2;
    int blockH = 18;
    int startY = (64 + blockH) / 2;

    drawHeartIcon(startX, startY - 6);
    u8g2.setCursor(startX + iconW + 6, startY);
    u8g2.print(title);
    u8g2.sendBuffer();
}

void VitaPulseDisplay::showLocalMode() {
    u8g2.clearBuffer();
    int x = 64, y = 28;
    u8g2.drawTriangle(x - 15, y, x, y - 15, x + 15, y);  // roof
    u8g2.drawFrame(x - 12, y, 24, 18);                   // house body
    u8g2.setFont(u8g2_font_6x12_tr);
    const char *msg = "Running Offline";
    u8g2.drawStr((128 - u8g2.getStrWidth(msg)) / 2, 62, msg);
    u8g2.sendBuffer();
}

void VitaPulseDisplay::showInterrupt() {
    u8g2.clearBuffer();
    int cx = 64, cy = 28;

    // Filled triangle
    u8g2.drawTriangle(cx - 18, cy + 18, cx + 18, cy + 18, cx, cy - 20);

    // Inverted "!" inside
    u8g2.setFont(u8g2_font_logisoso22_tr);
    const char *warn = "!";
    int w = u8g2.getStrWidth(warn);
    int h = 22;
    u8g2.setDrawColor(0); // inverted
    u8g2.drawStr(cx - w/2, cy + h/2 -2, warn);
    u8g2.setDrawColor(1);

    // Text below
    u8g2.setFont(u8g2_font_6x12_tr);
    const char *msg = "WiFi reset from app";
    u8g2.drawStr((128 - u8g2.getStrWidth(msg)) / 2, 62, msg);
    u8g2.sendBuffer();
}

void VitaPulseDisplay::showConnected(uint8_t frame) {
    u8g2.clearBuffer();

    int cx = 50;
    int cy = 30;

    int shortArm = 8;  // length of short arm
    int longArm  = 20; // length of long arm
    int thickness = 3; // line thickness

    int shortLen = std::min<int>((int)frame, shortArm);
    int longLen  = std::min<int>((int)(frame > shortArm ? frame - shortArm : 0), longArm);

    // Draw short arm of checkmark (thicker)
    for (int t = 0; t < thickness; t++) {
        u8g2.drawLine(cx, cy + t, cx + shortLen, cy + shortLen + t);
    }

    // Draw long arm of checkmark (thicker)
    for (int t = 0; t < thickness; t++) {
        u8g2.drawLine(cx + shortLen, cy + shortLen + t, cx + shortLen + longLen, cy + shortLen - longLen + t);
    }

    // Label below checkmark
    u8g2.setFont(u8g2_font_6x12_tr); // same font as "Running Offline"
    const char* msg = "WiFi Connected";
    u8g2.drawStr((128 - u8g2.getStrWidth(msg)) / 2, 62, msg);

    u8g2.sendBuffer();
}

void VitaPulseDisplay::showConnecting(uint8_t strength, uint8_t dotCount, bool buttonSelected) {
    u8g2.clearBuffer();
    int yOffset = -3;

    int totalWidth = 4*6 - 1;
    int startX = (128 - totalWidth)/2;
    drawSignalBarsConn(startX, 30 + yOffset, strength);

    char msg[25];
    snprintf(msg, sizeof(msg), "Connecting%.*s", dotCount, "............");
    u8g2.setFont(u8g2_font_6x12_tr);
    int msgWidth = u8g2.getStrWidth(msg);
    u8g2.drawStr((128 - msgWidth)/2, 45 + yOffset, msg);

    int btnX = 20, btnY = 54 + yOffset, btnW = 40, btnH = 14;

    if (!buttonSelected) {
        u8g2.setDrawColor(1);
        u8g2.drawBox(btnX, btnY, btnW, btnH);
        u8g2.setFont(u8g2_font_8x13B_tr);
        int textWidth = u8g2.getStrWidth("LOC");
        int textX = btnX + (btnW - textWidth)/2;
        int textY = btnY + btnH - 3;
        u8g2.setDrawColor(0);
        u8g2.drawStr(textX, textY, "LOC");
        u8g2.setDrawColor(1);
    } else {
        u8g2.drawFrame(btnX, btnY, btnW, btnH);
        u8g2.setFont(u8g2_font_8x13B_tr);
        int textWidth = u8g2.getStrWidth("LOC");
        int textX = btnX + (btnW - textWidth)/2;
        int textY = btnY + btnH - 3;
        u8g2.drawStr(textX, textY, "LOC");
    }

    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(btnX + btnW + 8, btnY + btnH - 3, "-> Exit");
    u8g2.sendBuffer();
}

void VitaPulseDisplay::showVitals(const char* label, float heartRate, float spo2, uint8_t signalStrength) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(2, 10, label);
    drawSignalBars(128-20, 12, signalStrength);

    char hrStr[10];
    snprintf(hrStr, sizeof(hrStr), "%.1f", heartRate);
    const char* unitStr = "bpm";

    u8g2.setFont(u8g2_font_fub30_tn);
    int16_t hrWidth = u8g2.getStrWidth(hrStr);
    u8g2.setFont(u8g2_font_6x12_tr);
    int16_t unitWidth = u8g2.getStrWidth(unitStr);

    int16_t totalWidth = hrWidth + 2 + unitWidth;
    int16_t startX = (128 - totalWidth)/2;
    int16_t hrY = 47;

    u8g2.setFont(u8g2_font_fub30_tn);
    u8g2.setCursor(startX, hrY);
    u8g2.print(hrStr);

    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(startX + hrWidth + 2, hrY, unitStr);

    char spo2Str[15];
    snprintf(spo2Str, sizeof(spo2Str), "SpO2:%.1f%%", spo2);
    int16_t spo2Width = u8g2.getStrWidth(spo2Str);
    u8g2.drawStr((128 - spo2Width)/2, 62, spo2Str);

    u8g2.sendBuffer();
}

void VitaPulseDisplay::updateConnecting(bool buttonSelected) {
    connButtonSelected = buttonSelected;

    // Draw connecting screen using current state
    showConnecting(connStrength, connDotCount + 1, connButtonSelected);

    // Update animation for next frame
    connStrength = (connStrength + 1) % 5;   // 0–4 bars
    connDotCount = (connDotCount + 1) % 12;  // 1–12 dots
}

// ---------------- showOLEDMessage ----------------
void VitaPulseDisplay::showOLEDMessage(const char* title, const char* body, const char* footer) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);

    // Title (~y=20)
    if (title && strlen(title) > 0) {
        int16_t x = (128 - u8g2.getStrWidth(title)) / 2;
        u8g2.setCursor(x, 20);
        u8g2.print(title);
    }

    // Body (~y=38)
    if (body && strlen(body) > 0) {
        int16_t x = (128 - u8g2.getStrWidth(body)) / 2;
        u8g2.setCursor(x, 38);
        u8g2.print(body);
    }

    // Footer (~y=56)
    if (footer && strlen(footer) > 0) {
        int16_t x = (128 - u8g2.getStrWidth(footer)) / 2;
        u8g2.setCursor(x, 56);
        u8g2.print(footer);
    }

    u8g2.sendBuffer();
}