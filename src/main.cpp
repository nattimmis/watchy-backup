/*
 * NEURAL OCEAN v11.0 - WALKIE TALKIE EDITION
 * Real time clock, LoRa pairing, voice messaging
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>
#include <driver/i2s.h>

// Pins
#define I2C_SDA     10
#define I2C_SCL     11
#define TFT_BL      45
#define TFT_CS      12
#define TFT_DC      38
#define TFT_MOSI    13
#define TFT_SCLK    18

// LoRa Pins (SX1262)
#define LORA_MOSI   1
#define LORA_MISO   3
#define LORA_SCK    2
#define LORA_CS     4
#define LORA_RST    5
#define LORA_BUSY   6
#define LORA_DIO1   7

// I2S Mic
#define I2S_WS      9
#define I2S_SD      46
#define I2S_SCK     8

// Buttons
#define BTN_1       0   // Main button

// Display
#define SCREEN_W    240
#define SCREEN_H    240

// AXP2101 & RTC
#define AXP2101_ADDR    0x34
#define AXP2101_LDO_EN  0x90
#define AXP2101_ALDO3   0x94
#define PCF8563_ADDR    0x51

// Colors
#define BLACK       0x0000
#define WHITE       0xFFFF
#define NEON_GREEN  0x07E0
#define NEON_CYAN   0x07FF
#define NEON_RED    0xF800
#define NEON_YELLOW 0xFFE0
#define NEON_PURPLE 0x780F
#define NEON_ORANGE 0xFD20
#define DIM_GREEN   0x02E0
#define DIM_CYAN    0x0410
#define DIM_RED     0x4000
#define GRID_DIM    0x0208

// Framebuffer
uint16_t fb[SCREEN_W * SCREEN_H];

// State
int currentFace = 0;
const int TOTAL_FACES = 14;
uint32_t frame = 0;

// RTC Time
int rtcHour = 21, rtcMin = 49, rtcSec = 0;
int rtcDay = 8, rtcMonth = 1, rtcYear = 26;
unsigned long lastRTCRead = 0;

// Biometrics
int heartRate = 72;
int hrv = 42;
int breathRate = 14;
int stress = 35;
float bodyTemp = 36.6;
int sleepScore = 78;
int socialCredit = 742;
int stepsToday = 8472;

// Entity tracking
struct Entity {
    uint8_t mac[6];
    int8_t rssi;
    float distanceM;
    float x, y;
    uint32_t lastSeen;
    bool isDrone;
};
#define MAX_ENTITIES 30
Entity entities[MAX_ENTITIES];
volatile int entityCount = 0;
int droneCount = 0;
float nearestEntityM = 99.9;
float avgDistanceM = 0;

// Drone OUI
const uint8_t DRONE_OUIS[][3] = {
    {0x60, 0x60, 0x1F}, {0x34, 0xD2, 0x62}, {0x48, 0x1C, 0xB9},
    {0xA0, 0x14, 0x3D}, {0x90, 0x03, 0xB7}, {0x00, 0x26, 0x7E},
    {0xD8, 0x96, 0x95}, {0x24, 0xD3, 0xF2}, {0xF4, 0xDD, 0x9E},
};
#define NUM_DRONE_OUIS 9

// LoRa Walkie Talkie
bool loraPaired = false;
uint32_t pairID = 0;
uint32_t partnerID = 0;
bool pairingMode = false;
unsigned long pairingStart = 0;
bool hasNewMessage = false;
bool isRecording = false;
bool isPlaying = false;
unsigned long recordStart = 0;
#define VOICE_BUF_SIZE 4000
int16_t voiceBuffer[VOICE_BUF_SIZE];
int voiceLen = 0;
int msgRSSI = 0;

// SPI for LoRa
SPIClass loraSPI(HSPI);

// ============= PMIC =============
void axpWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(AXP2101_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t axpRead(uint8_t reg) {
    Wire.beginTransmission(AXP2101_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)AXP2101_ADDR, 1);
    return Wire.available() ? Wire.read() : 0;
}

void initPMIC() {
    Wire.beginTransmission(AXP2101_ADDR);
    if (Wire.endTransmission() == 0) {
        axpWrite(AXP2101_ALDO3, 0x1C);
        uint8_t ldo = axpRead(AXP2101_LDO_EN);
        axpWrite(AXP2101_LDO_EN, ldo | 0x0F);
        delay(50);
    }
}

// ============= RTC =============
uint8_t bcd2dec(uint8_t bcd) { return (bcd >> 4) * 10 + (bcd & 0x0F); }
uint8_t dec2bcd(uint8_t dec) { return ((dec / 10) << 4) | (dec % 10); }

void rtcRead() {
    Wire.beginTransmission(PCF8563_ADDR);
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom((int)PCF8563_ADDR, 7);
    if (Wire.available() >= 7) {
        rtcSec = bcd2dec(Wire.read() & 0x7F);
        rtcMin = bcd2dec(Wire.read() & 0x7F);
        rtcHour = bcd2dec(Wire.read() & 0x3F);
        rtcDay = bcd2dec(Wire.read() & 0x3F);
        Wire.read(); // weekday
        rtcMonth = bcd2dec(Wire.read() & 0x1F);
        rtcYear = bcd2dec(Wire.read());
    }
}

void rtcWrite(int h, int m, int s, int d, int mo, int y) {
    Wire.beginTransmission(PCF8563_ADDR);
    Wire.write(0x02);
    Wire.write(dec2bcd(s));
    Wire.write(dec2bcd(m));
    Wire.write(dec2bcd(h));
    Wire.write(dec2bcd(d));
    Wire.write(0);
    Wire.write(dec2bcd(mo));
    Wire.write(dec2bcd(y));
    Wire.endTransmission();
}

// ============= Framebuffer =============
void fbClear(uint16_t color) {
    for (int i = 0; i < SCREEN_W * SCREEN_H; i++) fb[i] = color;
}

void fbPixel(int x, int y, uint16_t color) {
    if (x >= 0 && x < SCREEN_W && y >= 0 && y < SCREEN_H)
        fb[y * SCREEN_W + x] = color;
}

void fbRect(int x, int y, int w, int h, uint16_t color) {
    for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
            fbPixel(i, j, color);
}

void fbLine(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    while (1) {
        fbPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void fbCircle(int cx, int cy, int r, uint16_t color) {
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        fbPixel(cx + x, cy + y, color); fbPixel(cx + y, cy + x, color);
        fbPixel(cx - y, cy + x, color); fbPixel(cx - x, cy + y, color);
        fbPixel(cx - x, cy - y, color); fbPixel(cx - y, cy - x, color);
        fbPixel(cx + y, cy - x, color); fbPixel(cx + x, cy - y, color);
        y++;
        if (err < 0) err += 2 * y + 1;
        else { x--; err += 2 * (y - x) + 1; }
    }
}

void fbFillCircle(int cx, int cy, int r, uint16_t color) {
    for (int y = -r; y <= r; y++) {
        int w = sqrt(r * r - y * y);
        fbRect(cx - w, cy + y, w * 2 + 1, 1, color);
    }
}

// 8x8 font
const uint8_t font8x8[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // space
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // !
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, // "
    {0x36,0x7F,0x36,0x36,0x7F,0x36,0x00,0x00}, // #
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, // $
    {0x63,0x33,0x18,0x0C,0x66,0x63,0x00,0x00}, // %
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, // &
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, // '
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, // (
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, // )
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // *
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, // +
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, // ,
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, // .
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, // /
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, // 0
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, // 1
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, // 2
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, // 3
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, // 4
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, // 5
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, // 6
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // 7
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, // 8
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, // 9
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, // :
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06}, // ;
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, // <
    {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, // =
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // >
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, // ?
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, // @
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, // A
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, // B
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, // C
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, // D
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, // E
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, // F
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, // G
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, // H
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // I
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, // J
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, // K
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, // L
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, // M
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, // N
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, // O
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, // P
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, // Q
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, // R
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, // S
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // T
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, // U
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, // V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // W
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, // X
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, // Y
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, // Z
};

void fbChar(int x, int y, char c, uint16_t color, int size) {
    if (c < 32 || c > 90) c = 32;
    if (c >= 'a' && c <= 'z') c -= 32;
    int idx = c - 32;
    for (int row = 0; row < 8; row++) {
        uint8_t line = font8x8[idx][row];
        for (int col = 0; col < 8; col++) {
            if (line & (1 << col)) {
                if (size == 1) fbPixel(x + col, y + row, color);
                else fbRect(x + col * size, y + row * size, size, size, color);
            }
        }
    }
}

void fbText(int x, int y, const char* str, uint16_t color, int size) {
    while (*str) {
        fbChar(x, y, *str++, color, size);
        x += 8 * size + size;
    }
}

void fbTextCenter(int y, const char* str, uint16_t color, int size) {
    int len = strlen(str);
    int x = (SCREEN_W - len * (8 * size + size)) / 2;
    fbText(x, y, str, color, size);
}

// ============= Display =============
void sendCmd(uint8_t cmd) {
    digitalWrite(TFT_CS, LOW);
    digitalWrite(TFT_DC, LOW);
    SPI.transfer(cmd);
    digitalWrite(TFT_CS, HIGH);
}

void sendData(uint8_t data) {
    digitalWrite(TFT_CS, LOW);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(data);
    digitalWrite(TFT_CS, HIGH);
}

void pushFramebuffer() {
    sendCmd(0x2A);
    digitalWrite(TFT_CS, LOW); digitalWrite(TFT_DC, HIGH);
    SPI.transfer16(0); SPI.transfer16(239);
    digitalWrite(TFT_CS, HIGH);

    sendCmd(0x2B);
    digitalWrite(TFT_CS, LOW); digitalWrite(TFT_DC, HIGH);
    SPI.transfer16(0); SPI.transfer16(239);
    digitalWrite(TFT_CS, HIGH);

    sendCmd(0x2C);
    digitalWrite(TFT_CS, LOW);
    digitalWrite(TFT_DC, HIGH);
    for (int i = 0; i < SCREEN_W * SCREEN_H; i++) SPI.transfer16(fb[i]);
    digitalWrite(TFT_CS, HIGH);
}

void initDisplay() {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    pinMode(TFT_CS, OUTPUT);
    pinMode(TFT_DC, OUTPUT);
    digitalWrite(TFT_CS, HIGH);

    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
    SPI.setFrequency(80000000);

    sendCmd(0x01); delay(150);
    sendCmd(0x11); delay(120);
    sendCmd(0x3A); sendData(0x55);
    sendCmd(0x36); sendData(0x00);
    sendCmd(0x21);
    sendCmd(0x13); delay(10);
    sendCmd(0x29); delay(10);
}

// ============= LoRa SX1262 =============
void loraCmd(uint8_t cmd) {
    while (digitalRead(LORA_BUSY)) delay(1);
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(cmd);
    digitalWrite(LORA_CS, HIGH);
}

void loraWrite(uint8_t cmd, uint8_t* data, int len) {
    while (digitalRead(LORA_BUSY)) delay(1);
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(cmd);
    for (int i = 0; i < len; i++) loraSPI.transfer(data[i]);
    digitalWrite(LORA_CS, HIGH);
}

uint8_t loraRead(uint8_t cmd) {
    while (digitalRead(LORA_BUSY)) delay(1);
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(cmd);
    loraSPI.transfer(0x00);
    uint8_t r = loraSPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    return r;
}

void initLoRa() {
    pinMode(LORA_CS, OUTPUT);
    pinMode(LORA_RST, OUTPUT);
    pinMode(LORA_BUSY, INPUT);
    pinMode(LORA_DIO1, INPUT);

    digitalWrite(LORA_CS, HIGH);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(50);

    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    loraSPI.setFrequency(10000000);

    // Set standby
    uint8_t standby[] = {0x00};
    loraWrite(0x80, standby, 1);
    delay(10);

    // Set packet type LoRa
    uint8_t pkt[] = {0x01};
    loraWrite(0x8A, pkt, 1);

    // Set frequency 868 MHz
    uint32_t freq = (uint32_t)(868000000.0 * 33554432.0 / 32000000.0);
    uint8_t frf[] = {(uint8_t)(freq >> 24), (uint8_t)(freq >> 16), (uint8_t)(freq >> 8), (uint8_t)freq};
    loraWrite(0x86, frf, 4);

    // Set PA config
    uint8_t pa[] = {0x04, 0x07, 0x00, 0x01};
    loraWrite(0x95, pa, 4);

    // Set TX params
    uint8_t tx[] = {0x16, 0x04};
    loraWrite(0x8E, tx, 2);

    // Set modulation params SF7, BW 125k, CR 4/5
    uint8_t mod[] = {0x07, 0x04, 0x01, 0x00};
    loraWrite(0x8B, mod, 4);

    // Generate unique pair ID from MAC
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    pairID = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5];

    Serial.printf("LoRa init, ID: %08X\n", pairID);
}

void loraSend(uint8_t* data, int len) {
    // Write to buffer
    uint8_t buf[256];
    buf[0] = 0x00;
    memcpy(buf + 1, data, len);
    loraWrite(0x0E, buf, len + 1);

    // Set packet params
    uint8_t params[] = {0x00, 0x08, (uint8_t)len, 0x01, 0x00};
    loraWrite(0x8C, params, 5);

    // TX
    uint8_t txcmd[] = {0x00, 0x00, 0x00};
    loraWrite(0x83, txcmd, 3);

    delay(50);
}

int loraReceive(uint8_t* data, int maxLen) {
    // Set to RX
    uint8_t rxcmd[] = {0xFF, 0xFF, 0xFF};
    loraWrite(0x82, rxcmd, 3);

    delay(100);

    // Check IRQ
    uint8_t irq = loraRead(0x12);
    if (irq & 0x02) {  // RX done
        // Get buffer status
        while (digitalRead(LORA_BUSY)) delay(1);
        digitalWrite(LORA_CS, LOW);
        loraSPI.transfer(0x13);
        loraSPI.transfer(0x00);
        uint8_t len = loraSPI.transfer(0x00);
        uint8_t start = loraSPI.transfer(0x00);
        digitalWrite(LORA_CS, HIGH);

        if (len > 0 && len <= maxLen) {
            // Read buffer
            while (digitalRead(LORA_BUSY)) delay(1);
            digitalWrite(LORA_CS, LOW);
            loraSPI.transfer(0x1E);
            loraSPI.transfer(start);
            loraSPI.transfer(0x00);
            for (int i = 0; i < len; i++) data[i] = loraSPI.transfer(0x00);
            digitalWrite(LORA_CS, HIGH);

            // Get RSSI
            msgRSSI = -loraRead(0x14) / 2;

            // Clear IRQ
            uint8_t clr[] = {0xFF, 0xFF};
            loraWrite(0x02, clr, 2);

            return len;
        }
    }
    return 0;
}

// ============= Walkie Talkie Protocol =============
#define MSG_PAIR_REQ    0x01
#define MSG_PAIR_ACK    0x02
#define MSG_PING        0x03
#define MSG_PONG        0x04
#define MSG_VOICE       0x05

void sendPairRequest() {
    uint8_t pkt[8];
    pkt[0] = MSG_PAIR_REQ;
    memcpy(pkt + 1, &pairID, 4);
    loraSend(pkt, 5);
    Serial.println("Sent pair request");
}

void sendPairAck(uint32_t toID) {
    uint8_t pkt[8];
    pkt[0] = MSG_PAIR_ACK;
    memcpy(pkt + 1, &pairID, 4);
    loraSend(pkt, 5);
    partnerID = toID;
    loraPaired = true;
    Serial.printf("Paired with %08X\n", partnerID);
}

void sendPing() {
    if (!loraPaired) return;
    uint8_t pkt[8];
    pkt[0] = MSG_PING;
    memcpy(pkt + 1, &pairID, 4);
    loraSend(pkt, 5);
}

void sendVoice(int16_t* samples, int count) {
    if (!loraPaired) return;

    // Compress: take every 8th sample, convert to 8-bit
    int compLen = count / 8;
    if (compLen > 200) compLen = 200;

    uint8_t pkt[210];
    pkt[0] = MSG_VOICE;
    memcpy(pkt + 1, &pairID, 4);
    pkt[5] = compLen;

    for (int i = 0; i < compLen; i++) {
        int16_t s = samples[i * 8];
        pkt[6 + i] = (s >> 8) + 128;  // Convert to unsigned 8-bit
    }

    loraSend(pkt, 6 + compLen);
    Serial.printf("Sent voice %d bytes\n", compLen);
}

void processLoRa() {
    uint8_t buf[256];
    int len = loraReceive(buf, 256);

    if (len > 0) {
        uint8_t type = buf[0];
        uint32_t fromID;
        memcpy(&fromID, buf + 1, 4);

        switch (type) {
            case MSG_PAIR_REQ:
                if (pairingMode && !loraPaired) {
                    sendPairAck(fromID);
                    pairingMode = false;
                }
                break;

            case MSG_PAIR_ACK:
                if (pairingMode) {
                    partnerID = fromID;
                    loraPaired = true;
                    pairingMode = false;
                    Serial.printf("Paired! Partner: %08X\n", partnerID);
                }
                break;

            case MSG_PING:
                if (loraPaired && fromID == partnerID) {
                    uint8_t pong[8];
                    pong[0] = MSG_PONG;
                    memcpy(pong + 1, &pairID, 4);
                    loraSend(pong, 5);
                }
                break;

            case MSG_VOICE:
                if (loraPaired && fromID == partnerID) {
                    hasNewMessage = true;
                    int vlen = buf[5];
                    voiceLen = vlen * 8;
                    // Decompress
                    for (int i = 0; i < vlen && i * 8 < VOICE_BUF_SIZE; i++) {
                        int16_t s = ((int16_t)buf[6 + i] - 128) << 8;
                        for (int j = 0; j < 8 && i * 8 + j < VOICE_BUF_SIZE; j++) {
                            voiceBuffer[i * 8 + j] = s;
                        }
                    }
                    Serial.printf("Got voice %d samples\n", voiceLen);
                }
                break;
        }
    }
}

// ============= I2S Microphone =============
void initMic() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 8000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}

void recordVoice() {
    size_t bytes_read;
    voiceLen = 0;

    for (int i = 0; i < VOICE_BUF_SIZE && isRecording; i += 256) {
        int toRead = (VOICE_BUF_SIZE - i < 256) ? (VOICE_BUF_SIZE - i) : 256;
        i2s_read(I2S_NUM_0, &voiceBuffer[i], toRead * 2, &bytes_read, 100);
        voiceLen += bytes_read / 2;
    }
}

// ============= WiFi Sniffer =============
float rssiToDistance(int rssi) {
    float txPower = -45.0;
    float n = 2.7;
    return pow(10.0, (txPower - rssi) / (10.0 * n));
}

bool isDroneMAC(uint8_t* mac) {
    for (int i = 0; i < NUM_DRONE_OUIS; i++) {
        if (mac[0] == DRONE_OUIS[i][0] && mac[1] == DRONE_OUIS[i][1] && mac[2] == DRONE_OUIS[i][2])
            return true;
    }
    return false;
}

void IRAM_ATTR snifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
    int rssi = pkt->rx_ctrl.rssi;
    uint8_t *mac = pkt->payload + 10;

    int idx = -1;
    for (int i = 0; i < entityCount; i++) {
        if (memcmp(entities[i].mac, mac, 6) == 0) { idx = i; break; }
    }
    if (idx < 0 && entityCount < MAX_ENTITIES) {
        idx = entityCount++;
        memcpy(entities[idx].mac, mac, 6);
        entities[idx].isDrone = isDroneMAC(mac);
    }
    if (idx >= 0) {
        entities[idx].rssi = rssi;
        entities[idx].distanceM = rssiToDistance(rssi);
        entities[idx].lastSeen = millis();
        float angle = (mac[0] + mac[5]) * 0.1;
        float r = constrain(entities[idx].distanceM * 8, 10, 90);
        entities[idx].x = 120 + cos(angle) * r;
        entities[idx].y = 120 + sin(angle) * r;
    }
}

void updateEntityStats() {
    droneCount = 0;
    nearestEntityM = 99.9;
    float totalDist = 0;
    int validCount = 0;

    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen > 30000) continue;
        if (entities[i].isDrone) droneCount++;
        if (entities[i].distanceM < nearestEntityM) nearestEntityM = entities[i].distanceM;
        totalDist += entities[i].distanceM;
        validCount++;
    }
    avgDistanceM = validCount > 0 ? totalDist / validCount : 0;
}

void startSniffer() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(snifferCallback);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
}

// ============= WATCH FACES =============

void face_Clock() {
    fbClear(BLACK);

    // Update RTC every second
    if (millis() - lastRTCRead > 1000) {
        lastRTCRead = millis();
        rtcRead();
    }

    // Background rings
    for (int i = 0; i < 3; i++) {
        int r = 50 + ((frame + i * 30) % 80);
        uint8_t a = 80 - ((frame + i * 30) % 80);
        fbCircle(120, 100, r, (a >> 4) << 6);
    }

    // Big time
    char buf[32];
    sprintf(buf, "%02d:%02d", rtcHour, rtcMin);
    fbTextCenter(60, buf, NEON_GREEN, 4);

    // Seconds
    sprintf(buf, "%02d", rtcSec);
    fbTextCenter(110, buf, (rtcSec % 2) ? NEON_CYAN : DIM_CYAN, 2);

    // Date
    const char* months[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
    sprintf(buf, "%02d %s 20%02d", rtcDay, months[rtcMonth - 1], rtcYear);
    fbTextCenter(145, buf, NEON_PURPLE, 1);

    // Stats at bottom
    fbLine(0, 170, 240, 170, GRID_DIM);

    sprintf(buf, "%d BPM", heartRate);
    fbText(15, 180, buf, NEON_RED, 1);

    sprintf(buf, "%d NEAR", entityCount);
    fbText(100, 180, buf, NEON_GREEN, 1);

    sprintf(buf, "%.1fM", nearestEntityM < 50 ? nearestEntityM : 0);
    fbText(180, 180, buf, NEON_CYAN, 1);

    // Walkie status
    if (loraPaired) {
        fbFillCircle(220, 210, 8, NEON_GREEN);
        fbText(15, 205, "PAIRED", NEON_GREEN, 1);
    } else {
        fbCircle(220, 210, 8, DIM_GREEN);
        fbText(15, 205, "NO LINK", DIM_CYAN, 1);
    }

    if (hasNewMessage) {
        fbText(80, 205, "NEW MSG!", NEON_ORANGE, 1);
    }
}

void face_WalkieTalkie() {
    fbClear(BLACK);

    fbTextCenter(5, "WALKIE TALKIE", NEON_ORANGE, 2);

    char buf[32];

    if (!loraPaired) {
        // Not paired
        if (pairingMode) {
            // Searching animation
            int r = 30 + (frame % 40);
            fbCircle(120, 100, r, NEON_ORANGE);
            fbCircle(120, 100, r - 10, DIM_CYAN);

            fbTextCenter(90, "SEARCHING", NEON_ORANGE, 2);
            fbTextCenter(120, "HOLD TO PAIR", DIM_CYAN, 1);

            // Timeout after 30s
            if (millis() - pairingStart > 30000) {
                pairingMode = false;
            }
        } else {
            fbTextCenter(80, "NOT PAIRED", NEON_RED, 2);
            fbTextCenter(120, "HOLD BUTTON", DIM_CYAN, 1);
            fbTextCenter(140, "TO START PAIRING", DIM_CYAN, 1);
        }

        sprintf(buf, "MY ID: %08X", pairID);
        fbTextCenter(180, buf, DIM_GREEN, 1);

    } else {
        // Paired
        fbFillCircle(120, 70, 25, NEON_GREEN);
        fbTextCenter(60, "OK", BLACK, 2);

        sprintf(buf, "PARTNER");
        fbTextCenter(110, buf, DIM_CYAN, 1);
        sprintf(buf, "%08X", partnerID);
        fbTextCenter(125, buf, NEON_CYAN, 1);

        sprintf(buf, "SIGNAL %dDB", msgRSSI);
        fbTextCenter(150, buf, NEON_GREEN, 1);

        if (isRecording) {
            // Recording animation
            int pulse = sin(frame * 0.3) * 10 + 20;
            fbFillCircle(120, 190, pulse, NEON_RED);
            fbTextCenter(185, "REC", WHITE, 2);
        } else if (hasNewMessage) {
            fbFillCircle(120, 190, 25, NEON_ORANGE);
            fbTextCenter(185, "NEW", BLACK, 1);
            fbTextCenter(210, "TAP TO PLAY", NEON_ORANGE, 1);
        } else {
            fbCircle(120, 190, 20, DIM_GREEN);
            fbTextCenter(210, "HOLD TO RECORD", DIM_CYAN, 1);
        }
    }
}

void face_Vitals() {
    fbClear(BLACK);
    for (int x = 0; x < 240; x += 40) fbLine(x, 0, x, 240, GRID_DIM);
    for (int y = 0; y < 240; y += 40) fbLine(0, y, 240, y, GRID_DIM);

    fbTextCenter(5, "VITALS", NEON_CYAN, 2);

    float pulse = sin(frame * 0.15) * 0.2 + 1.0;
    fbFillCircle(52, 55, 10 * pulse, NEON_RED);
    fbFillCircle(68, 55, 10 * pulse, NEON_RED);

    heartRate = 68 + sin(frame * 0.1) * 4;
    char buf[32];
    sprintf(buf, "%d BPM", heartRate);
    fbText(25, 75, buf, NEON_RED, 2);

    hrv = 38 + sin(frame * 0.08) * 8;
    sprintf(buf, "HRV %dMS", hrv);
    fbText(130, 50, buf, NEON_GREEN, 1);

    for (int i = 0; i < 50; i++) {
        int h = 8 + sin(frame * 0.1 + i * 0.3) * hrv / 6;
        fbRect(135 + i * 2, 75 - h, 1, h, (i > 45) ? NEON_GREEN : DIM_GREEN);
    }

    for (int x = 10; x < 230; x++) {
        int phase = (x + frame * 3) % 70;
        int y = 115;
        if (phase > 20 && phase < 25) y = 115 - (phase - 20) * 10;
        else if (phase >= 25 && phase < 30) y = 65 + (phase - 25) * 14;
        else if (phase >= 30 && phase < 35) y = 135 - (phase - 30) * 4;
        fbPixel(x, y, (x > 210) ? NEON_GREEN : DIM_GREEN);
    }

    stress = 30 + sin(frame * 0.05) * 15;
    sprintf(buf, "STRESS %d%%", (int)stress);
    fbText(10, 140, buf, stress < 50 ? NEON_GREEN : NEON_YELLOW, 1);

    int sw = stress * 2;
    fbRect(10, 155, 200, 12, GRID_DIM);
    for (int x = 0; x < sw; x++) {
        uint16_t c = (x < 80) ? NEON_GREEN : (x < 140) ? NEON_YELLOW : NEON_RED;
        fbRect(10 + x, 156, 1, 10, c);
    }

    fbLine(0, 175, 240, 175, NEON_CYAN);
    sprintf(buf, "%.1fM NEAR", nearestEntityM < 50 ? nearestEntityM : 0);
    fbText(10, 185, buf, NEON_GREEN, 2);

    sprintf(buf, "%d ENTITIES", entityCount);
    fbText(10, 215, buf, DIM_CYAN, 1);
}

void face_Radar() {
    fbClear(BLACK);

    if (droneCount > 0) {
        fbTextCenter(5, "DRONE ALERT", NEON_RED, 2);
    } else {
        fbTextCenter(5, "RADAR", NEON_CYAN, 2);
    }

    int cx = 120, cy = 115;
    for (int r = 25; r <= 75; r += 25) fbCircle(cx, cy, r, GRID_DIM);
    fbText(cx + 20, cy - 5, "3M", DIM_GREEN, 1);
    fbText(cx + 45, cy - 5, "6M", DIM_GREEN, 1);
    fbText(cx + 70, cy - 5, "9M", DIM_GREEN, 1);

    fbLine(cx - 80, cy, cx + 80, cy, GRID_DIM);
    fbLine(cx, cy - 80, cx, cy + 80, GRID_DIM);

    float sweep = frame * 0.08;
    for (int i = 0; i < 15; i++) {
        float a = sweep - i * 0.04;
        int x2 = cx + cos(a) * 75;
        int y2 = cy + sin(a) * 75;
        uint8_t b = 200 - i * 12;
        fbLine(cx, cy, x2, y2, (b >> 3) << 6);
    }

    char buf[16];
    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen > 30000) continue;
        int ex = entities[i].x;
        int ey = entities[i].y;
        float dist = entities[i].distanceM;
        float p = sin(frame * 0.2 + i) * 2;

        if (entities[i].isDrone) {
            fbFillCircle(ex, ey, 6 + p, NEON_RED);
            fbLine(ex - 8, ey - 8, ex + 8, ey + 8, NEON_RED);
            fbLine(ex + 8, ey - 8, ex - 8, ey + 8, NEON_RED);
            sprintf(buf, "%.0fM", dist);
            fbText(ex + 10, ey - 4, buf, NEON_RED, 1);
        } else {
            fbFillCircle(ex, ey, 4 + p, NEON_GREEN);
        }
    }

    fbLine(0, 195, 240, 195, NEON_CYAN);
    sprintf(buf, "%d", entityCount);
    fbText(20, 205, buf, NEON_GREEN, 3);
    fbText(65, 215, "ENTITIES", DIM_GREEN, 1);

    sprintf(buf, "%.1fM", nearestEntityM < 50 ? nearestEntityM : 0);
    fbText(140, 205, buf, NEON_CYAN, 2);
    fbText(140, 225, "NEAREST", DIM_CYAN, 1);
}

void face_Bio() {
    fbClear(BLACK);
    for (int y = 0; y < 240; y += 4) fbLine(0, y, 240, y, 0x0041);

    fbTextCenter(5, "BIOMETRICS", NEON_CYAN, 2);

    char buf[32];
    bodyTemp = 36.5 + sin(frame * 0.02) * 0.3;
    uint16_t tCol = bodyTemp > 37.3 ? NEON_RED : NEON_GREEN;

    fbText(20, 40, "BODY TEMP", DIM_CYAN, 1);
    sprintf(buf, "%.1fC", bodyTemp);
    fbText(20, 55, buf, tCol, 3);

    fbRect(180, 40, 20, 70, GRID_DIM);
    fbFillCircle(190, 120, 15, GRID_DIM);
    int mercH = (bodyTemp - 35.0) * 30;
    fbRect(183, 110 - mercH, 14, mercH, tCol);
    fbFillCircle(190, 120, 12, tCol);

    int spo2 = 97 + sin(frame * 0.07) * 1.5;
    fbText(20, 100, "OXYGEN SPO2", DIM_CYAN, 1);
    sprintf(buf, "%d%%", spo2);
    fbText(20, 115, buf, NEON_CYAN, 3);

    for (int i = 0; i < 3; i++) {
        float a = frame * 0.08 + i * 2;
        int ox = 180 + cos(a) * 20;
        int oy = 115 + sin(a) * 15;
        fbFillCircle(ox - 5, oy, 5, NEON_CYAN);
        fbFillCircle(ox + 5, oy, 5, NEON_CYAN);
    }

    breathRate = 14 + sin(frame * 0.04) * 2;
    fbText(20, 160, "BREATH RATE", DIM_CYAN, 1);
    sprintf(buf, "%d /MIN", breathRate);
    fbText(20, 175, buf, NEON_GREEN, 2);

    float bp = sin(frame * 0.06);
    int br = 20 + bp * 15;
    fbCircle(190, 175, br, NEON_GREEN);
    fbCircle(190, 175, br + 3, DIM_GREEN);

    fbLine(0, 205, 240, 205, NEON_CYAN);
    sprintf(buf, "%d NEARBY %.1fM", entityCount, nearestEntityM < 50 ? nearestEntityM : 0);
    fbTextCenter(215, buf, NEON_GREEN, 1);
}

void face_Drone() {
    fbClear(BLACK);

    uint16_t titleCol = (droneCount > 0 && (frame / 8) % 2) ? NEON_RED : NEON_CYAN;
    fbTextCenter(5, "DRONE SCAN", titleCol, 2);

    int cx = 120, cy = 100;
    char buf[32];

    for (int r = 20; r <= 60; r += 20) fbCircle(cx, cy, r, GRID_DIM);

    float sweep = frame * 0.1;
    fbLine(cx, cy, cx + cos(sweep) * 65, cy + sin(sweep) * 65, NEON_CYAN);

    int droneIdx = 0;
    for (int i = 0; i < entityCount && droneIdx < 4; i++) {
        if (!entities[i].isDrone) continue;
        if (millis() - entities[i].lastSeen > 30000) continue;

        int dx = cx + cos(droneIdx * 1.5 + frame * 0.03) * (25 + droneIdx * 15);
        int dy = cy + sin(droneIdx * 1.5 + frame * 0.03) * (20 + droneIdx * 10);

        fbFillCircle(dx, dy, 8, NEON_RED);
        fbLine(dx - 12, dy - 12, dx + 12, dy + 12, NEON_RED);
        fbLine(dx + 12, dy - 12, dx - 12, dy + 12, NEON_RED);

        sprintf(buf, "%.1fM", entities[i].distanceM);
        fbText(dx + 15, dy - 4, buf, NEON_RED, 1);
        droneIdx++;
    }

    fbLine(0, 165, 240, 165, NEON_CYAN);

    sprintf(buf, "%d", droneCount);
    fbText(30, 175, buf, droneCount > 0 ? NEON_RED : NEON_GREEN, 4);
    fbText(80, 195, "DRONES", droneCount > 0 ? DIM_RED : DIM_GREEN, 1);

    if (droneCount > 0) {
        float nearDrone = 99;
        for (int i = 0; i < entityCount; i++) {
            if (entities[i].isDrone && entities[i].distanceM < nearDrone)
                nearDrone = entities[i].distanceM;
        }
        sprintf(buf, "%.1fM", nearDrone);
        fbText(140, 175, buf, NEON_RED, 2);
        fbText(140, 200, "NEAREST", DIM_RED, 1);
    } else {
        fbText(140, 180, "CLEAR", NEON_GREEN, 2);
    }
}

void face_Profile() {
    fbClear(BLACK);
    fbTextCenter(5, "PROFILE", NEON_PURPLE, 2);

    const char* types[] = {"INTJ", "INFJ", "INTP", "INFP", "ENTJ", "ENFJ", "ENTP", "ENFP"};
    int ti = (hrv + entityCount) % 8;
    fbTextCenter(30, types[ti], NEON_CYAN, 4);

    const char* traits[] = {"INTRO", "INTUIT", "THINK", "JUDGE"};
    int vals[] = {65, 72, 45, 58};

    for (int i = 0; i < 4; i++) {
        int y = 75 + i * 25;
        int v = vals[i] + sin(frame * 0.05 + i) * 5;
        fbText(5, y, traits[i], DIM_CYAN, 1);
        fbRect(60, y, 120, 14, GRID_DIM);
        fbRect(60, y, v * 1.2, 14, NEON_CYAN);
        char buf[8];
        sprintf(buf, "%d", v);
        fbText(190, y, buf, NEON_GREEN, 1);
    }

    socialCredit = 720 + sin(frame * 0.03) * 30;
    uint16_t scCol = socialCredit > 700 ? NEON_GREEN : NEON_YELLOW;

    fbText(10, 180, "SOCIAL SCORE", DIM_GREEN, 1);
    char buf[16];
    sprintf(buf, "%d", socialCredit);
    fbText(10, 195, buf, scCol, 3);

    for (int a = 0; a <= 180; a += 2) {
        float rad = (180 + a) * DEG_TO_RAD;
        int x = 180 + cos(rad) * 40;
        int y = 210 + sin(rad) * 40;
        int scoreA = (socialCredit - 300) * 180 / 600;
        fbPixel(x, y, (a < scoreA) ? scCol : GRID_DIM);
    }
}

void face_System() {
    fbClear(BLACK);
    fbTextCenter(5, "SYSTEM", NEON_CYAN, 2);

    char buf[32];

    int batt = 75 + sin(frame * 0.02) * 5;
    uint16_t bc = batt > 50 ? NEON_GREEN : batt > 20 ? NEON_YELLOW : NEON_RED;

    fbRect(60, 35, 100, 40, WHITE);
    fbRect(160, 47, 8, 16, WHITE);
    fbRect(62, 37, batt * 0.96, 36, bc);
    sprintf(buf, "%d%%", batt);
    fbText(85, 80, buf, bc, 2);

    int cpu = 30 + sin(frame * 0.08) * 20;
    int ram = 45 + cos(frame * 0.06) * 15;

    fbText(10, 115, "CPU", DIM_CYAN, 1);
    fbRect(50, 115, 140, 12, GRID_DIM);
    fbRect(50, 115, cpu * 1.4, 12, NEON_GREEN);
    sprintf(buf, "%d%%", cpu);
    fbText(200, 115, buf, NEON_GREEN, 1);

    fbText(10, 135, "RAM", DIM_CYAN, 1);
    fbRect(50, 135, 140, 12, GRID_DIM);
    fbRect(50, 135, ram * 1.4, 12, NEON_CYAN);
    sprintf(buf, "%d%%", ram);
    fbText(200, 135, buf, NEON_CYAN, 1);

    float temp = 42 + sin(frame * 0.05) * 5;
    sprintf(buf, "TEMP %.1fC", temp);
    fbText(10, 160, buf, temp > 50 ? NEON_RED : NEON_GREEN, 1);

    // LoRa status
    fbLine(0, 180, 240, 180, NEON_CYAN);

    if (loraPaired) {
        sprintf(buf, "LORA: PAIRED");
        fbText(10, 190, buf, NEON_GREEN, 1);
        sprintf(buf, "%08X", partnerID);
        fbText(10, 205, buf, DIM_GREEN, 1);
    } else {
        fbText(10, 190, "LORA: STANDBY", DIM_CYAN, 1);
    }

    sprintf(buf, "%d ENTITIES", entityCount);
    fbText(130, 190, buf, NEON_CYAN, 1);

    fbTextCenter(225, "V11.0 WALKIE", DIM_CYAN, 1);
}

void face_Neural() {
    fbClear(BLACK);
    fbTextCenter(5, "NEURAL MESH", NEON_GREEN, 2);

    for (int i = 0; i < 12; i++) {
        float a = i * 30 * DEG_TO_RAD + frame * 0.015;
        float r = 55 + sin(frame * 0.02 + i) * 15;
        int nx = 120 + cos(a) * r;
        int ny = 115 + sin(a) * r;

        for (int j = i + 1; j < 12; j++) {
            float a2 = j * 30 * DEG_TO_RAD + frame * 0.015;
            float r2 = 55 + sin(frame * 0.02 + j) * 15;
            int nx2 = 120 + cos(a2) * r2;
            int ny2 = 120 + sin(a2) * r2;

            if (abs(i - j) <= 3) {
                fbLine(nx, ny, nx2, ny2, GRID_DIM);
                float p = fmod(frame * 0.04 + i * 0.15, 1.0);
                int px = nx + (nx2 - nx) * p;
                int py = ny + (ny2 - ny) * p;
                fbFillCircle(px, py, 2, NEON_GREEN);
            }
        }

        float pulse = sin(frame * 0.15 + i * 0.3) * 2;
        fbFillCircle(nx, ny, 5 + pulse, NEON_CYAN);
    }

    fbFillCircle(120, 115, 10, NEON_GREEN);

    char buf[32];
    fbLine(0, 195, 240, 195, NEON_CYAN);
    sprintf(buf, "%d NODES", 12 + entityCount);
    fbText(15, 205, buf, NEON_CYAN, 1);
    sprintf(buf, "%.1fM RANGE", avgDistanceM);
    fbText(120, 205, buf, NEON_GREEN, 1);
}

void face_Spectrum() {
    fbClear(BLACK);
    fbTextCenter(5, "SPECTRUM", NEON_PURPLE, 2);

    for (int i = 0; i < 20; i++) {
        float h = sin(frame * 0.12 + i * 0.4) * 40 + cos(frame * 0.08 + i * 0.6) * 25 + 65;
        h = constrain(h, 15, 130);
        int x = 12 + i * 11;
        int y = 190 - h;
        uint16_t c = (h > 100) ? NEON_RED : (h > 70) ? NEON_YELLOW : NEON_GREEN;
        fbRect(x, y, 9, h, c);
        fbRect(x, y - 3, 9, 2, WHITE);
    }

    float bass = sin(frame * 0.1) * 0.5 + 0.5;
    int br = 15 + bass * 20;
    fbFillCircle(120, 45, br, ((int)(bass * 15) << 11));
    fbCircle(120, 45, br + 2, NEON_RED);

    fbText(10, 200, "20HZ", DIM_CYAN, 1);
    fbText(100, 200, "1KHZ", DIM_CYAN, 1);
    fbText(185, 200, "20KHZ", DIM_CYAN, 1);

    char buf[32];
    sprintf(buf, "%d NEARBY %.1fM", entityCount, avgDistanceM);
    fbTextCenter(220, buf, NEON_GREEN, 1);
}

void face_Location() {
    fbClear(BLACK);

    int off = frame % 25;
    for (int x = off; x < 240; x += 25) fbLine(x, 0, x, 240, GRID_DIM);
    for (int y = off; y < 240; y += 25) fbLine(0, y, 240, y, GRID_DIM);

    fbTextCenter(5, "LOCATION", NEON_CYAN, 2);

    int cx = 120, cy = 100;
    for (int i = 0; i < 3; i++) {
        int r = 15 + i * 20 + sin(frame * 0.1 + i) * 5;
        fbCircle(cx, cy, r, DIM_CYAN);
    }

    fbFillCircle(cx, cy - 12, 12, NEON_RED);
    fbFillCircle(cx, cy - 12, 6, WHITE);
    for (int i = 0; i < 12; i++) {
        fbLine(cx - 6 + i/2, cy, cx, cy + 18, NEON_RED);
        fbLine(cx + 6 - i/2, cy, cx, cy + 18, NEON_RED);
    }

    float lat = 47.3769 + sin(frame * 0.01) * 0.001;
    float lon = 8.5417 + cos(frame * 0.01) * 0.001;

    char buf[32];
    fbText(20, 145, "LAT", DIM_CYAN, 1);
    sprintf(buf, "%.4f", lat);
    fbText(60, 145, buf, NEON_GREEN, 2);

    fbText(20, 175, "LON", DIM_CYAN, 1);
    sprintf(buf, "%.4f", lon);
    fbText(60, 175, buf, NEON_GREEN, 2);

    sprintf(buf, "%d NEARBY", entityCount);
    fbText(160, 175, buf, NEON_CYAN, 1);

    sprintf(buf, "%.1fM NEAREST", nearestEntityM < 50 ? nearestEntityM : 0);
    fbTextCenter(210, buf, NEON_GREEN, 1);
}

void face_Presence() {
    fbClear(BLACK);
    fbTextCenter(5, "PRESENCE", NEON_CYAN, 2);

    for (int r = 30; r < 180; r += 2) {
        int spread = (180 - r) / 3;
        uint8_t b = (180 - r);
        uint16_t c = (b >> 4) << 6;
        for (int x = 120 - spread; x <= 120 + spread; x++) {
            int y = 200 - (180 - r);
            if (y > 30) fbPixel(x, y, c);
        }
    }

    char buf[16];
    int maxP = (entityCount < 6) ? entityCount : 6;
    for (int i = 0; i < maxP; i++) {
        float ex = 120 + sin(i * 1.3 + frame * 0.02) * (25 + i * 12);
        float ey = 50 + i * 25;
        float dist = entities[i].distanceM;
        float p = sin(frame * 0.12 + i) * 2;

        fbFillCircle(ex, ey, 6 + p, NEON_GREEN);
        fbRect(ex - 5, ey + 8, 10, 16, NEON_GREEN);

        sprintf(buf, "%.1fM", dist);
        fbText(ex + 12, ey - 2, buf, NEON_CYAN, 1);
    }

    fbFillCircle(120, 215, 12, NEON_YELLOW);
    fbFillCircle(120, 215, 8, WHITE);

    sprintf(buf, "%d DETECTED", entityCount);
    fbTextCenter(230, buf, NEON_GREEN, 1);
}

void face_About() {
    fbClear(BLACK);

    for (int y = (frame * 2) % 5; y < 240; y += 5) {
        fbLine(0, y, 240, y, 0x0841);
    }

    fbTextCenter(30, "NEURAL", NEON_GREEN, 3);
    fbTextCenter(65, "OCEAN", NEON_CYAN, 3);
    fbTextCenter(105, "V11.0", NEON_PURPLE, 2);

    const char* feat[] = {"WALKIE TALKIE", "LORA PAIRING", "REAL TIME CLOCK", "DRONE DETECT"};
    int fi = (frame / 45) % 4;
    fbTextCenter(140, feat[fi], NEON_ORANGE, 1);

    for (int i = 0; i < 12; i++) {
        float a = (i * 30 + frame * 2) * DEG_TO_RAD;
        int px = 120 + cos(a) * 100;
        int py = 140 + sin(a) * 100;
        fbFillCircle(px, py, 3, (i % 2) ? NEON_CYAN : NEON_GREEN);
    }

    fbTextCenter(175, "T-WATCH S3", DIM_GREEN, 1);
    fbTextCenter(195, "WALKIE TALKIE ED", DIM_CYAN, 1);

    char buf[32];
    sprintf(buf, "%02d:%02d  %02d/%02d", rtcHour, rtcMin, rtcDay, rtcMonth);
    fbTextCenter(220, buf, NEON_GREEN, 1);
}

void drawCurrentFace() {
    switch (currentFace) {
        case 0: face_Clock(); break;
        case 1: face_WalkieTalkie(); break;
        case 2: face_Vitals(); break;
        case 3: face_Radar(); break;
        case 4: face_Bio(); break;
        case 5: face_Drone(); break;
        case 6: face_Profile(); break;
        case 7: face_System(); break;
        case 8: face_Neural(); break;
        case 9: face_Spectrum(); break;
        case 10: face_Location(); break;
        case 11: face_Presence(); break;
        case 12: face_About(); break;
    }
    pushFramebuffer();
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    initPMIC();
    delay(100);

    // Set initial time
    rtcWrite(21, 49, 0, 8, 1, 26);

    initDisplay();
    initLoRa();
    initMic();
    startSniffer();

    pinMode(BTN_1, INPUT_PULLUP);

    // Splash
    fbClear(BLACK);
    fbTextCenter(50, "NEURAL", NEON_GREEN, 3);
    fbTextCenter(85, "OCEAN", NEON_CYAN, 3);
    fbTextCenter(125, "V11.0", NEON_PURPLE, 2);
    fbTextCenter(160, "WALKIE TALKIE", NEON_ORANGE, 1);
    pushFramebuffer();
    delay(2000);

    Serial.println("Neural Ocean v11.0 - Ready");
    Serial.printf("My ID: %08X\n", pairID);
}

unsigned long lastFrame = 0;
unsigned long lastFaceChange = 0;
unsigned long btnPressStart = 0;
bool btnWasPressed = false;

void loop() {
    // Handle button
    bool btnPressed = (digitalRead(BTN_1) == LOW);

    if (btnPressed && !btnWasPressed) {
        btnPressStart = millis();
        btnWasPressed = true;
    }

    if (!btnPressed && btnWasPressed) {
        unsigned long pressDuration = millis() - btnPressStart;
        btnWasPressed = false;

        if (pressDuration < 500) {
            // Short press - next face
            currentFace = (currentFace + 1) % TOTAL_FACES;
            lastFaceChange = millis();
        } else if (pressDuration < 2000) {
            // Medium press - play message or record
            if (currentFace == 1) {  // Walkie face
                if (hasNewMessage) {
                    // Play message (simplified - just clear flag)
                    hasNewMessage = false;
                    Serial.println("Playing message");
                }
            }
        }
    }

    // Long press - pairing or recording
    if (btnPressed && btnWasPressed && (millis() - btnPressStart > 2000)) {
        if (currentFace == 1 && !loraPaired && !pairingMode) {
            // Start pairing
            pairingMode = true;
            pairingStart = millis();
            sendPairRequest();
            Serial.println("Pairing mode started");
        } else if (currentFace == 1 && loraPaired && !isRecording) {
            // Start recording
            isRecording = true;
            recordStart = millis();
            Serial.println("Recording...");
        }
    }

    // Stop recording after 3 seconds
    if (isRecording && (millis() - recordStart > 3000 || !btnPressed)) {
        isRecording = false;
        recordVoice();
        sendVoice(voiceBuffer, voiceLen);
        Serial.println("Sent voice clip");
    }

    // Process LoRa
    processLoRa();

    // 25 FPS
    if (millis() - lastFrame > 40) {
        lastFrame = millis();
        frame++;

        updateEntityStats();
        drawCurrentFace();
    }

    // Auto cycle (except on walkie face)
    if (currentFace != 1 && millis() - lastFaceChange > 6000) {
        lastFaceChange = millis();
        currentFace = (currentFace + 1) % TOTAL_FACES;
    }

    // Channel hop for WiFi
    static unsigned long lastHop = 0;
    static int ch = 1;
    if (millis() - lastHop > 3000) {
        ch = (ch % 13) + 1;
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        lastHop = millis();
    }

    // Pairing beacon
    if (pairingMode && millis() - pairingStart < 30000) {
        static unsigned long lastBeacon = 0;
        if (millis() - lastBeacon > 1000) {
            sendPairRequest();
            lastBeacon = millis();
        }
    }

    // Serial control
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'n') { currentFace = (currentFace + 1) % TOTAL_FACES; lastFaceChange = millis(); }
        if (c == 'p') { currentFace = (currentFace + TOTAL_FACES - 1) % TOTAL_FACES; lastFaceChange = millis(); }
        if (c == 'w') { currentFace = 1; lastFaceChange = millis(); }  // Go to walkie
    }
}
