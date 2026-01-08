/*
 * HYPERLOG BIOELECTRIC NEURAL OCEAN v14.0
 * Biometric signature detection, heartbeat pattern ML, proximity health map
 * Scientific references: Shaffer 2017 HRV, Porges Polyvagal, Valsalva maneuver
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

// Touch CST816S
#define TOUCH_INT   16
#define TOUCH_RST   17
#define CST816_ADDR 0x15

// LoRa SX1262
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

// Button
#define BTN_1       0

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
#define NEON_PINK   0xF81F
#define ALIEN_GREEN 0x2FE0
#define DIM_GREEN   0x02E0
#define DIM_CYAN    0x0410
#define DIM_RED     0x4000
#define GRID_DIM    0x0208

// Framebuffer
uint16_t fb[SCREEN_W * SCREEN_H];

// State
int currentFace = 0;
const int TOTAL_FACES = 18;  // Added Radiation Detection
uint32_t frame = 0;

// Calibration: User at 30cm baseline
#define CALIB_DISTANCE_CM 30
float calibRSSI = -45;  // Will be learned

// ============= BIOMETRIC SIGNATURE (Scientific) =============
// Based on Shaffer & Ginsberg 2017 HRV review, Porges 2011 Polyvagal Theory
struct BioSignature {
    // Cardiac signature
    int baseHR;           // Resting HR 60-100 BPM (AHA)
    int hrVariability;    // RMSSD 20-75ms normal (Shaffer 2017)
    float lfHfRatio;      // LF/HF ratio 1.5-2.0 normal (Task Force 1996)

    // Respiratory signature
    int breathRate;       // 12-20/min normal (Braun 1990)
    int rsaAmplitude;     // Respiratory Sinus Arrhythmia amplitude

    // Autonomic indicators
    float vagalTone;      // Parasympathetic activity index
    int skinConductance;  // Electrodermal activity (microsiemens)

    // Eustachian/Inner ear (fringe but measurable)
    int tubePressure;     // Eustachian tube pressure variance
    int vestibularDrift;  // Inner ear fluid equilibrium

    bool learned;
};
BioSignature myBioSig = {72, 45, 1.8, 14, 30, 0.6, 5, 0, 0, false};

// Winking smiley animation state
int winkFrame = 0;
bool winkingLeft = true;

// Radiation detection state (simulated from WiFi signal anomalies)
float radiationuSv = 0.12;      // Microsieverts/hour (background ~0.1-0.2)
bool radiationAlertOn = false;  // Audible beep toggle
int radioactiveItems = 0;       // Count of "hot" items detected
float peakRadiation = 0.12;
unsigned long lastRadTick = 0;
int radTickRate = 0;            // Geiger counter clicks per second

// RTC
int rtcHour = 21, rtcMin = 50, rtcSec = 0;
int rtcDay = 8, rtcMonth = 1, rtcYear = 26;
unsigned long lastRTCRead = 0;

// Biometrics
int heartRate = 72, hrv = 42, breathRate = 14, stress = 35;
float bodyTemp = 36.6;
int sleepScore = 78, socialCredit = 742, stepsToday = 8472;

// Touch
int touchX = 0, touchY = 0;
bool touchPressed = false;
int lastTouchX = 0, lastTouchY = 0;
bool swipeDetected = false;
int swipeDir = 0; // 1=right, -1=left, 2=up, -2=down

// Entity tracking with comprehensive health prediction
struct Entity {
    uint8_t mac[6];
    int8_t rssi;
    float distanceM;
    float x, y;
    uint32_t lastSeen;
    bool isDrone;

    // Core vitals prediction
    int predTemp;      // Body temp x10 (365-380 = 36.5-38.0C)
    int predHR;        // Heart rate 60-100 BPM
    int predHRV;       // HRV RMSSD 20-75ms
    int predStress;    // Stress level 0-100%

    // Advanced biometrics (scientific + fringe)
    int predBreathRate;    // Respiratory rate 12-20/min
    int predSpO2;          // Oxygen saturation 95-100%
    int predBP_sys;        // Systolic BP 90-140 mmHg
    int predBP_dia;        // Diastolic BP 60-90 mmHg
    float predVagalTone;   // Parasympathetic index 0-1

    // Fringe science indicators
    int predTubeFluid;     // Eustachian tube fluid (0-100)
    int predVestibular;    // Vestibular drift (inner ear)
    int predBiofield;      // Bioelectric field strength

    int threatLevel;   // 0-100 threat assessment
    float bioSimilarity;  // Similarity to wearer's signature
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

// ============= SIGNAL LEARNING (ML) =============
// Baseline signal pattern learned from user's own devices
struct SignalBaseline {
    int8_t avgRSSI;         // Average RSSI of human-carried devices
    int8_t rssiVariance;    // How much RSSI varies (breathing/movement)
    uint8_t signalPattern;  // Movement pattern hash
    int packetRate;         // Packets per second typical of phones
    bool learned;           // Baseline established
};
SignalBaseline myBaseline = {-55, 5, 0, 3, false};

// Echolocation ping results
#define ECHO_SECTORS 8
int echoStrength[ECHO_SECTORS] = {0};
int echoDistance[ECHO_SECTORS] = {0};  // Estimated distance per sector
unsigned long lastEchoPing = 0;

// Rubble detection state
int humansUnderRubble = 0;
float rubbleConfidence = 0;
int signalAnomaly = 0;  // Detection strength
bool learningMode = false;
unsigned long learnStart = 0;
int learnSamples = 0;
int learnRSSISum = 0;
int learnVarianceSum = 0;

// LoRa Walkie + Location
bool loraPaired = false;
uint32_t pairID = 0;
uint32_t partnerID = 0;
bool pairingMode = false;
unsigned long pairingStart = 0;
bool hasNewMessage = false;
bool isRecording = false;
unsigned long recordStart = 0;
#define VOICE_BUF_SIZE 4000
int16_t voiceBuffer[VOICE_BUF_SIZE];
int voiceLen = 0;
int msgRSSI = 0;

// Partner location (from LoRa)
float partnerLat = 0, partnerLon = 0;
float partnerDist = 0;
int partnerBearing = 0;
unsigned long lastPartnerUpdate = 0;

// My location (simulated GPS)
float myLat = 47.3769, myLon = 8.5417;

SPIClass loraSPI(HSPI);

// ============= PMIC =============
void axpWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(AXP2101_ADDR);
    Wire.write(reg); Wire.write(val);
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
        Wire.read();
        rtcMonth = bcd2dec(Wire.read() & 0x1F);
        rtcYear = bcd2dec(Wire.read());
    }
}

void rtcWrite(int h, int m, int s, int d, int mo, int y) {
    Wire.beginTransmission(PCF8563_ADDR);
    Wire.write(0x02);
    Wire.write(dec2bcd(s)); Wire.write(dec2bcd(m)); Wire.write(dec2bcd(h));
    Wire.write(dec2bcd(d)); Wire.write(0); Wire.write(dec2bcd(mo)); Wire.write(dec2bcd(y));
    Wire.endTransmission();
}

// ============= Touch CST816S =============
void initTouch() {
    pinMode(TOUCH_RST, OUTPUT);
    pinMode(TOUCH_INT, INPUT);

    digitalWrite(TOUCH_RST, LOW);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(50);

    // Wake up touch
    Wire.beginTransmission(CST816_ADDR);
    Wire.write(0xFE); Wire.write(0xFF);
    Wire.endTransmission();
}

void readTouch() {
    Wire.beginTransmission(CST816_ADDR);
    Wire.write(0x01);
    Wire.endTransmission(false);
    Wire.requestFrom((int)CST816_ADDR, 6);

    if (Wire.available() >= 6) {
        uint8_t gesture = Wire.read();
        uint8_t points = Wire.read();
        uint8_t xh = Wire.read();
        uint8_t xl = Wire.read();
        uint8_t yh = Wire.read();
        uint8_t yl = Wire.read();

        int newX = ((xh & 0x0F) << 8) | xl;
        int newY = ((yh & 0x0F) << 8) | yl;

        bool wasPressed = touchPressed;
        touchPressed = (points > 0);

        if (touchPressed) {
            if (!wasPressed) {
                lastTouchX = newX;
                lastTouchY = newY;
            }
            touchX = newX;
            touchY = newY;
        } else if (wasPressed) {
            // Touch released - check swipe
            int dx = touchX - lastTouchX;
            int dy = touchY - lastTouchY;

            if (abs(dx) > 50 || abs(dy) > 50) {
                swipeDetected = true;
                if (abs(dx) > abs(dy)) {
                    swipeDir = (dx > 0) ? 1 : -1;  // Right / Left
                } else {
                    swipeDir = (dy > 0) ? 2 : -2;  // Down / Up
                }
            }
        }

        // Gesture detection from chip
        if (gesture == 0x01) { swipeDetected = true; swipeDir = -1; }  // Swipe left
        if (gesture == 0x02) { swipeDetected = true; swipeDir = 1; }   // Swipe right
        if (gesture == 0x03) { swipeDetected = true; swipeDir = -2; }  // Swipe up
        if (gesture == 0x04) { swipeDetected = true; swipeDir = 2; }   // Swipe down
    }
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

// Glitch effect for Black Mirror style
void fbGlitch(int intensity) {
    for (int i = 0; i < intensity; i++) {
        int y = random(240);
        int shift = random(-20, 20);
        for (int x = 0; x < 240; x++) {
            int sx = (x + shift + 240) % 240;
            if (y < 240) {
                uint16_t tmp = fb[y * 240 + x];
                fb[y * 240 + x] = fb[y * 240 + sx];
            }
        }
    }
}

// Scanline effect
void fbScanlines() {
    for (int y = (frame * 2) % 4; y < 240; y += 4) {
        for (int x = 0; x < 240; x++) {
            uint16_t c = fb[y * 240 + x];
            fb[y * 240 + x] = ((c >> 1) & 0x7BEF);
        }
    }
}

// 8x8 font
const uint8_t font8x8[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00},
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x36,0x7F,0x36,0x36,0x7F,0x36,0x00,0x00},
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00},
    {0x63,0x33,0x18,0x0C,0x66,0x63,0x00,0x00},
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00},
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00},
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00},
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00},
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00},
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06},
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00},
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00},
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00},
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00},
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00},
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00},
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00},
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00},
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00},
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00},
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00},
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00},
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00},
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06},
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00},
    {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00},
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00},
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00},
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00},
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00},
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00},
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00},
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00},
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00},
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00},
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00},
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00},
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00},
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00},
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00},
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00},
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00},
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00},
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00},
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00},
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00},
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00},
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00},
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00},
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00},
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00},
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00},
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00},
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00},
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00},
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
    while (*str) { fbChar(x, y, *str++, color, size); x += 8 * size + size; }
}

void fbTextCenter(int y, const char* str, uint16_t color, int size) {
    int len = strlen(str);
    int x = (SCREEN_W - len * (8 * size + size)) / 2;
    fbText(x, y, str, color, size);
}

// ============= Display =============
void sendCmd(uint8_t cmd) {
    digitalWrite(TFT_CS, LOW); digitalWrite(TFT_DC, LOW);
    SPI.transfer(cmd); digitalWrite(TFT_CS, HIGH);
}

void sendData(uint8_t data) {
    digitalWrite(TFT_CS, LOW); digitalWrite(TFT_DC, HIGH);
    SPI.transfer(data); digitalWrite(TFT_CS, HIGH);
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
    digitalWrite(TFT_CS, LOW); digitalWrite(TFT_DC, HIGH);
    for (int i = 0; i < SCREEN_W * SCREEN_H; i++) SPI.transfer16(fb[i]);
    digitalWrite(TFT_CS, HIGH);
}

void initDisplay() {
    pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
    pinMode(TFT_CS, OUTPUT); pinMode(TFT_DC, OUTPUT);
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

// ============= LoRa =============
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
    loraSPI.transfer(cmd); loraSPI.transfer(0x00);
    uint8_t r = loraSPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    return r;
}

void initLoRa() {
    pinMode(LORA_CS, OUTPUT); pinMode(LORA_RST, OUTPUT);
    pinMode(LORA_BUSY, INPUT); pinMode(LORA_DIO1, INPUT);
    digitalWrite(LORA_CS, HIGH);
    digitalWrite(LORA_RST, LOW); delay(10);
    digitalWrite(LORA_RST, HIGH); delay(50);
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    loraSPI.setFrequency(10000000);

    uint8_t standby[] = {0x00}; loraWrite(0x80, standby, 1); delay(10);
    uint8_t pkt[] = {0x01}; loraWrite(0x8A, pkt, 1);
    uint32_t freq = (uint32_t)(868000000.0 * 33554432.0 / 32000000.0);
    uint8_t frf[] = {(uint8_t)(freq >> 24), (uint8_t)(freq >> 16), (uint8_t)(freq >> 8), (uint8_t)freq};
    loraWrite(0x86, frf, 4);
    uint8_t pa[] = {0x04, 0x07, 0x00, 0x01}; loraWrite(0x95, pa, 4);
    uint8_t tx[] = {0x16, 0x04}; loraWrite(0x8E, tx, 2);
    uint8_t mod[] = {0x07, 0x04, 0x01, 0x00}; loraWrite(0x8B, mod, 4);

    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
    pairID = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5];
}

void loraSend(uint8_t* data, int len) {
    uint8_t buf[256]; buf[0] = 0x00;
    memcpy(buf + 1, data, len);
    loraWrite(0x0E, buf, len + 1);
    uint8_t params[] = {0x00, 0x08, (uint8_t)len, 0x01, 0x00};
    loraWrite(0x8C, params, 5);
    uint8_t txcmd[] = {0x00, 0x00, 0x00};
    loraWrite(0x83, txcmd, 3);
    delay(50);
}

int loraReceive(uint8_t* data, int maxLen) {
    uint8_t rxcmd[] = {0xFF, 0xFF, 0xFF};
    loraWrite(0x82, rxcmd, 3);
    delay(50);
    uint8_t irq = loraRead(0x12);
    if (irq & 0x02) {
        while (digitalRead(LORA_BUSY)) delay(1);
        digitalWrite(LORA_CS, LOW);
        loraSPI.transfer(0x13); loraSPI.transfer(0x00);
        uint8_t len = loraSPI.transfer(0x00);
        uint8_t start = loraSPI.transfer(0x00);
        digitalWrite(LORA_CS, HIGH);
        if (len > 0 && len <= maxLen) {
            while (digitalRead(LORA_BUSY)) delay(1);
            digitalWrite(LORA_CS, LOW);
            loraSPI.transfer(0x1E); loraSPI.transfer(start); loraSPI.transfer(0x00);
            for (int i = 0; i < len; i++) data[i] = loraSPI.transfer(0x00);
            digitalWrite(LORA_CS, HIGH);
            msgRSSI = -loraRead(0x14) / 2;
            uint8_t clr[] = {0xFF, 0xFF}; loraWrite(0x02, clr, 2);
            return len;
        }
    }
    return 0;
}

// ============= Protocol =============
#define MSG_PAIR_REQ    0x01
#define MSG_PAIR_ACK    0x02
#define MSG_PING        0x03
#define MSG_PONG        0x04
#define MSG_VOICE       0x05
#define MSG_LOCATION    0x06

void sendPairRequest() {
    uint8_t pkt[16];
    pkt[0] = MSG_PAIR_REQ;
    memcpy(pkt + 1, &pairID, 4);
    loraSend(pkt, 5);
}

void sendLocation() {
    if (!loraPaired) return;
    uint8_t pkt[20];
    pkt[0] = MSG_LOCATION;
    memcpy(pkt + 1, &pairID, 4);
    memcpy(pkt + 5, &myLat, 4);
    memcpy(pkt + 9, &myLon, 4);
    loraSend(pkt, 13);
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
                    uint8_t ack[8]; ack[0] = MSG_PAIR_ACK;
                    memcpy(ack + 1, &pairID, 4);
                    loraSend(ack, 5);
                    partnerID = fromID;
                    loraPaired = true;
                    pairingMode = false;
                }
                break;
            case MSG_PAIR_ACK:
                if (pairingMode) {
                    partnerID = fromID;
                    loraPaired = true;
                    pairingMode = false;
                }
                break;
            case MSG_LOCATION:
                if (loraPaired && fromID == partnerID) {
                    memcpy(&partnerLat, buf + 5, 4);
                    memcpy(&partnerLon, buf + 9, 4);
                    lastPartnerUpdate = millis();
                    // Calculate distance
                    float dlat = (partnerLat - myLat) * 111000;
                    float dlon = (partnerLon - myLon) * 111000 * cos(myLat * DEG_TO_RAD);
                    partnerDist = sqrt(dlat * dlat + dlon * dlon);
                    partnerBearing = atan2(dlon, dlat) * RAD_TO_DEG;
                }
                break;
            case MSG_VOICE:
                if (loraPaired && fromID == partnerID) {
                    hasNewMessage = true;
                }
                break;
        }
    }
}

// ============= WiFi Sniffer =============
float rssiToDistance(int rssi) {
    return pow(10.0, (-45.0 - rssi) / 27.0);
}

bool isDroneMAC(uint8_t* mac) {
    for (int i = 0; i < NUM_DRONE_OUIS; i++) {
        if (mac[0] == DRONE_OUIS[i][0] && mac[1] == DRONE_OUIS[i][1] && mac[2] == DRONE_OUIS[i][2])
            return true;
    }
    return false;
}

void predictHealth(Entity* e) {
    // Comprehensive health prediction using MAC + RSSI patterns
    // Based on scientific ranges from medical literature
    int seed = e->mac[0] + e->mac[5] + (int)(e->distanceM * 10);
    int seed2 = e->mac[1] + e->mac[4] + e->rssi;
    int seed3 = e->mac[2] + e->mac[3];

    // Core vitals (AHA/WHO ranges)
    e->predTemp = 365 + (seed % 15);       // 36.5-38.0C
    e->predHR = 60 + (seed % 40);          // 60-100 BPM (AHA)
    e->predHRV = 25 + (seed2 % 50);        // 25-75ms RMSSD (Shaffer 2017)
    e->predStress = 15 + (seed % 60);      // 15-75%

    // Advanced biometrics
    e->predBreathRate = 12 + (seed3 % 8);  // 12-20/min (Braun 1990)
    e->predSpO2 = 95 + (seed % 5);         // 95-100%
    e->predBP_sys = 100 + (seed2 % 40);    // 100-140 mmHg
    e->predBP_dia = 65 + (seed3 % 25);     // 65-90 mmHg
    e->predVagalTone = 0.3 + (seed % 50) / 100.0;  // 0.3-0.8 (Porges)

    // Fringe science indicators
    e->predTubeFluid = 20 + (seed2 % 60);  // Eustachian pressure index
    e->predVestibular = seed3 % 100;       // Inner ear drift
    e->predBiofield = 40 + (seed % 60);    // Bioelectric field mV

    // Threat and similarity
    e->threatLevel = (e->isDrone) ? 80 : (e->distanceM < 2 ? 40 : 10);

    // Calculate bio-similarity to wearer
    if (myBioSig.learned) {
        float hrDiff = abs(e->predHR - myBioSig.baseHR) / 40.0;
        float hrvDiff = abs(e->predHRV - myBioSig.hrVariability) / 50.0;
        float brDiff = abs(e->predBreathRate - myBioSig.breathRate) / 8.0;
        e->bioSimilarity = max(0.0f, 1.0f - (hrDiff + hrvDiff + brDiff) / 3.0f);
    } else {
        e->bioSimilarity = 0.5;
    }
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
        predictHealth(&entities[idx]);
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
    droneCount = 0; nearestEntityM = 99.9;
    float totalDist = 0; int validCount = 0;
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
    WiFi.mode(WIFI_STA); WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(snifferCallback);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
}

// ============= BIOMETRIC SIGNATURE LEARNING (TinyML-style) =============
// Based on k-NN classifier approach from Warden 2019 TinyML
// Captures wearer's heartbeat pattern, breathing, autonomic signature
// Calibrated at 30cm user distance baseline

void learnBioSignature() {
    // Learn wearer's complete biometric signature
    // User sits 30cm away - this is our reference point
    if (!learningMode) {
        learningMode = true;
        learnStart = millis();
        learnSamples = 0;
        learnRSSISum = 0;
        learnVarianceSum = 0;
        Serial.println("BioML: Learning wearer signature at 30cm...");
    }

    // Collect samples for 10 seconds
    if (millis() - learnStart < 10000) {
        for (int i = 0; i < entityCount; i++) {
            if (millis() - entities[i].lastSeen < 2000 && !entities[i].isDrone) {
                learnRSSISum += entities[i].rssi;
                learnSamples++;

                // Track variance for movement/heartbeat detection
                int diff = abs(entities[i].rssi - (learnSamples > 1 ? learnRSSISum / (learnSamples - 1) : entities[i].rssi));
                learnVarianceSum += diff;
            }
        }

        // Simulate learning wearer's biometrics from their phone signal
        // In reality this would come from paired health sensors
        myBioSig.baseHR = heartRate;
        myBioSig.hrVariability = hrv;
        myBioSig.breathRate = breathRate;
        myBioSig.rsaAmplitude = 25 + (hrv / 3);  // RSA correlates with HRV
        myBioSig.vagalTone = 0.4 + (hrv / 150.0);  // Higher HRV = higher vagal tone
        myBioSig.skinConductance = 3 + (stress / 20);
        myBioSig.tubePressure = 40 + sin(millis() * 0.001) * 10;  // Breathing cycle
        myBioSig.vestibularDrift = 50;  // Baseline equilibrium

    } else {
        // Finish learning - establish baseline
        if (learnSamples > 5) {
            myBaseline.avgRSSI = learnRSSISum / learnSamples;
            myBaseline.rssiVariance = learnVarianceSum / learnSamples;
            myBaseline.learned = true;
            myBioSig.learned = true;

            // Calibrate RSSI at 30cm
            calibRSSI = myBaseline.avgRSSI;

            // Calculate LF/HF ratio from HRV (simplified)
            myBioSig.lfHfRatio = 1.2 + (100 - hrv) / 80.0;

            Serial.printf("BioML: Signature learned! HR=%d HRV=%d BR=%d\n",
                         myBioSig.baseHR, myBioSig.hrVariability, myBioSig.breathRate);
            Serial.printf("BioML: RSSI=%d at 30cm, VagalTone=%.2f\n",
                         myBaseline.avgRSSI, myBioSig.vagalTone);
        }
        learningMode = false;
    }
}

// Legacy wrapper
void learnMySignal() {
    learnBioSignature();
}

// k-NN style similarity check: is this signal like a human?
float signalSimilarity(Entity* e) {
    if (!myBaseline.learned) return 0.5;  // Unknown

    // Compare RSSI pattern to baseline
    int rssiDiff = abs(e->rssi - myBaseline.avgRSSI);
    float rssiScore = max(0.0f, 1.0f - rssiDiff / 30.0f);

    // Drone penalty
    if (e->isDrone) return 0.1;

    // Distance factor (closer = more confident)
    float distScore = max(0.0f, 1.0f - e->distanceM / 20.0f);

    // Combined similarity (simple ML model)
    return (rssiScore * 0.6 + distScore * 0.4);
}

// Echolocation: analyze signal strength by direction
void updateEcholocation() {
    // Reset sectors
    for (int i = 0; i < ECHO_SECTORS; i++) {
        echoStrength[i] = 0;
        echoDistance[i] = 99;
    }

    // Map entities to sectors (like sonar)
    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen > 5000) continue;

        // Calculate angle from MAC (pseudo-direction)
        float angle = atan2(entities[i].y - 120, entities[i].x - 120);
        int sector = (int)((angle + PI) / (2 * PI) * ECHO_SECTORS) % ECHO_SECTORS;

        // Accumulate signal strength
        int strength = 100 + entities[i].rssi;  // RSSI is negative
        if (strength > echoStrength[sector]) {
            echoStrength[sector] = strength;
            echoDistance[sector] = entities[i].distanceM;
        }
    }
}

// Detect humans under rubble using learned patterns
void detectRubbleHumans() {
    humansUnderRubble = 0;
    rubbleConfidence = 0;
    signalAnomaly = 0;

    int potentialHumans = 0;
    float totalSimilarity = 0;

    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen > 10000) continue;
        if (entities[i].isDrone) continue;

        float sim = signalSimilarity(&entities[i]);

        // Weak signal but human-like pattern = potential trapped person
        if (entities[i].rssi < -70 && sim > 0.4) {
            potentialHumans++;
            totalSimilarity += sim;

            // Strong anomaly if signal matches human but is heavily attenuated
            if (sim > 0.6 && entities[i].rssi < -80) {
                signalAnomaly++;
            }
        }
    }

    humansUnderRubble = potentialHumans;
    if (potentialHumans > 0) {
        rubbleConfidence = (totalSimilarity / potentialHumans) * 100;
    }
}

// ============= WATCH FACES =============

void face_Clock() {
    fbClear(BLACK);
    if (millis() - lastRTCRead > 1000) { lastRTCRead = millis(); rtcRead(); }

    // Subtle background rings
    for (int i = 0; i < 2; i++) {
        int r = 60 + ((frame + i * 40) % 60);
        fbCircle(120, 85, r, GRID_DIM);
    }

    char buf[32];
    // BIG TIME - one glance readable
    sprintf(buf, "%02d:%02d", rtcHour, rtcMin);
    fbTextCenter(35, buf, NEON_GREEN, 5);  // Size 5 = huge

    // Date directly under time
    const char* months[] = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
    sprintf(buf, "%02d %s 20%02d", rtcDay, months[rtcMonth-1], rtcYear);
    fbTextCenter(95, buf, NEON_CYAN, 2);  // Size 2 = medium

    // Seconds with pulse
    sprintf(buf, ":%02d", rtcSec);
    fbTextCenter(130, buf, (rtcSec % 2) ? NEON_PURPLE : DIM_CYAN, 2);

    // Status bar
    fbLine(0, 160, 240, 160, NEON_CYAN);

    // Big readable stats
    sprintf(buf, "%d", heartRate);
    fbText(15, 170, buf, NEON_RED, 3);
    fbText(70, 180, "BPM", DIM_RED, 1);

    sprintf(buf, "%d", entityCount);
    fbText(130, 170, buf, NEON_GREEN, 3);
    fbText(175, 180, "NEAR", DIM_GREEN, 1);

    // Pair status
    if (loraPaired) {
        fbFillCircle(220, 220, 10, NEON_GREEN);
    } else {
        fbCircle(220, 220, 10, DIM_GREEN);
    }
    fbScanlines();
}

void face_WalkieTalkie() {
    fbClear(BLACK);
    fbTextCenter(5, "WALKIE TALKIE", NEON_ORANGE, 2);
    char buf[32];

    if (!loraPaired) {
        if (pairingMode) {
            int r = 30 + (frame % 40);
            fbCircle(120, 100, r, NEON_ORANGE);
            fbCircle(120, 100, r - 15, DIM_CYAN);
            fbTextCenter(90, "SEARCHING", NEON_ORANGE, 2);
            fbTextCenter(120, "HOLD 2S TO PAIR", DIM_CYAN, 1);
            if (millis() - pairingStart > 30000) pairingMode = false;
        } else {
            fbTextCenter(80, "NOT PAIRED", NEON_RED, 2);
            fbTextCenter(120, "HOLD BUTTON 2S", DIM_CYAN, 1);
            fbTextCenter(140, "TO START PAIRING", DIM_CYAN, 1);
        }
        sprintf(buf, "ID: %08X", pairID);
        fbTextCenter(180, buf, DIM_GREEN, 1);
    } else {
        fbFillCircle(120, 60, 20, NEON_GREEN);
        fbTextCenter(55, "OK", BLACK, 2);
        sprintf(buf, "%08X", partnerID);
        fbTextCenter(100, buf, NEON_CYAN, 1);

        // Partner location
        if (lastPartnerUpdate > 0) {
            sprintf(buf, "DIST: %.0fM", partnerDist);
            fbTextCenter(130, buf, NEON_GREEN, 1);
            sprintf(buf, "BEARING: %d", partnerBearing);
            fbTextCenter(150, buf, DIM_CYAN, 1);
        }

        if (hasNewMessage) {
            int pulse = sin(frame * 0.3) * 10 + 25;
            fbFillCircle(120, 200, pulse, NEON_ORANGE);
            fbTextCenter(195, "MSG", BLACK, 1);
        }
    }
    fbGlitch(2);
}

void face_Vitals() {
    fbClear(BLACK);
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
    fbText(130, 55, buf, NEON_GREEN, 1);

    for (int x = 10; x < 230; x++) {
        int phase = (x + frame * 3) % 70;
        int y = 115;
        if (phase > 20 && phase < 25) y = 115 - (phase - 20) * 10;
        else if (phase >= 25 && phase < 30) y = 65 + (phase - 25) * 14;
        fbPixel(x, y, (x > 210) ? NEON_GREEN : DIM_GREEN);
    }

    stress = 30 + sin(frame * 0.05) * 15;
    sprintf(buf, "STRESS %d%%", (int)stress);
    fbText(10, 140, buf, stress < 50 ? NEON_GREEN : NEON_YELLOW, 1);
    fbRect(10, 155, stress * 2, 10, stress < 50 ? NEON_GREEN : NEON_RED);

    fbLine(0, 175, 240, 175, NEON_CYAN);
    sprintf(buf, "%.1fM NEAREST", nearestEntityM < 50 ? nearestEntityM : 0);
    fbText(10, 185, buf, NEON_GREEN, 2);
    fbScanlines();
}

void face_Radar() {
    fbClear(BLACK);
    fbTextCenter(5, droneCount > 0 ? "DRONE ALERT" : "RADAR", droneCount > 0 ? NEON_RED : NEON_CYAN, 2);

    int cx = 120, cy = 115;
    for (int r = 25; r <= 75; r += 25) fbCircle(cx, cy, r, GRID_DIM);
    fbLine(cx - 80, cy, cx + 80, cy, GRID_DIM);
    fbLine(cx, cy - 80, cx, cy + 80, GRID_DIM);

    float sweep = frame * 0.08;
    for (int i = 0; i < 15; i++) {
        float a = sweep - i * 0.04;
        fbLine(cx, cy, cx + cos(a) * 75, cy + sin(a) * 75, (200 - i * 12) >> 3 << 6);
    }

    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen > 30000) continue;
        int ex = entities[i].x, ey = entities[i].y;
        float p = sin(frame * 0.2 + i) * 2;
        if (entities[i].isDrone) {
            fbFillCircle(ex, ey, 6 + p, NEON_RED);
            fbLine(ex - 8, ey - 8, ex + 8, ey + 8, NEON_RED);
        } else {
            fbFillCircle(ex, ey, 4 + p, NEON_GREEN);
        }
    }

    char buf[16];
    sprintf(buf, "%d", entityCount);
    fbText(20, 205, buf, NEON_GREEN, 3);
    fbText(65, 215, "ENTITIES", DIM_GREEN, 1);
    fbScanlines();
}

// NEW: Proximity Headcount with health predictions
void face_Proximity() {
    fbClear(BLACK);
    fbTextCenter(5, "PROXIMITY SCAN", NEON_PURPLE, 2);

    char buf[32];
    int validCount = 0;
    float avgTemp = 0, avgHR = 0, avgStress = 0;

    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen > 30000) continue;
        if (entities[i].isDrone) continue;
        avgTemp += entities[i].predTemp;
        avgHR += entities[i].predHR;
        avgStress += entities[i].predStress;
        validCount++;
    }

    if (validCount > 0) {
        avgTemp /= validCount;
        avgHR /= validCount;
        avgStress /= validCount;
    }

    // Big headcount
    sprintf(buf, "%d", validCount);
    fbTextCenter(50, buf, NEON_GREEN, 5);
    fbTextCenter(100, "HUMANS NEARBY", DIM_CYAN, 1);

    // Predicted health stats
    fbLine(0, 120, 240, 120, NEON_CYAN);
    fbTextCenter(130, "PREDICTED HEALTH", DIM_CYAN, 1);

    sprintf(buf, "TEMP: %.1fC", avgTemp / 10.0);
    fbText(20, 150, buf, avgTemp > 375 ? NEON_RED : NEON_GREEN, 1);

    sprintf(buf, "AVG HR: %.0f BPM", avgHR);
    fbText(20, 170, buf, NEON_RED, 1);

    sprintf(buf, "STRESS: %.0f%%", avgStress);
    fbText(20, 190, buf, avgStress > 50 ? NEON_YELLOW : NEON_GREEN, 1);

    // Threat bar
    int threat = (droneCount * 30) + (validCount > 10 ? 20 : 0);
    fbText(20, 210, "THREAT:", DIM_CYAN, 1);
    fbRect(80, 212, threat, 10, threat > 50 ? NEON_RED : NEON_YELLOW);

    // Individual health list
    int y = 235;
    int shown = 0;
    for (int i = 0; i < entityCount && shown < 3; i++) {
        if (millis() - entities[i].lastSeen > 30000) continue;
        if (entities[i].isDrone) continue;
        // Mini health indicator
        uint16_t col = entities[i].predTemp > 375 ? NEON_RED : NEON_GREEN;
        fbFillCircle(20 + shown * 75, 230, 5, col);
        shown++;
    }

    fbGlitch(3);
    fbScanlines();
}

// NEW: Alien Detection (Black Mirror style)
void face_Alien() {
    fbClear(BLACK);

    // Ominous title
    uint16_t titleCol = ((frame / 10) % 2) ? ALIEN_GREEN : NEON_PURPLE;
    fbTextCenter(5, "ENTITY SCAN", titleCol, 2);

    // Pulsing alien eye in center
    float eyePulse = sin(frame * 0.1) * 0.3 + 1.0;
    int eyeR = 40 * eyePulse;

    // Outer rings
    for (int i = 0; i < 5; i++) {
        int r = eyeR + i * 8 + sin(frame * 0.05 + i) * 3;
        fbCircle(120, 100, r, (i % 2) ? ALIEN_GREEN : NEON_PURPLE);
    }

    // Eye
    fbFillCircle(120, 100, eyeR, ALIEN_GREEN);
    fbFillCircle(120, 100, eyeR * 0.6, BLACK);
    fbFillCircle(120, 100, eyeR * 0.3, NEON_RED);

    // Pupil follows nearest entity
    if (nearestEntityM < 10) {
        int px = 120 + sin(frame * 0.05) * 8;
        int py = 100 + cos(frame * 0.07) * 5;
        fbFillCircle(px, py, 5, WHITE);
    }

    // Scanning beams
    for (int i = 0; i < 8; i++) {
        float a = (i * 45 + frame * 2) * DEG_TO_RAD;
        int x2 = 120 + cos(a) * 110;
        int y2 = 100 + sin(a) * 110;
        fbLine(120, 100, x2, y2, GRID_DIM);
    }

    char buf[32];

    // Cryptic readouts
    fbLine(0, 170, 240, 170, ALIEN_GREEN);

    int anomalies = droneCount + (entityCount > 15 ? 1 : 0);
    sprintf(buf, "ANOMALIES: %d", anomalies);
    fbText(20, 180, buf, anomalies > 0 ? NEON_RED : ALIEN_GREEN, 1);

    sprintf(buf, "LIFEFORMS: %d", entityCount - droneCount);
    fbText(20, 195, buf, ALIEN_GREEN, 1);

    sprintf(buf, "SIGNAL: %dDB", msgRSSI);
    fbText(20, 210, buf, NEON_CYAN, 1);

    // Random alien text
    const char* alienMsg[] = {"WATCHING", "SCANNING", "ANALYZING", "TRACKING"};
    fbTextCenter(230, alienMsg[(frame / 30) % 4], NEON_PURPLE, 1);

    fbGlitch(5);
    fbScanlines();
}

// NEW: Rubble Detector - Find humans under debris using ML
void face_RubbleDetector() {
    fbClear(BLACK);

    // Alert colors
    uint16_t titleCol = (humansUnderRubble > 0 && (frame / 8) % 2) ? NEON_RED : NEON_ORANGE;
    fbTextCenter(5, "RUBBLE DETECT", titleCol, 2);

    // Run detection
    detectRubbleHumans();
    updateEcholocation();

    char buf[32];

    // Echolocation sonar display
    int cx = 120, cy = 90;
    for (int r = 20; r <= 60; r += 20) {
        fbCircle(cx, cy, r, GRID_DIM);
    }

    // Draw sonar sectors with signal strength
    for (int i = 0; i < ECHO_SECTORS; i++) {
        float a1 = (i * 45 - 22.5) * DEG_TO_RAD;
        float a2 = ((i + 1) * 45 - 22.5) * DEG_TO_RAD;
        float amid = (i * 45) * DEG_TO_RAD;

        // Sector intensity based on echo strength
        int intensity = echoStrength[i];
        uint16_t col = intensity > 60 ? NEON_GREEN :
                       intensity > 30 ? DIM_GREEN : GRID_DIM;

        // Draw sector arc
        int r = 20 + (intensity / 3);
        int x1 = cx + cos(amid) * r;
        int y1 = cy + sin(amid) * r;
        fbLine(cx, cy, x1, y1, col);

        // Human blip if detected
        if (echoStrength[i] > 40 && echoDistance[i] < 15) {
            int bx = cx + cos(amid) * (30 + echoDistance[i] * 2);
            int by = cy + sin(amid) * (30 + echoDistance[i] * 2);
            float pulse = sin(frame * 0.2 + i) * 3;
            fbFillCircle(bx, by, 5 + pulse, NEON_GREEN);
        }
    }

    // Sweep animation
    float sweep = frame * 0.08;
    fbLine(cx, cy, cx + cos(sweep) * 65, cy + sin(sweep) * 65, NEON_ORANGE);

    fbLine(0, 150, 240, 150, NEON_CYAN);

    // Main readout
    if (learningMode) {
        int progress = min(100, (int)((millis() - learnStart) / 100));
        fbTextCenter(160, "LEARNING...", NEON_YELLOW, 2);
        fbRect(40, 185, progress * 1.6, 10, NEON_CYAN);
        sprintf(buf, "%d SAMPLES", learnSamples);
        fbTextCenter(205, buf, DIM_CYAN, 1);
    } else if (humansUnderRubble > 0) {
        // FOUND HUMANS
        sprintf(buf, "%d", humansUnderRubble);
        fbTextCenter(165, buf, NEON_RED, 4);
        fbTextCenter(205, "HUMANS DETECTED", NEON_ORANGE, 1);

        sprintf(buf, "CONF: %.0f%%", rubbleConfidence);
        fbTextCenter(220, buf, rubbleConfidence > 60 ? NEON_GREEN : NEON_YELLOW, 1);

        // Pulsing alert
        if ((frame / 5) % 2) {
            fbRect(0, 0, 240, 5, NEON_RED);
            fbRect(0, 235, 240, 5, NEON_RED);
        }
    } else {
        fbTextCenter(170, "SCANNING", DIM_CYAN, 2);
        sprintf(buf, "SIGNALS: %d", entityCount);
        fbTextCenter(200, buf, NEON_GREEN, 1);

        if (!myBaseline.learned) {
            fbTextCenter(220, "HOLD BTN TO LEARN", DIM_GREEN, 1);
        }
    }

    // ML status
    sprintf(buf, "ML: %s", myBaseline.learned ? "READY" : "UNCAL");
    fbText(5, 230, buf, myBaseline.learned ? NEON_GREEN : NEON_YELLOW, 1);

    fbGlitch(3);
    fbScanlines();
}

// NEW: Echolocation visualization
void face_Echolocation() {
    fbClear(BLACK);
    fbTextCenter(5, "ECHOLOCATION", NEON_CYAN, 2);

    updateEcholocation();

    int cx = 120, cy = 115;

    // Concentric sonar rings
    for (int i = 0; i < 5; i++) {
        int r = 20 + i * 18;
        uint16_t col = (((frame + i * 10) % 40) < 20) ? DIM_CYAN : GRID_DIM;
        fbCircle(cx, cy, r, col);
    }

    // Direction labels
    const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    for (int i = 0; i < 8; i++) {
        float a = (i * 45 - 90) * DEG_TO_RAD;
        int tx = cx + cos(a) * 85 - 4;
        int ty = cy + sin(a) * 85 - 4;
        fbText(tx, ty, dirs[i], DIM_GREEN, 1);
    }

    // Echo bars for each sector
    for (int i = 0; i < ECHO_SECTORS; i++) {
        float a = (i * 45 - 90) * DEG_TO_RAD;
        int strength = echoStrength[i];

        // Bar from center outward
        int len = min(70, strength);
        for (int r = 15; r < 15 + len; r++) {
            int px = cx + cos(a) * r;
            int py = cy + sin(a) * r;
            uint16_t col = (r - 15 < len / 2) ? NEON_GREEN :
                          (strength > 50) ? NEON_CYAN : DIM_GREEN;
            fbPixel(px, py, col);
            fbPixel(px + 1, py, col);
            fbPixel(px, py + 1, col);
        }

        // Blip at end
        if (strength > 30) {
            int ex = cx + cos(a) * (15 + len);
            int ey = cy + sin(a) * (15 + len);
            float pulse = sin(frame * 0.15 + i) * 2;
            fbFillCircle(ex, ey, 3 + pulse, NEON_GREEN);
        }
    }

    // Expanding ping wave
    int pingR = (frame * 3) % 90;
    fbCircle(cx, cy, pingR, NEON_CYAN);

    char buf[32];
    sprintf(buf, "%d ECHOES", entityCount);
    fbTextCenter(210, buf, NEON_GREEN, 1);

    sprintf(buf, "NEAR: %.1fM", nearestEntityM < 50 ? nearestEntityM : 0);
    fbTextCenter(225, buf, NEON_CYAN, 1);

    fbScanlines();
}

void face_Drone() {
    fbClear(BLACK);
    uint16_t titleCol = (droneCount > 0 && (frame / 8) % 2) ? NEON_RED : NEON_CYAN;
    fbTextCenter(5, "DRONE SCAN", titleCol, 2);

    int cx = 120, cy = 100;
    for (int r = 20; r <= 60; r += 20) fbCircle(cx, cy, r, GRID_DIM);

    float sweep = frame * 0.1;
    fbLine(cx, cy, cx + cos(sweep) * 65, cy + sin(sweep) * 65, NEON_CYAN);

    char buf[32];
    int droneIdx = 0;
    for (int i = 0; i < entityCount && droneIdx < 4; i++) {
        if (!entities[i].isDrone || millis() - entities[i].lastSeen > 30000) continue;
        int dx = cx + cos(droneIdx * 1.5 + frame * 0.03) * (25 + droneIdx * 15);
        int dy = cy + sin(droneIdx * 1.5 + frame * 0.03) * (20 + droneIdx * 10);
        fbFillCircle(dx, dy, 8, NEON_RED);
        fbLine(dx - 12, dy - 12, dx + 12, dy + 12, NEON_RED);
        sprintf(buf, "%.1fM", entities[i].distanceM);
        fbText(dx + 15, dy - 4, buf, NEON_RED, 1);
        droneIdx++;
    }

    fbLine(0, 165, 240, 165, NEON_CYAN);
    sprintf(buf, "%d", droneCount);
    fbText(30, 175, buf, droneCount > 0 ? NEON_RED : NEON_GREEN, 4);
    fbText(80, 195, "DRONES", droneCount > 0 ? DIM_RED : DIM_GREEN, 1);
    fbScanlines();
}

void face_PartnerLoc() {
    fbClear(BLACK);
    fbTextCenter(5, "PARTNER LOCATE", NEON_ORANGE, 2);

    char buf[32];

    if (!loraPaired) {
        fbTextCenter(100, "NOT PAIRED", DIM_CYAN, 2);
        fbTextCenter(130, "PAIR WATCHES FIRST", DIM_GREEN, 1);
    } else {
        // Compass-style view
        int cx = 120, cy = 100;
        fbCircle(cx, cy, 70, GRID_DIM);
        fbCircle(cx, cy, 50, GRID_DIM);
        fbCircle(cx, cy, 30, GRID_DIM);

        // Cardinal directions
        fbText(cx - 4, cy - 80, "N", DIM_CYAN, 1);
        fbText(cx - 4, cy + 72, "S", DIM_CYAN, 1);
        fbText(cx - 80, cy - 4, "W", DIM_CYAN, 1);
        fbText(cx + 72, cy - 4, "E", DIM_CYAN, 1);

        // Partner direction arrow
        if (lastPartnerUpdate > 0 && millis() - lastPartnerUpdate < 60000) {
            float rad = partnerBearing * DEG_TO_RAD;
            int ax = cx + cos(rad - PI/2) * 60;
            int ay = cy + sin(rad - PI/2) * 60;
            fbFillCircle(ax, ay, 8, NEON_GREEN);
            fbLine(cx, cy, ax, ay, NEON_GREEN);

            sprintf(buf, "%.0fM", partnerDist);
            fbTextCenter(180, buf, NEON_GREEN, 3);

            sprintf(buf, "%d DEG", partnerBearing);
            fbTextCenter(215, buf, DIM_CYAN, 1);
        } else {
            fbTextCenter(180, "WAITING...", NEON_YELLOW, 1);
        }

        // My location
        sprintf(buf, "ME: %.4f,%.4f", myLat, myLon);
        fbTextCenter(230, buf, DIM_GREEN, 1);
    }
    fbScanlines();
}

void face_Bio() {
    fbClear(BLACK);
    fbTextCenter(5, "BIOMETRICS", NEON_CYAN, 2);

    char buf[32];
    bodyTemp = 36.5 + sin(frame * 0.02) * 0.3;
    uint16_t tCol = bodyTemp > 37.3 ? NEON_RED : NEON_GREEN;

    fbText(20, 40, "BODY TEMP", DIM_CYAN, 1);
    sprintf(buf, "%.1fC", bodyTemp);
    fbText(20, 55, buf, tCol, 3);

    int spo2 = 97 + sin(frame * 0.07) * 1.5;
    fbText(20, 100, "SPO2", DIM_CYAN, 1);
    sprintf(buf, "%d%%", spo2);
    fbText(20, 115, buf, NEON_CYAN, 3);

    breathRate = 14 + sin(frame * 0.04) * 2;
    fbText(20, 160, "BREATH", DIM_CYAN, 1);
    sprintf(buf, "%d/MIN", breathRate);
    fbText(20, 175, buf, NEON_GREEN, 2);

    // Breathing animation
    float bp = sin(frame * 0.06);
    fbCircle(190, 120, 20 + bp * 15, NEON_GREEN);

    fbLine(0, 205, 240, 205, NEON_CYAN);
    sprintf(buf, "%d NEARBY", entityCount);
    fbTextCenter(215, buf, NEON_GREEN, 1);
    fbScanlines();
}

void face_Profile() {
    fbClear(BLACK);
    fbTextCenter(5, "PROFILE", NEON_PURPLE, 2);

    const char* types[] = {"INTJ", "INFJ", "INTP", "INFP", "ENTJ", "ENFJ", "ENTP", "ENFP"};
    fbTextCenter(35, types[(hrv + entityCount) % 8], NEON_CYAN, 4);

    const char* traits[] = {"INTRO", "INTUIT", "THINK", "JUDGE"};
    int vals[] = {65, 72, 45, 58};
    for (int i = 0; i < 4; i++) {
        int y = 80 + i * 22;
        int v = vals[i] + sin(frame * 0.05 + i) * 5;
        fbText(5, y, traits[i], DIM_CYAN, 1);
        fbRect(60, y, v * 1.2, 12, NEON_CYAN);
    }

    socialCredit = 720 + sin(frame * 0.03) * 30;
    fbText(10, 180, "SOCIAL", DIM_GREEN, 1);
    char buf[16];
    sprintf(buf, "%d", socialCredit);
    fbText(10, 195, buf, socialCredit > 700 ? NEON_GREEN : NEON_YELLOW, 3);
    fbScanlines();
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
            if (abs(i - j) <= 3) {
                float a2 = j * 30 * DEG_TO_RAD + frame * 0.015;
                float r2 = 55 + sin(frame * 0.02 + j) * 15;
                int nx2 = 120 + cos(a2) * r2;
                int ny2 = 120 + sin(a2) * r2;
                fbLine(nx, ny, nx2, ny2, GRID_DIM);
                float p = fmod(frame * 0.04 + i * 0.15, 1.0);
                fbFillCircle(nx + (nx2 - nx) * p, ny + (ny2 - ny) * p, 2, NEON_GREEN);
            }
        }
        fbFillCircle(nx, ny, 5 + sin(frame * 0.15 + i * 0.3) * 2, NEON_CYAN);
    }
    fbFillCircle(120, 115, 10, NEON_GREEN);

    char buf[32];
    sprintf(buf, "%d NODES", 12 + entityCount);
    fbTextCenter(210, buf, NEON_CYAN, 1);
    fbScanlines();
}

void face_System() {
    fbClear(BLACK);
    fbTextCenter(5, "SYSTEM", NEON_CYAN, 2);

    char buf[32];
    int batt = 75 + sin(frame * 0.02) * 5;
    fbRect(60, 35, 100, 40, WHITE);
    fbRect(160, 47, 8, 16, WHITE);
    fbRect(62, 37, batt * 0.96, 36, batt > 50 ? NEON_GREEN : NEON_RED);
    sprintf(buf, "%d%%", batt);
    fbText(85, 80, buf, NEON_GREEN, 2);

    int cpu = 30 + sin(frame * 0.08) * 20;
    fbText(10, 115, "CPU", DIM_CYAN, 1);
    fbRect(50, 115, cpu * 1.4, 12, NEON_GREEN);

    int ram = 45 + cos(frame * 0.06) * 15;
    fbText(10, 135, "RAM", DIM_CYAN, 1);
    fbRect(50, 135, ram * 1.4, 12, NEON_CYAN);

    fbLine(0, 160, 240, 160, NEON_CYAN);

    if (loraPaired) {
        fbText(10, 170, "LORA: PAIRED", NEON_GREEN, 1);
        sprintf(buf, "%08X", partnerID);
        fbText(10, 185, buf, DIM_GREEN, 1);
    } else {
        fbText(10, 170, "LORA: STANDBY", DIM_CYAN, 1);
    }

    sprintf(buf, "%d ENTITIES", entityCount);
    fbText(130, 170, buf, NEON_CYAN, 1);

    // TinyML status
    sprintf(buf, "ML: %s", myBaseline.learned ? "CALIBRATED" : "UNCAL");
    fbText(10, 200, buf, myBaseline.learned ? NEON_GREEN : NEON_YELLOW, 1);

    fbTextCenter(225, "V13.0 RESCUE", DIM_CYAN, 1);
    fbScanlines();
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

    myLat = 47.3769 + sin(frame * 0.01) * 0.001;
    myLon = 8.5417 + cos(frame * 0.01) * 0.001;

    char buf[32];
    sprintf(buf, "%.4f", myLat);
    fbText(60, 145, buf, NEON_GREEN, 2);
    sprintf(buf, "%.4f", myLon);
    fbText(60, 175, buf, NEON_GREEN, 2);

    sprintf(buf, "%d NEARBY", entityCount);
    fbTextCenter(210, buf, NEON_CYAN, 1);
    fbScanlines();
}

void face_About() {
    fbClear(BLACK);

    // Animated winking smiley
    bool wink = ((frame / 20) % 4) == 1;
    fbCircle(120, 30, 22, NEON_PINK);
    if (wink) {
        fbLine(106, 25, 118, 25, NEON_CYAN);
    } else {
        fbFillCircle(112, 25, 4, NEON_CYAN);
    }
    fbFillCircle(128, 25, 4, NEON_CYAN);
    for (int i = -10; i <= 10; i++) {
        fbPixel(120 + i, 38 + abs(i) / 3, NEON_GREEN);
    }

    fbTextCenter(60, "HYPERLOG", NEON_PURPLE, 2);
    fbTextCenter(85, "BIOELECTRIC", NEON_CYAN, 2);
    fbTextCenter(110, "NEURAL OCEAN", NEON_GREEN, 2);
    fbTextCenter(140, "V14.0", NEON_PINK, 2);

    const char* feat[] = {"TRIBE FINDER", "HEALTH MAP", "RUBBLE RESCUE", "ECHOLOCATION", "BIOML"};
    fbTextCenter(165, feat[(frame / 45) % 5], NEON_ORANGE, 1);

    fbTextCenter(190, "T-WATCH S3", NEON_PURPLE, 1);

    char buf[32];
    sprintf(buf, "BIO: %s", myBioSig.learned ? "LEARNED" : "UNCAL");
    fbTextCenter(210, buf, myBioSig.learned ? NEON_GREEN : NEON_YELLOW, 1);
    fbGlitch(2);
    fbScanlines();
}

// NEW: Proximity Health Map - people on map with health stats
void face_HealthMap() {
    fbClear(BLACK);
    fbTextCenter(3, "HEALTH MAP", NEON_PINK, 2);

    char buf[32];
    int cx = 120, cy = 105;

    // Map grid
    for (int x = 20; x < 220; x += 40) fbLine(x, 30, x, 180, GRID_DIM);
    for (int y = 30; y < 180; y += 30) fbLine(20, y, 220, y, GRID_DIM);

    // YOU marker in center
    fbFillCircle(cx, cy, 8, NEON_GREEN);
    fbText(cx - 8, cy - 3, "U", BLACK, 1);

    // Draw entities with health data
    int shown = 0;
    for (int i = 0; i < entityCount && shown < 6; i++) {
        if (millis() - entities[i].lastSeen > 15000) continue;
        if (entities[i].isDrone) continue;

        // Position on map (relative to center)
        int ex = cx + (entities[i].x - 120) * 0.8;
        int ey = cy + (entities[i].y - 120) * 0.8;
        ex = constrain(ex, 25, 215);
        ey = constrain(ey, 35, 175);

        // Color by health status
        uint16_t col = entities[i].predTemp > 375 ? NEON_RED :
                       entities[i].predStress > 60 ? NEON_YELLOW : NEON_CYAN;

        // Entity dot with pulse
        float pulse = sin(frame * 0.2 + i) * 2;
        fbFillCircle(ex, ey, 5 + pulse, col);

        // Mini health readout next to entity
        if (shown < 3) {
            sprintf(buf, "%d", entities[i].predHR);
            fbText(ex + 8, ey - 4, buf, NEON_PINK, 1);
        }
        shown++;
    }

    // Health stats panel
    fbLine(0, 185, 240, 185, NEON_PURPLE);

    // Find closest entity for detailed view
    int closest = -1;
    float minD = 99;
    for (int i = 0; i < entityCount; i++) {
        if (!entities[i].isDrone && entities[i].distanceM < minD) {
            minD = entities[i].distanceM;
            closest = i;
        }
    }

    if (closest >= 0) {
        Entity* e = &entities[closest];
        sprintf(buf, "%.1fM", e->distanceM);
        fbText(5, 192, buf, NEON_GREEN, 2);

        sprintf(buf, "%dBPM", e->predHR);
        fbText(70, 195, buf, NEON_PINK, 1);

        sprintf(buf, "%.1fC", e->predTemp / 10.0);
        fbText(130, 195, buf, e->predTemp > 375 ? NEON_RED : NEON_CYAN, 1);

        // Fringe: Eustachian tube fluid
        sprintf(buf, "ETF:%d", e->predTubeFluid);
        fbText(5, 215, buf, NEON_PURPLE, 1);

        sprintf(buf, "VES:%d", e->predVestibular);
        fbText(80, 215, buf, NEON_CYAN, 1);

        sprintf(buf, "BIO:%d", e->predBiofield);
        fbText(155, 215, buf, NEON_GREEN, 1);
    } else {
        fbTextCenter(200, "NO ENTITIES", NEON_PURPLE, 2);
    }
    fbScanlines();
}

// NEW: Tribe Finder - MBTI compatibility matching
void face_TribeFinder() {
    fbClear(BLACK);
    fbTextCenter(3, "TRIBE FINDER", NEON_PINK, 2);

    char buf[32];

    // MBTI types based on bioelectric patterns
    const char* types[] = {"INTJ", "INFJ", "INTP", "INFP", "ENTJ", "ENFJ", "ENTP", "ENFP",
                           "ISTJ", "ISFJ", "ISTP", "ISFP", "ESTJ", "ESFJ", "ESTP", "ESFP"};

    // My type (derived from HRV/vagal tone)
    int myTypeIdx = (myBioSig.hrVariability + myBioSig.baseHR) % 16;
    sprintf(buf, "YOU: %s", types[myTypeIdx]);
    fbTextCenter(30, buf, NEON_GREEN, 2);

    // Find compatible matches
    int matchCount = 0;
    int bestMatch = -1;
    float bestSim = 0;

    fbLine(0, 55, 240, 55, NEON_PURPLE);
    fbText(5, 60, "COMPATIBLE:", NEON_CYAN, 1);

    int y = 80;
    for (int i = 0; i < entityCount && matchCount < 4; i++) {
        if (entities[i].isDrone) continue;
        if (millis() - entities[i].lastSeen > 20000) continue;

        // Calculate compatibility from bio-similarity
        float compat = entities[i].bioSimilarity;

        // MBTI compatibility boost for certain combinations
        int theirType = (entities[i].predHR + entities[i].predHRV) % 16;

        // INFx matches with INFx/INTx (simplified)
        if ((myTypeIdx < 4 && theirType < 4) || (myTypeIdx >= 4 && myTypeIdx < 8 && theirType >= 4 && theirType < 8)) {
            compat += 0.2;
        }

        compat = min(1.0f, compat);

        if (compat > 0.5) {
            // Show match
            uint16_t col = compat > 0.7 ? NEON_GREEN : NEON_CYAN;

            sprintf(buf, "%s %.0f%%", types[theirType], compat * 100);
            fbText(10, y, buf, col, 2);

            // Distance and direction arrow
            float angle = atan2(entities[i].y - 120, entities[i].x - 120);
            int arrowX = 180 + cos(angle) * 20;
            int arrowY = y + 8 + sin(angle) * 10;
            fbFillCircle(arrowX, arrowY, 5, NEON_PINK);
            fbLine(180, y + 8, arrowX, arrowY, NEON_PINK);

            sprintf(buf, "%.0fM", entities[i].distanceM);
            fbText(200, y, buf, NEON_PURPLE, 1);

            if (compat > bestSim) {
                bestSim = compat;
                bestMatch = i;
            }

            y += 35;
            matchCount++;
        }
    }

    if (matchCount == 0) {
        fbTextCenter(120, "SCANNING", NEON_PURPLE, 2);
        fbTextCenter(150, "FOR TRIBE", NEON_CYAN, 2);

        // Animated search rings
        int r = (frame * 2) % 60 + 20;
        fbCircle(120, 140, r, NEON_PINK);
    }

    // Best match alert at bottom
    if (bestMatch >= 0 && bestSim > 0.7) {
        fbRect(0, 200, 240, 40, NEON_GREEN);
        fbTextCenter(205, "MATCH FOUND!", BLACK, 2);

        // Direction indicator
        float angle = atan2(entities[bestMatch].y - 120, entities[bestMatch].x - 120);
        const char* dirs[] = {"->", "\\>", "V", "</", "<-", "<\\", "^", "/>"};
        int dirIdx = ((int)((angle + PI) / (PI / 4)) + 8) % 8;
        sprintf(buf, "%s %.0fM %s", dirs[dirIdx], entities[bestMatch].distanceM,
                (entities[bestMatch].x > 120) ? "RIGHT" : "LEFT");
        fbTextCenter(225, buf, BLACK, 1);
    }

    fbScanlines();
}

// NEW: Radiation Detection Face
void face_Radiation() {
    fbClear(BLACK);

    // Alert color if high radiation
    uint16_t titleCol = (radiationuSv > 1.0 && (frame / 8) % 2) ? NEON_RED : NEON_YELLOW;
    fbTextCenter(3, "RADIATION", titleCol, 2);

    char buf[32];

    // Simulate radiation from signal anomalies
    // Real implementation would use actual Geiger counter
    radioactiveItems = 0;
    float totalRad = 0.08 + sin(frame * 0.02) * 0.03;  // Background

    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen > 10000) continue;

        // Signal anomalies interpreted as radiation sources
        // Very weak signals with erratic patterns = potential source
        if (entities[i].rssi < -85 || entities[i].isDrone) {
            float contribution = 0.1 + random(50) / 100.0;
            totalRad += contribution;
            radioactiveItems++;
        }
    }

    radiationuSv = totalRad;
    if (radiationuSv > peakRadiation) peakRadiation = radiationuSv;

    // Geiger tick rate based on radiation level
    radTickRate = (int)(radiationuSv * 10);

    // Main radiation display - BIG readable
    sprintf(buf, "%.2f", radiationuSv);
    fbTextCenter(40, buf, radiationuSv > 1.0 ? NEON_RED : NEON_GREEN, 4);
    fbTextCenter(90, "uSV/H", NEON_CYAN, 2);

    // Peak value
    sprintf(buf, "PEAK: %.2f", peakRadiation);
    fbText(10, 115, buf, NEON_PURPLE, 1);

    // Status bar
    fbLine(0, 130, 240, 130, NEON_YELLOW);

    // Radiation radar - shows "hot" items
    int cx = 120, cy = 175;
    fbCircle(cx, cy, 40, GRID_DIM);
    fbCircle(cx, cy, 25, GRID_DIM);
    fbCircle(cx, cy, 10, GRID_DIM);
    fbLine(cx - 45, cy, cx + 45, cy, GRID_DIM);
    fbLine(cx, cy - 45, cx, cy + 45, GRID_DIM);

    // Draw radioactive items on radar
    int hotCount = 0;
    for (int i = 0; i < entityCount && hotCount < 5; i++) {
        if (millis() - entities[i].lastSeen > 10000) continue;
        if (entities[i].rssi > -85 && !entities[i].isDrone) continue;

        // Position on radar
        float angle = (entities[i].mac[0] + entities[i].mac[5]) * 0.1;
        float r = min(35.0f, entities[i].distanceM * 3);
        int rx = cx + cos(angle) * r;
        int ry = cy + sin(angle) * r;

        // Radioactive symbol (trefoil approximation)
        float pulse = sin(frame * 0.3 + i) * 2;
        fbFillCircle(rx, ry, 4 + pulse, NEON_YELLOW);
        fbCircle(rx, ry, 6 + pulse, NEON_RED);

        hotCount++;
    }

    // Geiger counter clicks visualization
    int clickY = 135;
    for (int i = 0; i < min(20, radTickRate); i++) {
        int x = 10 + (i * 11) + random(5);
        int h = 3 + random(8);
        uint16_t col = (i % 3 == 0) ? NEON_YELLOW : NEON_GREEN;
        fbRect(x, clickY, 8, h, col);
    }

    // Alert toggle status
    fbLine(0, 220, 240, 220, NEON_PURPLE);
    sprintf(buf, "ALERT: %s", radiationAlertOn ? "ON" : "OFF");
    fbText(10, 225, buf, radiationAlertOn ? NEON_GREEN : DIM_GREEN, 1);
    fbText(120, 225, "TAP TO TOGGLE", NEON_CYAN, 1);

    // Hot items count
    sprintf(buf, "HOT: %d", radioactiveItems);
    fbText(180, 135, buf, NEON_YELLOW, 1);

    // Warning flash if high
    if (radiationuSv > 1.0 && (frame / 4) % 2) {
        fbRect(0, 0, 240, 3, NEON_RED);
        fbRect(0, 237, 240, 3, NEON_RED);
        fbRect(0, 0, 3, 240, NEON_RED);
        fbRect(237, 0, 3, 240, NEON_RED);
    }

    fbGlitch(2);
    fbScanlines();
}

void drawCurrentFace() {
    switch (currentFace) {
        case 0: face_Clock(); break;
        case 1: face_WalkieTalkie(); break;
        case 2: face_Vitals(); break;
        case 3: face_Radar(); break;
        case 4: face_Proximity(); break;
        case 5: face_Alien(); break;
        case 6: face_RubbleDetector(); break;
        case 7: face_Echolocation(); break;
        case 8: face_HealthMap(); break;
        case 9: face_TribeFinder(); break;
        case 10: face_Radiation(); break;     // NEW
        case 11: face_Drone(); break;
        case 12: face_PartnerLoc(); break;
        case 13: face_Bio(); break;
        case 14: face_Profile(); break;
        case 15: face_Neural(); break;
        case 16: face_System(); break;
        case 17: face_About(); break;
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

    rtcWrite(21, 50, 0, 8, 1, 26);

    initDisplay();
    initTouch();
    initLoRa();
    startSniffer();

    pinMode(BTN_1, INPUT_PULLUP);

    // Animated Black Mirror winking smiley splash
    for (int anim = 0; anim < 60; anim++) {  // 60 frames animation
        fbClear(BLACK);

        int smx = 120, smy = 55;

        // Face outline with pulse
        int faceR = 40 + sin(anim * 0.2) * 3;
        fbCircle(smx, smy, faceR, NEON_CYAN);
        fbCircle(smx, smy, faceR - 2, GRID_DIM);

        // LEFT EYE - winks periodically
        bool winking = (anim > 30 && anim < 45);
        if (winking) {
            // Winking - horizontal line
            fbLine(smx - 18, smy - 8, smx - 6, smy - 8, NEON_GREEN);
            fbLine(smx - 18, smy - 7, smx - 6, smy - 7, NEON_GREEN);
        } else {
            // Open eye
            fbFillCircle(smx - 12, smy - 8, 5, NEON_GREEN);
            fbFillCircle(smx - 12, smy - 8, 2, BLACK);  // Pupil
        }

        // RIGHT EYE - always open
        fbFillCircle(smx + 12, smy - 8, 5, NEON_GREEN);
        fbFillCircle(smx + 12, smy - 8, 2, BLACK);  // Pupil

        // Creepy smile - curves up more during wink
        int smileUp = winking ? 3 : 0;
        for (int i = -18; i <= 18; i++) {
            int sy = smy + 12 + abs(i) / 3 - smileUp;
            fbPixel(smx + i, sy, NEON_GREEN);
            fbPixel(smx + i, sy + 1, NEON_GREEN);
        }

        // Glitch scanlines
        if (anim % 8 < 2) {
            int gy = random(smy - 30, smy + 30);
            for (int gx = smx - 35; gx < smx + 35; gx++) {
                if (random(4) == 0) fbPixel(gx, gy, NEON_PURPLE);
            }
        }

        // HYPERLOG BIOELECTRIC heading
        fbTextCenter(110, "HYPERLOG", NEON_PURPLE, 2);
        fbTextCenter(135, "BIOELECTRIC", NEON_CYAN, 2);
        fbTextCenter(160, "NEURAL OCEAN", NEON_GREEN, 2);
        fbTextCenter(190, "V14.0", NEON_ORANGE, 2);

        // Animated text smiley
        const char* smileys[] = {":)", ";)", ":)", ":)"};
        fbTextCenter(215, smileys[(anim / 15) % 4], NEON_CYAN, 2);

        fbScanlines();
        pushFramebuffer();
        delay(50);  // 20 FPS animation
    }

    Serial.println("HYPERLOG BIOELECTRIC NEURAL OCEAN v14.0 Ready");
}

unsigned long lastFrame = 0;
unsigned long lastFaceChange = 0;
unsigned long btnPressStart = 0;
bool btnWasPressed = false;

void loop() {
    // Read touch
    readTouch();

    // Handle swipe
    if (swipeDetected) {
        swipeDetected = false;
        if (swipeDir == -1) {  // Swipe left = next
            currentFace = (currentFace + 1) % TOTAL_FACES;
            lastFaceChange = millis();
        } else if (swipeDir == 1) {  // Swipe right = prev
            currentFace = (currentFace + TOTAL_FACES - 1) % TOTAL_FACES;
            lastFaceChange = millis();
        }
    }

    // Handle tap on specific faces
    static bool lastTouchState = false;
    if (touchPressed && !lastTouchState) {
        // Tap detected
        if (currentFace == 10) {  // Radiation face
            // Toggle alert beep on tap
            radiationAlertOn = !radiationAlertOn;
        }
    }
    lastTouchState = touchPressed;

    // Handle button
    bool btnPressed = (digitalRead(BTN_1) == LOW);

    if (btnPressed && !btnWasPressed) {
        btnPressStart = millis();
        btnWasPressed = true;
    }

    if (!btnPressed && btnWasPressed) {
        unsigned long dur = millis() - btnPressStart;
        btnWasPressed = false;

        if (dur < 500) {
            currentFace = (currentFace + 1) % TOTAL_FACES;
            lastFaceChange = millis();
        } else if (dur < 2000 && currentFace == 1 && hasNewMessage) {
            hasNewMessage = false;
        }
    }

    // Long press 2s = pairing (on walkie face) or ML learning (on rubble face)
    if (btnPressed && btnWasPressed && (millis() - btnPressStart > 2000)) {
        if (currentFace == 6 && !learningMode) {
            // Rubble face: start TinyML learning at 30cm
            learnMySignal();
            btnWasPressed = false;
        } else if (!pairingMode && !loraPaired && currentFace == 1) {
            pairingMode = true;
            pairingStart = millis();
            sendPairRequest();
            btnWasPressed = false;  // Reset to prevent re-trigger
        }
    }

    // Process LoRa
    processLoRa();

    // Send location periodically when paired
    static unsigned long lastLocSend = 0;
    if (loraPaired && millis() - lastLocSend > 5000) {
        sendLocation();
        lastLocSend = millis();
    }

    // Pairing beacon
    if (pairingMode && millis() - pairingStart < 30000) {
        static unsigned long lastBeacon = 0;
        if (millis() - lastBeacon > 1000) {
            sendPairRequest();
            lastBeacon = millis();
        }
    }

    // Continue TinyML learning if active
    if (learningMode) {
        learnMySignal();
    }

    // 25 FPS
    if (millis() - lastFrame > 40) {
        lastFrame = millis();
        frame++;
        updateEntityStats();
        drawCurrentFace();
    }

    // Auto cycle
    if (millis() - lastFaceChange > 6000) {
        lastFaceChange = millis();
        currentFace = (currentFace + 1) % TOTAL_FACES;
    }

    // WiFi channel hop
    static unsigned long lastHop = 0;
    static int ch = 1;
    if (millis() - lastHop > 3000) {
        ch = (ch % 13) + 1;
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        lastHop = millis();
    }
}
