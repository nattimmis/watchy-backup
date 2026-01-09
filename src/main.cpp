/*
 * HYPERLOG BIOELECTRIC NEURAL OCEAN v15.0
 * 58 Spectacular Watch Faces with Black Mirror Effects
 * Features: Body Scanner, Sonar Emulation, Quantum Detection, Ternary Logic
 * Scientific references: Kyle 2004 BIA, Ma 2019 WiFi ToF, Nielsen 2010 Quantum
 * Shaffer 2017 HRV, Porges 2011 Polyvagal, Berger 1929 EEG, Boucsein 2012 EDA
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>
#include <driver/i2s.h>

// Second I2C bus for touch controller
TwoWire TouchWire = TwoWire(1);

// Pins - Main I2C (PMIC, RTC)
#define I2C_SDA     10
#define I2C_SCL     11

// Touch I2C - SEPARATE BUS on T-Watch S3 (FT6336U)
#define TOUCH_SDA   39
#define TOUCH_SCL   40
#define TOUCH_INT   16
// Note: TOUCH_RST is not connected on T-Watch S3
#define FT6336_ADDR  0x38  // Focaltech FT6336U address

// Display
#define TFT_BL      45
#define TFT_CS      12
#define TFT_DC      38
#define TFT_MOSI    13
#define TFT_SCLK    18

// LoRa SX1262 - T-Watch S3 Plus pins (from LilyGO wiki)
// https://wiki.lilygo.cc/get_started/en/Wearable/T-Watch-S3-PLUS/T-Watch-S3-PLUS.html
#define LORA_MOSI   1
#define LORA_MISO   4
#define LORA_SCK    3
#define LORA_CS     5
#define LORA_RST    8
#define LORA_BUSY   7
#define LORA_DIO1   9

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

// Forward declarations
float rssiToDistance(int rssi);

// State
int currentFace = 0;
const int TOTAL_FACES = 60;  // 18 original + 40 new + 2 walkie-talkie faces
uint32_t frame = 0;

// Calibration: Arm length from heart to wrist
// Average arm length ~60cm (Winter 2009 Biomechanics)
// Watch worn at wrist = ~60cm from heart center
// Nearest entity = wearer at ~0.6m from heart
#define ARM_LENGTH_CM 60       // Heart to wrist distance
#define WEARER_OFFSET_M 0.6    // Wearer's phone is ~0.6m from watch (in pocket/hand)
float calibRSSI = -45;         // Will be learned at calibration distance

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

// ============= QUANTUM-INSPIRED DETECTION (Deutsch-Jozsa style) =============
// Based on quantum probability superposition concepts
// Nielsen & Chuang 2010 "Quantum Computation and Quantum Information"
struct Qubit {
    float alpha;    // Probability amplitude |0⟩ (no human)
    float beta;     // Probability amplitude |1⟩ (human present)
    float phase;    // Phase angle (entanglement indicator)
    bool collapsed; // Measured state
    int result;     // 0, 1, or -1 (superposition)
};
Qubit qubits[8];    // 8-qubit quantum-style register
float quantumCoherence = 1.0;   // Decoherence factor
float entanglementStrength = 0.0;
int quantumIterations = 0;

// Max entities (defined early for array sizing)
#ifndef MAX_ENTITIES
#define MAX_ENTITIES 30
#endif

// ============= TERNARY LOGIC DETECTION (Łukasiewicz 1920) =============
// Three-valued logic: True(1), False(-1), Unknown(0)
// Kleene 1938 strong logic for uncertainty
struct TernaryState {
    int8_t humanPresent;    // -1=no, 0=unknown, 1=yes
    int8_t threatLevel;     // -1=safe, 0=unknown, 1=threat
    int8_t bioMatch;        // -1=foreign, 0=unknown, 1=familiar
    float confidence;       // Fuzzy confidence 0.0-1.0
};
TernaryState ternaryDetect[MAX_ENTITIES];
int ternaryTrue = 0, ternaryFalse = 0, ternaryUnknown = 0;

// ============= SONAR EMULATION (Time-of-Flight WiFi) =============
// Based on Fine Timing Measurement (FTM) IEEE 802.11mc
// Ma et al. 2019 "WiFi-based indoor positioning"
#define SONAR_SECTORS 16
#define SONAR_RANGE_M 25
struct SonarPing {
    float distance;         // Meters
    float strength;         // Signal return strength 0-100
    float velocity;         // Doppler-like velocity estimate m/s
    uint32_t lastPing;
    bool hasTarget;
};
SonarPing sonarMap[SONAR_SECTORS];
float sonarSweepAngle = 0;
int sonarTargetCount = 0;
float sonarNearestM = 99.9;
unsigned long lastSonarPulse = 0;

// ============= NEURAL NETWORK STATE (TinyML McCulloch-Pitts) =============
// Simplified perceptron for entity classification
// Rosenblatt 1958, updated Warden 2019 TinyML
#define NN_INPUTS 6
#define NN_HIDDEN 4
#define NN_OUTPUTS 3
float nnWeights1[NN_INPUTS][NN_HIDDEN];
float nnWeights2[NN_HIDDEN][NN_OUTPUTS];
float nnBias1[NN_HIDDEN], nnBias2[NN_OUTPUTS];
float nnOutput[NN_OUTPUTS];  // 0=Human, 1=Drone, 2=Unknown
bool nnTrained = false;

// ============= ADVANCED BIOMETRIC STATES =============
// Circadian rhythm (Czeisler 1999)
float circadianPhase = 0;   // 0-24 hours
float melatoninLevel = 0.5; // 0-1 simulated
float cortisolLevel = 0.5;  // 0-1 morning peak

// Electrodermal activity (Boucsein 2012)
float edaLevel = 5.0;       // Skin conductance microsiemens
float edaPhasic = 0;        // Fast response component
float edaTonic = 5.0;       // Baseline level

// Brain wave simulation (Berger 1929 EEG bands)
float alphaWave = 0.5;      // 8-13 Hz relaxed
float betaWave = 0.3;       // 14-30 Hz alert
float thetaWave = 0.2;      // 4-7 Hz drowsy
float deltaWave = 0.0;      // 0.5-4 Hz deep sleep
float gammaWave = 0.1;      // 30-100 Hz cognition

// Chakra/Biofield (fringe but visually spectacular)
float chakraEnergy[7] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
const char* chakraNames[] = {"ROOT", "SACRAL", "SOLAR", "HEART", "THROAT", "3RDEYE", "CROWN"};
uint16_t chakraColors[] = {NEON_RED, NEON_ORANGE, NEON_YELLOW, NEON_GREEN, NEON_CYAN, NEON_PURPLE, NEON_PINK};

// Aura detection (Kirlian-style visualization)
float auraStrength = 0.7;
float auraRadius = 30;
uint16_t auraColor = NEON_CYAN;

// Timeline/probability states
float timelineConvergence = 0.85;
int parallelTimelines = 3;
float fateIndex = 0.72;

// Morphic resonance (Sheldrake fringe theory)
float morphicField = 0.5;
int collectiveMemoryHits = 0;

// Dark matter detection (for fun - obviously simulated)
float darkMatterDensity = 0.23; // 23% of universe mass
int darkMatterEvents = 0;

// Gravitational wave (LIGO-inspired visualization)
float gwAmplitude = 0;
float gwFrequency = 0;

// Neutrino flux (Super-Kamiokande inspired)
int neutrinoCount = 0;
float neutrinoEnergy = 0;

// ============= BODY SCANNER / BIOIMPEDANCE (Kyle 2004, Lukaski 1987) =============
// Based on Bioelectrical Impedance Analysis (BIA) principles
// Measures body composition via electrical resistance at different frequencies
// Reference: Kyle et al. 2004 "Bioelectrical impedance analysis" Clinical Nutrition
// Lukaski 1987 "Methods for the assessment of human body composition"

struct BodyComposition {
    // Primary measurements (BIA standard - Kushner 1992)
    float totalBodyWater;     // TBW: 45-65% of body weight (Watson formula)
    float extracellularWater; // ECW: ~45% of TBW (Hanai mixture theory)
    float intracellularWater; // ICW: ~55% of TBW

    // Fat analysis (Segal 1988 equations)
    float bodyFatPercent;     // 10-30% healthy range (ACSM)
    float visceralFat;        // 1-12 healthy (Tanita scale)
    float subcutaneousFat;    // Surface fat layer mm

    // Adipocyte (fat cell) fluid analysis
    // Fat cells are 10-90% lipid, remainder is water + organelles
    // Reference: Frayn 2010 "Metabolic Regulation"
    float adipocyteHydration; // Fat cell water content 0-1
    float lipidDropletSize;   // Avg lipid droplet diameter μm (10-100)
    float freeGlycerol;       // Lipolysis indicator mmol/L

    // Muscle/Lean mass (Janssen 2000)
    float leanMass;           // Fat-free mass kg
    float skeletalMuscle;     // Skeletal muscle mass %
    float muscleTone;         // Impedance-based tone estimate

    // Cellular health (Cole-Cole model)
    float cellMembraneCap;    // Cell membrane capacitance pF
    float phaseAngle;         // Health indicator 4-10° normal (Barbosa-Silva 2005)
    float bodyCapacitance;    // Overall body capacitance

    // Hydration zones (segmental - Kyle 2004)
    float armHydration;
    float legHydration;
    float torsoHydration;

    bool calibrated;
};
BodyComposition bodyScan = {0};

// ============= ECHOLOCATION CALIBRATION (Kolarik 2014) =============
// Human echolocation studies: Thaler & Goodale 2016
// WiFi-based distance estimation: Bahl 2000 RADAR system
// Acoustic model: inverse square law + absorption

struct EchoCalibration {
    // Reference points (calibrated distances)
    float refDistance1m;      // RSSI at 1 meter
    float refDistance3m;      // RSSI at 3 meters
    float pathLossExp;        // Path loss exponent n (2.0-4.0, Rappaport 2002)

    // Body echo profile
    float bodyReflectivity;   // How much signal reflects off body
    float waterAbsorption;    // Higher water = more absorption at 2.4GHz
    float fatAttenuation;     // Fat tissue signal attenuation dB/cm

    // Fluid detection thresholds
    float normalHydration;    // Expected signal for normal hydration
    float dehydratedSig;      // Signal pattern when dehydrated
    float overhydratedSig;    // Edema detection threshold

    bool calibrated;
    int calibrationStep;
    unsigned long calibStart;
};
EchoCalibration echoCal = {-45, -55, 2.7, 0.3, 0.8, 0.5, -50, -45, -55, false, 0, 0};

// Scan animation state
int scanLineY = 0;
bool scanningBody = false;
int scanPhase = 0;
float scanProgress = 0;

// ============= ULTRASOUND-INSPIRED TISSUE ANALYSIS =============
// Based on acoustic impedance: Z = ρ × c (density × sound speed)
// Different tissues have different impedances (Szabo 2004)
// Fat: 1.38 MRayl, Muscle: 1.70 MRayl, Water: 1.48 MRayl

struct TissueLayer {
    const char* name;
    float thickness;      // mm
    float impedance;      // MRayl (Mega Rayls)
    float fluidContent;   // 0-1
    uint16_t color;
};

// Body layers from surface inward
TissueLayer tissueStack[6] = {
    {"SKIN", 2.0, 1.60, 0.70, NEON_PINK},
    {"SUBCUT FAT", 15.0, 1.38, 0.15, NEON_YELLOW},
    {"FASCIA", 1.0, 1.65, 0.65, NEON_CYAN},
    {"MUSCLE", 25.0, 1.70, 0.75, NEON_RED},
    {"DEEP FAT", 8.0, 1.38, 0.12, NEON_ORANGE},
    {"ORGAN", 20.0, 1.55, 0.80, NEON_PURPLE}
};

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
bool isPlaying = false;
unsigned long recordStart = 0;
#define VOICE_BUF_SIZE 8000
#define VOICE_CHUNK_SIZE 200  // Bytes per LoRa packet (max ~250)
int16_t voiceBuffer[VOICE_BUF_SIZE];
int16_t playBuffer[VOICE_BUF_SIZE];
int voiceLen = 0;
int playLen = 0;
int playPos = 0;
int msgRSSI = 0;
int voicePacketsReceived = 0;
int voicePacketsExpected = 0;

// Partner location (from LoRa)
float partnerLat = 0, partnerLon = 0;
float partnerDist = 0;
float partnerBearing = 0;
unsigned long lastPartnerUpdate = 0;
int partnerRSSI = -100;

// My location (from WiFi triangulation + partner help)
float myLat = 47.3769, myLon = 8.5417;

// Dual-watch triangulation for finding targets
struct TriangTarget {
    uint8_t mac[6];
    int myRSSI;
    int partnerRSSI;
    float myDist;
    float partnerDist;
    float estimatedX, estimatedY;  // Relative position
    bool valid;
    unsigned long lastSeen;
};
#define MAX_TRIANG_TARGETS 8
TriangTarget triangTargets[MAX_TRIANG_TARGETS];
int triangTargetCount = 0;

// Partner sensor data (shared via LoRa)
struct PartnerSensorData {
    int heartRate;
    int hrv;
    int stress;
    int batteryPct;
    int nearestEntityRSSI;
    uint8_t nearestEntityMAC[6];
    float nearestDist;
    bool valid;
};
PartnerSensorData partnerSensors = {0};

// Voice message queue
#define MAX_VOICE_MSGS 3
struct VoiceMessage {
    int16_t samples[VOICE_BUF_SIZE];
    int length;
    int rssi;
    unsigned long timestamp;
    bool played;
};
VoiceMessage voiceMsgs[MAX_VOICE_MSGS];
int voiceMsgCount = 0;
int currentPlayMsg = -1;

// PTT (Push-To-Talk) state
bool pttPressed = false;
unsigned long pttStart = 0;
#define MAX_RECORD_MS 5000  // Max 5 second voice notes

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
        // Set all ALDO voltages to 3.3V (value 28 = 0x1C)
        axpWrite(0x92, 0x1C);  // ALDO1 - RTC
        axpWrite(0x93, 0x1C);  // ALDO2 - TFT backlight
        axpWrite(0x94, 0x1C);  // ALDO3 - Touch screen power
        axpWrite(0x95, 0x1C);  // ALDO4 - LoRa
        axpWrite(0x97, 0x1C);  // BLDO2 - Haptic motor

        // Enable: ALDO1(1) + ALDO2(2) + ALDO3(4) + ALDO4(8) + BLDO2(32) = 47
        axpWrite(0x90, 47);    // LDO ON/OFF control
        axpWrite(0x80, 1);     // DCDC1 ON for ESP32

        // Enable button IRQ (register 0x40 = IRQ enable 0)
        axpWrite(0x40, 0x03);  // Enable short press and long press IRQ
        delay(100);  // Wait for power to stabilize
        Serial.println("PMIC: All power rails enabled");
    }
}

// Read button state from AXP2101 PMU
bool readPMUButton() {
    uint8_t irqStatus = axpRead(0x49);  // IRQ status register 1
    if (irqStatus & 0x01) {  // Short press detected
        axpWrite(0x49, 0x01);  // Clear IRQ
        return true;
    }
    return false;
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

// ============= Touch Controller (FT5336 / FT6336 / CST816) =============
// Auto-detect which touch controller is present

volatile bool touchInterrupt = false;
uint8_t touchAddr = 0;  // Detected touch controller address
bool isFocaltech = false;  // True for FT5336/FT6336, false for CST816

void IRAM_ATTR touchISR() {
    touchInterrupt = true;
}

// Store I2C addresses found during scan
uint8_t i2cFound[16];
int i2cFoundCount = 0;

// FT6336U register addresses (from Adafruit library and datasheet)
#define FT6336_REG_DEVICE_MODE  0x00
#define FT6336_REG_GEST_ID      0x01
#define FT6336_REG_TD_STATUS    0x02
#define FT6336_REG_THRESHOLD    0x80
#define FT6336_REG_POINT_RATE   0x88
#define FT6336_REG_CTRL         0x86
#define FT6336_REG_G_MODE       0xA4
#define FT6336_REG_POWER_MODE   0xA5
#define FT6336_REG_CHIP_ID      0xA3
#define FT6336_REG_FIRMWARE_ID  0xA6
#define FT6336_REG_VENDOR_ID    0xA8

// Expected chip IDs
#define FT6206_CHIP_ID   0x06
#define FT6236_CHIP_ID   0x36
#define FT6236U_CHIP_ID  0x64
#define FT6336U_CHIP_ID  0x64
#define FOCALTECH_VENDOR_ID 0x11

// Touch data buffer - LilyGO method: read 5 bytes from reg 0x02 (TD_STATUS)
// This matches the official TTGO_TWatch_Library FocalTech driver
uint8_t touchData[16];

// FT6336U register addresses for direct reading (aselectroworks library)
#define FT6336_REG_TD_STATUS   0x02   // Number of touch points
#define FT6336_REG_P1_XH       0x03   // Touch 1 X high byte + event flag
#define FT6336_REG_P1_XL       0x04   // Touch 1 X low byte
#define FT6336_REG_P1_YH       0x05   // Touch 1 Y high byte + touch ID
#define FT6336_REG_P1_YL       0x06   // Touch 1 Y low byte
#define FT6336_REG_P1_WEIGHT   0x07   // Touch 1 weight/pressure
#define FT6336_REG_P1_MISC     0x08   // Touch 1 misc/area

// Helper to write to FT6336U register (with proper timing)
void ft6336Write(uint8_t reg, uint8_t val) {
    TouchWire.beginTransmission(FT6336_ADDR);
    TouchWire.write(reg);
    TouchWire.write(val);
    TouchWire.endTransmission();
    delay(10);  // Critical: FT6336U needs time after write (per aselectroworks lib)
}

// Helper to read from FT6336U register (with proper timing)
uint8_t ft6336Read(uint8_t reg) {
    TouchWire.beginTransmission(FT6336_ADDR);
    TouchWire.write(reg);
    TouchWire.endTransmission();  // Full stop, not repeated start
    delay(10);  // Critical: 10ms delay per aselectroworks library
    TouchWire.requestFrom((int)FT6336_ADDR, 1);
    return TouchWire.available() ? TouchWire.read() : 0;
}

// Read multiple bytes from FT6336U (with proper timing)
bool ft6336ReadBytes(uint8_t reg, uint8_t* data, uint8_t len) {
    TouchWire.beginTransmission(FT6336_ADDR);
    TouchWire.write(reg);
    TouchWire.endTransmission();  // Full stop
    delay(10);  // Critical delay

    uint8_t received = TouchWire.requestFrom((int)FT6336_ADDR, (int)len);
    if (received < len) return false;

    for (uint8_t i = 0; i < len; i++) {
        data[i] = TouchWire.read();
    }
    return true;
}

// Read touch data - LilyGO method: read 5 bytes from TD_STATUS (0x02)
// Layout: [TD_STATUS, P1_XH, P1_XL, P1_YH, P1_YL]
// This is exactly how the official TTGO_TWatch_Library does it
bool ft6336ReadData() {
    // Method 1: Read 5 bytes starting at TD_STATUS (0x02) - LilyGO way
    return ft6336ReadBytes(FT6336_REG_TD_STATUS, touchData, 5);
}

void initTouch() {
    // Initialize separate I2C bus for touch (FT6336U on GPIO39/40)
    // T-Watch S3 uses a dedicated I2C bus for touch, separate from PMU/RTC
    TouchWire.begin(TOUCH_SDA, TOUCH_SCL);
    TouchWire.setClock(100000);  // Start at 100kHz for reliability

    // Set up interrupt pin - FT6336U pulls INT LOW when touch detected
    pinMode(TOUCH_INT, INPUT_PULLUP);

    // CRITICAL: T-Watch S3 has NO touch reset pin connected!
    // The FT6336U powers up when ALDO3 is enabled by the AXP2101.
    // We MUST wait sufficient time for the chip to initialize.
    // Reference: LILYGO wiki - "T-Watch-S3-Plus does not have the touch
    // reset pin connected, so if touch is set to sleep, it will not work"
    Serial.println("\n=== TOUCH INIT ===");
    Serial.println("Waiting 500ms for FT6336U power-up (no reset pin)...");
    delay(500);  // Longer delay since we can't reset the chip

    Serial.printf("Touch I2C: SDA=%d, SCL=%d, INT=%d\n", TOUCH_SDA, TOUCH_SCL, TOUCH_INT);
    Serial.printf("INT pin state: %s\n", digitalRead(TOUCH_INT) ? "HIGH (no touch)" : "LOW (touch?)");

    // Scan Touch I2C bus to find touch controller
    i2cFoundCount = 0;
    Serial.println("Scanning Touch I2C bus...");
    for (uint8_t addr = 0x10; addr < 0x78; addr++) {
        TouchWire.beginTransmission(addr);
        if (TouchWire.endTransmission() == 0) {
            if (i2cFoundCount < 16) {
                i2cFound[i2cFoundCount++] = addr;
            }
            Serial.printf("  Found device at 0x%02X\n", addr);
        }
    }
    Serial.printf("Total devices found: %d\n", i2cFoundCount);

    // Check for FT6336U at 0x38
    TouchWire.beginTransmission(FT6336_ADDR);
    if (TouchWire.endTransmission() == 0) {
        touchAddr = FT6336_ADDR;
        isFocaltech = true;
        Serial.println("FT6336U found at 0x38!");

        // Read chip info (with proper delays between reads)
        uint8_t vendorId = ft6336Read(FT6336_REG_VENDOR_ID);
        uint8_t chipId = ft6336Read(FT6336_REG_CHIP_ID);
        uint8_t firmwareId = ft6336Read(FT6336_REG_FIRMWARE_ID);

        Serial.printf("  Vendor ID: 0x%02X (expect 0x11 for FocalTech)\n", vendorId);
        Serial.printf("  Chip ID: 0x%02X (FT6236U/FT6336U=0x64)\n", chipId);
        Serial.printf("  Firmware: 0x%02X\n", firmwareId);

        // Validate chip (but continue even if unexpected - some clones work fine)
        if (vendorId != FOCALTECH_VENDOR_ID) {
            Serial.println("  NOTE: Vendor ID different from expected");
        }
        if (chipId != FT6236U_CHIP_ID && chipId != FT6206_CHIP_ID && chipId != FT6236_CHIP_ID) {
            Serial.printf("  NOTE: Chip ID 0x%02X differs from typical FT6x36\n", chipId);
        }

        // Configure FT6336U - based on aselectroworks and ESPHome libraries
        Serial.println("Configuring touch controller...");

        // CRITICAL: Set device to WORKING mode (not factory test mode)
        // Register 0x00 = 0x00 for working mode
        ft6336Write(FT6336_REG_DEVICE_MODE, 0x00);

        // Set touch threshold (sensitivity)
        // Lower = more sensitive. Default ~128. ESPHome uses 22-50.
        // Too low = ghost touches. Too high = hard to trigger.
        ft6336Write(FT6336_REG_THRESHOLD, 40);  // More sensitive for T-Watch

        // Set active mode touch rate (0x0E = 14 = ~70Hz updates)
        ft6336Write(FT6336_REG_POINT_RATE, 14);

        // CTRL register: 0 = keep active always, 1 = auto-sleep
        // NEVER use auto-sleep since we have no reset pin!
        ft6336Write(FT6336_REG_CTRL, 0x00);

        // G_MODE (interrupt mode):
        // 0x00 = polling mode (INT stays low while touched)
        // 0x01 = trigger mode (INT pulses on touch event)
        // Use trigger mode for better interrupt handling
        ft6336Write(FT6336_REG_G_MODE, 0x01);

        // Power mode: 0 = active, 1 = monitor (low power), 3 = hibernate
        // NEVER use hibernate - no reset pin means we can't wake it!
        ft6336Write(FT6336_REG_POWER_MODE, 0x00);

        delay(100);  // Let settings take effect

        // Verify configuration was written correctly
        uint8_t threshold = ft6336Read(FT6336_REG_THRESHOLD);
        uint8_t gmode = ft6336Read(FT6336_REG_G_MODE);
        uint8_t pmode = ft6336Read(FT6336_REG_POWER_MODE);
        uint8_t devmode = ft6336Read(FT6336_REG_DEVICE_MODE);
        Serial.printf("  Config: DevMode=0x%02X, Threshold=%d, G_MODE=0x%02X, Power=0x%02X\n",
                      devmode, threshold, gmode, pmode);

        // Test read touch data using LilyGO method (5 bytes from reg 0x02)
        Serial.println("Testing touch data read...");
        if (ft6336ReadData()) {
            uint8_t td_status = touchData[0];
            uint8_t numPoints = td_status & 0x0F;
            Serial.printf("  TD_STATUS=0x%02X (touches=%d)\n", td_status, numPoints);
            if (numPoints > 0 && numPoints <= 2) {
                int x = ((touchData[1] & 0x0F) << 8) | touchData[2];
                int y = ((touchData[3] & 0x0F) << 8) | touchData[4];
                Serial.printf("  Touch detected at: (%d, %d)\n", x, y);
            } else {
                Serial.println("  No touch currently detected (good!)");
            }
        } else {
            Serial.println("  WARNING: Test read failed!");
        }

        // Keep I2C at moderate speed (400kHz can be problematic on long traces)
        TouchWire.setClock(200000);
        Serial.println("Touch init complete!");

    } else {
        Serial.println("ERROR: Touch controller NOT FOUND at 0x38!");
        Serial.println("Possible causes:");
        Serial.println("  - ALDO3 not enabled (check PMIC init)");
        Serial.println("  - Wrong I2C pins (should be SDA=39, SCL=40)");
        Serial.println("  - Touch flex cable disconnected");
        touchAddr = 0;
    }

    // Attach interrupt for touch events (G_MODE=0x01 = trigger mode)
    if (touchAddr != 0) {
        attachInterrupt(digitalPinToInterrupt(TOUCH_INT), touchISR, FALLING);
        Serial.printf("Touch INT attached on GPIO%d\n", TOUCH_INT);
        Serial.printf("Current INT state: %s\n", digitalRead(TOUCH_INT) ? "HIGH" : "LOW");
    }
    Serial.println("=================\n");
}

void readTouch() {
    if (touchAddr == 0) return;  // No touch controller

    // LilyGO method: Read 5 bytes starting at TD_STATUS (0x02)
    // This matches TTGO_TWatch_Library FocalTech_Class::getPoint()
    if (!ft6336ReadData()) return;

    // Parse touch data from buffer (LilyGO layout from register 0x02)
    // touchData[0] = TD_STATUS: number of touch points (bits 0-3)
    // touchData[1] = P1_XH: X high byte (bits 0-3) + event flag (bits 6-7)
    // touchData[2] = P1_XL: X low byte
    // touchData[3] = P1_YH: Y high byte (bits 0-3) + touch ID (bits 4-7)
    // touchData[4] = P1_YL: Y low byte

    uint8_t numPoints = touchData[0] & 0x0F;

    // Validate - if 0 or > 2 touch points reported, it's noise or no touch
    if (numPoints == 0 || numPoints > 2) {
        // Also check if interrupt pin is still HIGH (no touch)
        if (digitalRead(TOUCH_INT) == HIGH) {
            // Confirm no touch
            bool wasPressed = touchPressed;
            touchPressed = false;
            if (wasPressed) {
                // Touch was just released - check for swipe
                int dx = touchX - lastTouchX;
                int dy = touchY - lastTouchY;
                if (abs(dx) > 50 || abs(dy) > 50) {
                    swipeDetected = true;
                    if (abs(dx) > abs(dy)) {
                        swipeDir = (dx > 0) ? 1 : -1;
                    } else {
                        swipeDir = (dy > 0) ? 2 : -2;
                    }
                }
            }
            return;
        }
        // INT is low but TD_STATUS says 0 - re-read once more
        numPoints = 0;
    }

    int newX = 0, newY = 0;
    if (numPoints > 0) {
        // Extract coordinates for first touch point (LilyGO way)
        // X = (buffer[1] & 0x0F) << 8 | buffer[2]
        // Y = (buffer[3] & 0x0F) << 8 | buffer[4]
        newX = ((touchData[1] & 0x0F) << 8) | touchData[2];
        newY = ((touchData[3] & 0x0F) << 8) | touchData[4];

        // Validate coordinates are within display range (240x240)
        if (newX > 240 || newY > 240) {
            // Invalid coordinates, ignore this reading
            return;
        }
    }

    bool wasPressed = touchPressed;
    touchPressed = (numPoints > 0);

    if (touchPressed) {
        if (!wasPressed) {
            // New touch started
            lastTouchX = newX;
            lastTouchY = newY;
        }
        touchX = newX;
        touchY = newY;
    } else if (wasPressed) {
        // Touch released - check for swipe gesture
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

// Scanline effect - only on header area (top 20 pixels)
void fbScanlines() {
    for (int y = (frame * 2) % 4; y < 20; y += 4) {  // Only top 20 pixels
        for (int x = 0; x < 240; x++) {
            uint16_t c = fb[y * 240 + x];
            fb[y * 240 + x] = ((c >> 1) & 0x7BEF);
        }
    }
}

// Black Mirror shimmer effect for headings only (call after drawing text)
void fbShimmer(int y, int h) {
    int shimmerX = (frame * 8) % 280 - 20;
    for (int row = y; row < y + h && row < 240; row++) {
        for (int x = 0; x < 240; x++) {
            int dist = abs(x - shimmerX);
            if (dist < 30) {
                uint16_t c = fb[row * 240 + x];
                if (c != BLACK) {
                    int boost = (30 - dist) / 6;
                    uint16_t r = ((c >> 11) & 0x1F) + boost;
                    uint16_t g = ((c >> 5) & 0x3F) + boost * 2;
                    uint16_t b = (c & 0x1F) + boost;
                    r = min((uint16_t)31, r);
                    g = min((uint16_t)63, g);
                    b = min((uint16_t)31, b);
                    fb[row * 240 + x] = (r << 11) | (g << 5) | b;
                }
            }
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

// High contrast bold text (with black outline for readability)
void fbTextBold(int x, int y, const char* str, uint16_t color, int size) {
    fbText(x - 1, y, str, BLACK, size);
    fbText(x + 1, y, str, BLACK, size);
    fbText(x, y - 1, str, BLACK, size);
    fbText(x, y + 1, str, BLACK, size);
    fbText(x, y, str, color, size);
}

void fbTextCenterBold(int y, const char* str, uint16_t color, int size) {
    int len = strlen(str);
    int x = (SCREEN_W - len * (8 * size + size)) / 2;
    fbTextBold(x, y, str, color, size);
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
    pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);  // Backlight ON
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
    Serial.println("Initializing SX1262 LoRa...");
    Serial.printf("  Pins: MOSI=%d MISO=%d SCK=%d CS=%d RST=%d BUSY=%d DIO1=%d\n",
                  LORA_MOSI, LORA_MISO, LORA_SCK, LORA_CS, LORA_RST, LORA_BUSY, LORA_DIO1);

    pinMode(LORA_CS, OUTPUT); pinMode(LORA_RST, OUTPUT);
    pinMode(LORA_BUSY, INPUT); pinMode(LORA_DIO1, INPUT);
    digitalWrite(LORA_CS, HIGH);

    // Hard reset the SX1262
    Serial.println("  Resetting SX1262...");
    digitalWrite(LORA_RST, LOW); delay(20);
    digitalWrite(LORA_RST, HIGH); delay(100);

    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    loraSPI.setFrequency(8000000);

    // Wait for busy to clear (with timeout)
    Serial.print("  Waiting for BUSY...");
    int busyWait = 0;
    while (digitalRead(LORA_BUSY) && busyWait < 100) {
        delay(10);
        busyWait++;
    }
    if (busyWait >= 100) {
        Serial.println(" TIMEOUT! SX1262 may not be present.");
    } else {
        Serial.printf(" OK (%dms)\n", busyWait * 10);
    }

    // Try to read device version to verify SX1262 is present
    // GetStatus command (0xC0) should return status byte
    Serial.print("  Checking SX1262 status...");
    while (digitalRead(LORA_BUSY)) delay(1);
    digitalWrite(LORA_CS, LOW);
    loraSPI.transfer(0xC0);  // GetStatus
    uint8_t status = loraSPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    Serial.printf(" Status=0x%02X\n", status);

    // SetStandby(STDBY_RC)
    uint8_t standby[] = {0x00}; loraWrite(0x80, standby, 1); delay(10);

    // SetPacketType(LORA)
    uint8_t pkt[] = {0x01}; loraWrite(0x8A, pkt, 1);

    // SetRfFrequency(868 MHz)
    uint32_t freq = (uint32_t)(868000000.0 * 33554432.0 / 32000000.0);
    uint8_t frf[] = {(uint8_t)(freq >> 24), (uint8_t)(freq >> 16), (uint8_t)(freq >> 8), (uint8_t)freq};
    loraWrite(0x86, frf, 4);
    Serial.printf("  Frequency: 868MHz (0x%08X)\n", freq);

    // SetPaConfig for SX1262: paDutyCycle=4, hpMax=7, deviceSel=0 (SX1262), paLut=1
    uint8_t pa[] = {0x04, 0x07, 0x00, 0x01}; loraWrite(0x95, pa, 4);

    // SetTxParams: power=22dBm, rampTime=200us
    uint8_t tx[] = {0x16, 0x04}; loraWrite(0x8E, tx, 2);

    // SetModulationParams: SF7, BW125, CR4/5, LowDataRateOptimize=OFF
    uint8_t mod[] = {0x07, 0x04, 0x01, 0x00}; loraWrite(0x8B, mod, 4);

    // SetPacketParams: preamble=12, header=variable, payloadLen=255, CRC=on, invertIQ=off
    uint8_t pktParams[] = {0x00, 0x0C, 0x00, 0xFF, 0x01, 0x00};
    loraWrite(0x8C, pktParams, 6);

    // SetDioIrqParams: enable RxDone and TxDone on DIO1
    uint8_t dioIrq[] = {0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00};
    loraWrite(0x08, dioIrq, 8);

    // Start continuous RX mode
    uint8_t rxcmd[] = {0xFF, 0xFF, 0xFF};
    loraWrite(0x82, rxcmd, 3);

    // Get unique ID from MAC
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
    pairID = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5];

    Serial.printf("LoRa init complete! pairID: 0x%08X\n", pairID);
}

void loraSend(uint8_t* data, int len) {
    // Set to standby before TX
    uint8_t standby[] = {0x00}; loraWrite(0x80, standby, 1);
    while (digitalRead(LORA_BUSY)) delay(1);

    // Write data to FIFO
    uint8_t buf[256]; buf[0] = 0x00;  // Offset = 0
    memcpy(buf + 1, data, len);
    loraWrite(0x0E, buf, len + 1);

    // Set payload length
    uint8_t params[] = {0x00, 0x0C, 0x00, (uint8_t)len, 0x01, 0x00};
    loraWrite(0x8C, params, 6);

    // Clear IRQ
    uint8_t clr[] = {0xFF, 0xFF}; loraWrite(0x02, clr, 2);

    // Transmit
    uint8_t txcmd[] = {0x00, 0x00, 0x00};  // No timeout
    loraWrite(0x83, txcmd, 3);

    // Wait for TX done
    for (int i = 0; i < 100; i++) {
        uint8_t irq = loraRead(0x12);
        if (irq & 0x01) break;  // TxDone
        delay(5);
    }

    // Clear IRQ and return to RX mode
    loraWrite(0x02, clr, 2);
    uint8_t rxcmd[] = {0xFF, 0xFF, 0xFF};
    loraWrite(0x82, rxcmd, 3);
}

int loraReceive(uint8_t* data, int maxLen) {
    // Check IRQ status - bit 1 = RxDone
    uint8_t irq = loraRead(0x12);
    if (irq & 0x02) {
        // Wait for busy
        while (digitalRead(LORA_BUSY)) delay(1);

        // Get RX buffer status
        digitalWrite(LORA_CS, LOW);
        loraSPI.transfer(0x13); loraSPI.transfer(0x00);
        uint8_t len = loraSPI.transfer(0x00);
        uint8_t start = loraSPI.transfer(0x00);
        digitalWrite(LORA_CS, HIGH);

        if (len > 0 && len <= maxLen) {
            // Read buffer
            while (digitalRead(LORA_BUSY)) delay(1);
            digitalWrite(LORA_CS, LOW);
            loraSPI.transfer(0x1E); loraSPI.transfer(start); loraSPI.transfer(0x00);
            for (int i = 0; i < len; i++) data[i] = loraSPI.transfer(0x00);
            digitalWrite(LORA_CS, HIGH);

            // Get RSSI
            msgRSSI = -loraRead(0x14) / 2;

            // Clear IRQ
            uint8_t clr[] = {0xFF, 0xFF}; loraWrite(0x02, clr, 2);

            Serial.printf("LoRa RX: %d bytes, RSSI %d\n", len, msgRSSI);
            return len;
        }
        // Clear IRQ even if invalid
        uint8_t clr[] = {0xFF, 0xFF}; loraWrite(0x02, clr, 2);
    }
    return 0;
}

// ============= Protocol =============
#define MSG_PAIR_REQ    0x01
#define MSG_PAIR_ACK    0x02
#define MSG_PING        0x03
#define MSG_PONG        0x04
#define MSG_VOICE_START 0x05  // Voice transmission start
#define MSG_VOICE_DATA  0x06  // Voice data chunk
#define MSG_VOICE_END   0x07  // Voice transmission end
#define MSG_LOCATION    0x08
#define MSG_SENSORS     0x09  // Share sensor readings
#define MSG_TRIANG_REQ  0x0A  // Request partner's RSSI for MAC
#define MSG_TRIANG_RSP  0x0B  // Response with RSSI data
#define MSG_HEARTBEAT   0x0C  // Keep-alive with basic status

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

// Send heartbeat with status every few seconds
void sendHeartbeat() {
    if (!loraPaired) return;
    uint8_t pkt[16];
    pkt[0] = MSG_HEARTBEAT;
    memcpy(pkt + 1, &pairID, 4);
    pkt[5] = (uint8_t)heartRate;
    pkt[6] = (uint8_t)hrv;
    pkt[7] = (uint8_t)stress;
    pkt[8] = 85;  // Battery % placeholder
    loraSend(pkt, 9);
}

// Send sensor data including nearest entity for triangulation
void sendSensorData() {
    if (!loraPaired) return;
    uint8_t pkt[32];
    pkt[0] = MSG_SENSORS;
    memcpy(pkt + 1, &pairID, 4);
    pkt[5] = (uint8_t)heartRate;
    pkt[6] = (uint8_t)hrv;
    pkt[7] = (uint8_t)stress;
    pkt[8] = 85;  // Battery
    // Include nearest entity info for triangulation
    if (entityCount > 0) {
        memcpy(pkt + 9, entities[0].mac, 6);
        pkt[15] = (uint8_t)(-entities[0].rssi);  // Send as positive
        float dist = entities[0].distanceM;
        memcpy(pkt + 16, &dist, 4);
        loraSend(pkt, 20);
    } else {
        loraSend(pkt, 9);
    }
}

// Request partner to report RSSI for a specific MAC (triangulation)
void sendTriangRequest(uint8_t* targetMAC) {
    if (!loraPaired) return;
    uint8_t pkt[16];
    pkt[0] = MSG_TRIANG_REQ;
    memcpy(pkt + 1, &pairID, 4);
    memcpy(pkt + 5, targetMAC, 6);
    loraSend(pkt, 11);
}

// Respond with our RSSI reading for requested MAC
void sendTriangResponse(uint8_t* targetMAC, int rssi, float dist) {
    if (!loraPaired) return;
    uint8_t pkt[20];
    pkt[0] = MSG_TRIANG_RSP;
    memcpy(pkt + 1, &pairID, 4);
    memcpy(pkt + 5, targetMAC, 6);
    pkt[11] = (uint8_t)(-rssi);
    memcpy(pkt + 12, &dist, 4);
    loraSend(pkt, 16);
}

// Voice transmission functions
void sendVoiceStart(int totalSamples) {
    if (!loraPaired) return;
    uint8_t pkt[12];
    pkt[0] = MSG_VOICE_START;
    memcpy(pkt + 1, &pairID, 4);
    int16_t samples = (int16_t)totalSamples;
    memcpy(pkt + 5, &samples, 2);
    int16_t chunks = (totalSamples * 2 + VOICE_CHUNK_SIZE - 1) / VOICE_CHUNK_SIZE;
    memcpy(pkt + 7, &chunks, 2);
    loraSend(pkt, 9);
}

void sendVoiceChunk(int chunkNum, uint8_t* data, int len) {
    if (!loraPaired) return;
    uint8_t pkt[VOICE_CHUNK_SIZE + 8];
    pkt[0] = MSG_VOICE_DATA;
    memcpy(pkt + 1, &pairID, 4);
    pkt[5] = (uint8_t)(chunkNum >> 8);
    pkt[6] = (uint8_t)(chunkNum & 0xFF);
    pkt[7] = (uint8_t)len;
    memcpy(pkt + 8, data, len);
    loraSend(pkt, 8 + len);
}

void sendVoiceEnd() {
    if (!loraPaired) return;
    uint8_t pkt[8];
    pkt[0] = MSG_VOICE_END;
    memcpy(pkt + 1, &pairID, 4);
    loraSend(pkt, 5);
}

// Transmit recorded voice over LoRa (compressed)
void transmitVoice() {
    if (voiceLen == 0) return;

    // Downsample and compress: keep every 4th sample, convert to 8-bit
    int compLen = voiceLen / 4;
    uint8_t* compressed = (uint8_t*)malloc(compLen);
    if (!compressed) return;

    for (int i = 0; i < compLen; i++) {
        int16_t sample = voiceBuffer[i * 4];
        compressed[i] = (uint8_t)((sample >> 8) + 128);  // Convert to unsigned 8-bit
    }

    sendVoiceStart(compLen);
    delay(100);

    int chunks = (compLen + VOICE_CHUNK_SIZE - 1) / VOICE_CHUNK_SIZE;
    for (int c = 0; c < chunks; c++) {
        int offset = c * VOICE_CHUNK_SIZE;
        int len = min(VOICE_CHUNK_SIZE, compLen - offset);
        sendVoiceChunk(c, compressed + offset, len);
        delay(80);  // Allow LoRa to process
    }

    sendVoiceEnd();
    free(compressed);
}

// Temporary buffer for receiving voice data
uint8_t voiceRxBuffer[VOICE_BUF_SIZE];
int voiceRxLen = 0;
int voiceRxExpected = 0;
bool voiceRxInProgress = false;

// Triangulation: calculate position using two distance measurements
// Uses circle intersection algorithm
void triangulateTarget(TriangTarget* t) {
    if (t->myDist <= 0 || t->partnerDist <= 0 || partnerDist <= 0) {
        t->valid = false;
        return;
    }

    // We're at origin (0,0), partner is at (partnerDist, 0)
    // Target is at intersection of two circles
    float d = partnerDist;  // Distance between us and partner
    float r1 = t->myDist;   // Our distance to target
    float r2 = t->partnerDist;  // Partner's distance to target

    // Check if triangulation is possible
    if (d > r1 + r2 || d < fabs(r1 - r2)) {
        t->valid = false;
        return;
    }

    // Calculate intersection point (there are 2, we pick the one closer to midpoint)
    float a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    float h = sqrt(r1 * r1 - a * a);

    // Two possible positions
    float x1 = a, y1 = h;
    float x2 = a, y2 = -h;

    // Pick the one that makes more sense (usually positive y = in front)
    t->estimatedX = x1;
    t->estimatedY = (y1 > 0) ? y1 : y2;
    t->valid = true;
}

void processLoRa() {
    uint8_t buf[256];
    int len = loraReceive(buf, 256);
    if (len > 0) {
        uint8_t type = buf[0];
        uint32_t fromID;
        memcpy(&fromID, buf + 1, 4);
        partnerRSSI = msgRSSI;  // Store RSSI of last partner message

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
                    // Calculate distance using RSSI + GPS hybrid
                    float dlat = (partnerLat - myLat) * 111000;
                    float dlon = (partnerLon - myLon) * 111000 * cos(myLat * DEG_TO_RAD);
                    float gpsDist = sqrt(dlat * dlat + dlon * dlon);
                    float rssiDist = rssiToDistance(partnerRSSI);
                    // Weighted average: trust RSSI more at close range
                    partnerDist = (gpsDist < 100) ? (rssiDist * 0.7 + gpsDist * 0.3) : gpsDist;
                    partnerBearing = atan2(dlon, dlat) * RAD_TO_DEG;
                }
                break;

            case MSG_HEARTBEAT:
                if (loraPaired && fromID == partnerID) {
                    partnerSensors.heartRate = buf[5];
                    partnerSensors.hrv = buf[6];
                    partnerSensors.stress = buf[7];
                    partnerSensors.batteryPct = buf[8];
                    partnerSensors.valid = true;
                    lastPartnerUpdate = millis();
                }
                break;

            case MSG_SENSORS:
                if (loraPaired && fromID == partnerID) {
                    partnerSensors.heartRate = buf[5];
                    partnerSensors.hrv = buf[6];
                    partnerSensors.stress = buf[7];
                    partnerSensors.batteryPct = buf[8];
                    partnerSensors.valid = true;
                    lastPartnerUpdate = millis();

                    // If includes entity data for triangulation
                    if (len >= 20) {
                        memcpy(partnerSensors.nearestEntityMAC, buf + 9, 6);
                        partnerSensors.nearestEntityRSSI = -(int)buf[15];
                        memcpy(&partnerSensors.nearestDist, buf + 16, 4);

                        // Check if we also see this entity
                        for (int i = 0; i < entityCount; i++) {
                            if (memcmp(entities[i].mac, partnerSensors.nearestEntityMAC, 6) == 0) {
                                // Found same entity! Add to triangulation targets
                                bool found = false;
                                for (int t = 0; t < triangTargetCount; t++) {
                                    if (memcmp(triangTargets[t].mac, entities[i].mac, 6) == 0) {
                                        triangTargets[t].myRSSI = entities[i].rssi;
                                        triangTargets[t].partnerRSSI = partnerSensors.nearestEntityRSSI;
                                        triangTargets[t].myDist = entities[i].distanceM;
                                        triangTargets[t].partnerDist = partnerSensors.nearestDist;
                                        triangTargets[t].lastSeen = millis();
                                        triangulateTarget(&triangTargets[t]);
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found && triangTargetCount < MAX_TRIANG_TARGETS) {
                                    TriangTarget* t = &triangTargets[triangTargetCount++];
                                    memcpy(t->mac, entities[i].mac, 6);
                                    t->myRSSI = entities[i].rssi;
                                    t->partnerRSSI = partnerSensors.nearestEntityRSSI;
                                    t->myDist = entities[i].distanceM;
                                    t->partnerDist = partnerSensors.nearestDist;
                                    t->lastSeen = millis();
                                    triangulateTarget(t);
                                }
                                break;
                            }
                        }
                    }
                }
                break;

            case MSG_TRIANG_REQ:
                if (loraPaired && fromID == partnerID) {
                    // Partner wants our RSSI for a specific MAC
                    uint8_t targetMAC[6];
                    memcpy(targetMAC, buf + 5, 6);
                    for (int i = 0; i < entityCount; i++) {
                        if (memcmp(entities[i].mac, targetMAC, 6) == 0) {
                            sendTriangResponse(targetMAC, entities[i].rssi, entities[i].distanceM);
                            break;
                        }
                    }
                }
                break;

            case MSG_TRIANG_RSP:
                if (loraPaired && fromID == partnerID) {
                    uint8_t targetMAC[6];
                    memcpy(targetMAC, buf + 5, 6);
                    int rssi = -(int)buf[11];
                    float dist;
                    memcpy(&dist, buf + 12, 4);

                    // Update triangulation target
                    for (int t = 0; t < triangTargetCount; t++) {
                        if (memcmp(triangTargets[t].mac, targetMAC, 6) == 0) {
                            triangTargets[t].partnerRSSI = rssi;
                            triangTargets[t].partnerDist = dist;
                            triangTargets[t].lastSeen = millis();
                            triangulateTarget(&triangTargets[t]);
                            break;
                        }
                    }
                }
                break;

            case MSG_VOICE_START:
                if (loraPaired && fromID == partnerID) {
                    int16_t samples;
                    memcpy(&samples, buf + 5, 2);
                    voiceRxExpected = samples;
                    voiceRxLen = 0;
                    voiceRxInProgress = true;
                    hasNewMessage = true;
                }
                break;

            case MSG_VOICE_DATA:
                if (loraPaired && fromID == partnerID && voiceRxInProgress) {
                    int chunkNum = (buf[5] << 8) | buf[6];
                    int chunkLen = buf[7];
                    int offset = chunkNum * VOICE_CHUNK_SIZE;
                    if (offset + chunkLen <= VOICE_BUF_SIZE) {
                        memcpy(voiceRxBuffer + offset, buf + 8, chunkLen);
                        voiceRxLen = max(voiceRxLen, offset + chunkLen);
                    }
                }
                break;

            case MSG_VOICE_END:
                if (loraPaired && fromID == partnerID && voiceRxInProgress) {
                    voiceRxInProgress = false;
                    // Decompress: expand 8-bit to 16-bit, upsample 4x
                    playLen = 0;
                    for (int i = 0; i < voiceRxLen && playLen < VOICE_BUF_SIZE; i++) {
                        int16_t sample = ((int16_t)voiceRxBuffer[i] - 128) << 8;
                        // Upsample 4x with simple interpolation
                        for (int j = 0; j < 4 && playLen < VOICE_BUF_SIZE; j++) {
                            playBuffer[playLen++] = sample;
                        }
                    }
                    playPos = 0;
                    hasNewMessage = true;
                    voicePacketsReceived++;

                    // Store in message queue
                    if (voiceMsgCount < MAX_VOICE_MSGS) {
                        VoiceMessage* msg = &voiceMsgs[voiceMsgCount++];
                        memcpy(msg->samples, playBuffer, playLen * 2);
                        msg->length = playLen;
                        msg->rssi = partnerRSSI;
                        msg->timestamp = millis();
                        msg->played = false;
                    }
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
        float angle = (mac[0] + mac[5]) * 0.1 + PI;  // +PI to flip radar to correct orientation
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

    // Subtle background pulse rings
    for (int i = 0; i < 2; i++) {
        int r = 50 + ((frame + i * 30) % 50);
        fbCircle(120, 70, r, GRID_DIM);
    }

    char buf[32];

    // TIME - size 4 fits perfectly, shimmer on heading only
    sprintf(buf, "%02d:%02d", rtcHour, rtcMin);
    fbTextCenterBold(20, buf, WHITE, 4);
    fbShimmer(20, 40);  // Shimmer on time only

    // Date under time
    const char* months[] = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
    sprintf(buf, "%02d %s 20%02d", rtcDay, months[rtcMonth-1], rtcYear);
    fbTextCenter(70, buf, NEON_CYAN, 2);

    // Seconds
    sprintf(buf, ":%02d", rtcSec);
    fbTextCenter(100, buf, (rtcSec % 2) ? NEON_PINK : NEON_PURPLE, 2);

    // Status bar
    fbLine(0, 125, 240, 125, NEON_CYAN);

    // Stats row - fits on screen
    sprintf(buf, "%d", heartRate);
    fbTextBold(15, 135, buf, NEON_RED, 3);
    fbText(70, 150, "BPM", WHITE, 1);

    // Nearby count (excludes self)
    int nearbyCount = 0;
    for (int i = 0; i < entityCount; i++) {
        if (entities[i].distanceM > WEARER_OFFSET_M) nearbyCount++;
    }
    sprintf(buf, "%d", nearbyCount);
    fbTextBold(130, 135, buf, NEON_GREEN, 3);
    fbText(180, 150, "NEAR", WHITE, 1);

    // Bottom info
    fbLine(0, 175, 240, 175, NEON_PURPLE);
    fbText(10, 185, "YOU: 0.6M", NEON_PINK, 1);

    if (loraPaired) {
        fbFillCircle(210, 195, 8, NEON_GREEN);
        fbText(170, 210, "LINKED", NEON_GREEN, 1);
    } else {
        fbCircle(210, 195, 8, DIM_GREEN);
        fbText(165, 210, "UNPAIRED", DIM_GREEN, 1);
    }

    sprintf(buf, "%d TOTAL", entityCount);
    fbText(10, 210, buf, NEON_CYAN, 1);
    fbScanlines();
}

void face_WalkieTalkie() {
    fbClear(BLACK);
    char buf[40];

    if (!loraPaired) {
        // Auto-pairing mode - always searching
        fbTextCenter(5, "WALKIE TALKIE", NEON_ORANGE, 2);

        // Animated radar sweep while searching
        int sweep = (frame * 3) % 360;
        for (int r = 20; r < 80; r += 20) {
            fbCircle(120, 110, r, DIM_CYAN);
        }
        // Sweep line
        float ang = sweep * DEG_TO_RAD;
        int ex = 120 + cos(ang) * 70;
        int ey = 110 + sin(ang) * 70;
        fbLine(120, 110, ex, ey, NEON_GREEN);

        // Pulsing search indicator
        int pulse = (frame % 30) < 15 ? NEON_ORANGE : DIM_CYAN;
        fbTextCenter(50, "SEARCHING...", pulse, 2);
        fbTextCenter(180, "AUTO-PAIRING", NEON_CYAN, 1);

        sprintf(buf, "MY ID: %08X", pairID);
        fbTextCenter(200, buf, DIM_GREEN, 1);

        // Send pair request periodically
        if (frame % 60 == 0) {
            sendPairRequest();
        }
    } else {
        // PAIRED - Full walkie-talkie interface
        fbTextCenter(2, "LINKED", NEON_GREEN, 2);

        // Partner status bar
        sprintf(buf, "PARTNER: %08X", partnerID);
        fbText(5, 22, buf, DIM_CYAN, 1);

        // Partner vitals (received via LoRa)
        if (partnerSensors.valid) {
            sprintf(buf, "HR:%d HRV:%d", partnerSensors.heartRate, partnerSensors.hrv);
            fbText(5, 34, buf, NEON_GREEN, 1);
            sprintf(buf, "BAT:%d%%", partnerSensors.batteryPct);
            fbText(170, 34, buf, partnerSensors.batteryPct > 20 ? NEON_GREEN : NEON_RED, 1);
        }

        // Distance and bearing compass
        fbLine(0, 48, 240, 48, DIM_CYAN);

        // Mini compass showing partner direction
        int cx = 60, cy = 90;
        fbCircle(cx, cy, 35, DIM_CYAN);
        fbCircle(cx, cy, 36, DIM_CYAN);
        float bRad = partnerBearing * DEG_TO_RAD;
        int px = cx + cos(bRad - PI/2) * 30;
        int py = cy + sin(bRad - PI/2) * 30;
        fbFillCircle(px, py, 6, NEON_GREEN);
        fbText(cx - 4, cy - 4, "N", WHITE, 1);

        // Distance display
        sprintf(buf, "%.1fm", partnerDist);
        fbText(110, 70, buf, NEON_CYAN, 2);
        sprintf(buf, "%d dBm", partnerRSSI);
        fbText(110, 95, buf, partnerRSSI > -70 ? NEON_GREEN : NEON_YELLOW, 1);

        // Signal strength bars
        int bars = (partnerRSSI > -50) ? 5 : (partnerRSSI > -60) ? 4 : (partnerRSSI > -70) ? 3 : (partnerRSSI > -80) ? 2 : 1;
        for (int i = 0; i < 5; i++) {
            int bh = 5 + i * 4;
            uint16_t col = (i < bars) ? NEON_GREEN : GRID_DIM;
            for (int y = 0; y < bh; y++) fbLine(200 + i * 8, 100 - y, 205 + i * 8, 100 - y, col);
        }

        fbLine(0, 115, 240, 115, DIM_CYAN);

        // PTT (Push-To-Talk) button area
        if (isRecording) {
            // Recording animation
            int pulse = 25 + sin(frame * 0.3) * 10;
            fbFillCircle(120, 160, pulse, NEON_RED);
            fbTextCenter(155, "REC", BLACK, 2);
            int elapsed = (millis() - recordStart) / 1000;
            sprintf(buf, "%ds / 5s", elapsed);
            fbTextCenter(195, buf, NEON_ORANGE, 1);
        } else if (hasNewMessage) {
            // New message indicator
            int pulse = 30 + sin(frame * 0.2) * 8;
            fbFillCircle(120, 160, pulse, NEON_ORANGE);
            fbTextCenter(155, "NEW", BLACK, 2);
            fbTextCenter(195, "TAP TO PLAY", NEON_CYAN, 1);
        } else {
            // Ready state
            fbCircle(120, 160, 30, NEON_GREEN);
            fbCircle(120, 160, 28, NEON_GREEN);
            fbTextCenter(155, "PTT", NEON_GREEN, 2);
            fbTextCenter(195, "HOLD TO TALK", DIM_CYAN, 1);
        }

        // Voice message count
        sprintf(buf, "MSGS:%d", voiceMsgCount);
        fbText(5, 220, buf, voiceMsgCount > 0 ? NEON_ORANGE : DIM_CYAN, 1);

        // Triangulated targets indicator
        if (triangTargetCount > 0) {
            sprintf(buf, "TRI:%d", triangTargetCount);
            fbText(180, 220, buf, NEON_PURPLE, 1);
        }
    }

    // Always send sensor data when paired
    if (loraPaired && frame % 120 == 0) {
        sendSensorData();
    }
}

// Dual-watch triangulation face - find targets through walls
void face_Triangulation() {
    fbClear(BLACK);
    fbTextCenter(2, "TRIANGULATION", NEON_PURPLE, 2);
    char buf[40];

    if (!loraPaired) {
        fbTextCenter(100, "NEED PARTNER", NEON_RED, 2);
        fbTextCenter(130, "Pair another watch", DIM_CYAN, 1);
        fbTextCenter(150, "to triangulate", DIM_CYAN, 1);
        return;
    }

    // Draw coordinate system with us at center, partner to the right
    int cx = 120, cy = 130;
    int scale = 3;  // pixels per meter

    // Grid lines
    for (int x = 20; x < 220; x += 40) fbLine(x, 50, x, 210, GRID_DIM);
    for (int y = 50; y < 210; y += 40) fbLine(20, y, 220, y, GRID_DIM);

    // Us (blue circle at center)
    fbFillCircle(cx, cy, 8, NEON_CYAN);
    fbText(cx - 8, cy + 12, "YOU", NEON_CYAN, 1);

    // Partner (green circle)
    int partnerX = cx + min(80, (int)(partnerDist * scale));
    fbFillCircle(partnerX, cy, 8, NEON_GREEN);
    fbText(partnerX - 12, cy + 12, "PAIR", NEON_GREEN, 1);

    // Draw line between us and partner
    fbLine(cx, cy, partnerX, cy, DIM_GREEN);

    // Status bar
    sprintf(buf, "DIST:%.1fm RSSI:%ddB", partnerDist, partnerRSSI);
    fbText(5, 22, buf, NEON_GREEN, 1);

    // Draw triangulated targets
    int targetsDrawn = 0;
    for (int i = 0; i < triangTargetCount && targetsDrawn < 5; i++) {
        TriangTarget* t = &triangTargets[i];
        if (!t->valid || millis() - t->lastSeen > 10000) continue;

        // Scale position to screen coordinates
        int tx = cx + (int)(t->estimatedX * scale);
        int ty = cy - (int)(t->estimatedY * scale);  // Y inverted

        // Clamp to screen
        tx = max(25, min(215, tx));
        ty = max(55, min(205, ty));

        // Draw target as red triangle
        fbFillCircle(tx, ty, 6, NEON_RED);

        // Draw distance circles from both watchers
        int r1 = (int)(t->myDist * scale);
        int r2 = (int)(t->partnerDist * scale);
        if (r1 < 100) fbCircle(cx, cy, r1, DIM_CYAN);
        if (r2 < 100) fbCircle(partnerX, cy, r2, DIM_GREEN);

        // Target info
        sprintf(buf, "%02X:%02X", t->mac[4], t->mac[5]);
        fbText(tx - 12, ty - 12, buf, NEON_ORANGE, 1);

        targetsDrawn++;
    }

    // Legend
    sprintf(buf, "TARGETS: %d", triangTargetCount);
    fbText(5, 215, buf, NEON_ORANGE, 1);

    // Instructions
    if (triangTargetCount == 0) {
        fbTextCenter(180, "Walk around to", DIM_CYAN, 1);
        fbTextCenter(195, "triangulate targets", DIM_CYAN, 1);
    }

    // Request triangulation data periodically
    if (frame % 90 == 0 && entityCount > 0) {
        sendTriangRequest(entities[0].mac);
    }
}

void face_Vitals() {
    fbClear(BLACK);

    // Heading with shimmer only
    fbTextCenter(3, "VITALS", NEON_CYAN, 2);
    fbShimmer(3, 18);

    // Animated heart
    float pulse = sin(frame * 0.15) * 0.15 + 1.0;
    fbFillCircle(40, 40, 10 * pulse, NEON_RED);
    fbFillCircle(55, 40, 10 * pulse, NEON_RED);

    // Heart rate - size 3 fits
    heartRate = 68 + sin(frame * 0.1) * 4;
    char buf[32];
    sprintf(buf, "%d", heartRate);
    fbTextBold(20, 55, buf, WHITE, 3);
    fbText(75, 65, "BPM", NEON_RED, 1);

    // HRV
    hrv = 38 + sin(frame * 0.08) * 8;
    sprintf(buf, "HRV:%dMS", hrv);
    fbText(130, 40, buf, NEON_GREEN, 1);
    fbText(130, 52, "RMSSD", DIM_CYAN, 1);

    // ECG wave
    for (int x = 10; x < 230; x++) {
        int phase = (x + frame * 3) % 70;
        int y = 100;
        if (phase > 20 && phase < 25) y = 100 - (phase - 20) * 10;
        else if (phase >= 25 && phase < 30) y = 50 + (phase - 25) * 14;
        fbPixel(x, y, NEON_GREEN);
        fbPixel(x, y + 1, NEON_GREEN);
    }

    // Stress bar
    stress = 30 + sin(frame * 0.05) * 15;
    sprintf(buf, "STRESS %d%%", (int)stress);
    fbText(10, 120, buf, WHITE, 1);
    fbRect(10, 135, (int)(stress * 2), 8, stress < 50 ? NEON_GREEN : NEON_RED);
    fbRect(10, 135, 200, 8, GRID_DIM);

    // SpO2
    int spo2 = 97 + sin(frame * 0.07) * 1;
    sprintf(buf, "SPO2: %d%%", spo2);
    fbText(10, 155, buf, NEON_CYAN, 1);

    // Breath rate
    sprintf(buf, "BREATH: %d/MIN", breathRate);
    fbText(120, 155, buf, NEON_GREEN, 1);

    fbLine(0, 175, 240, 175, NEON_PURPLE);

    // Nearest from heart
    float nearestFromHeart = nearestEntityM < 50 ? nearestEntityM + 0.6 : 0;
    sprintf(buf, "NEAREST: %.1fM", nearestFromHeart);
    fbText(10, 185, buf, NEON_CYAN, 2);

    fbText(10, 215, "REF: SHAFFER 2017", DIM_CYAN, 1);
    fbText(140, 215, "PORGES 2011", DIM_CYAN, 1);
    fbScanlines();
}

void face_Radar() {
    fbClear(BLACK);

    // Heading with shimmer only
    const char* title = droneCount > 0 ? "DRONE ALERT" : "RADAR";
    uint16_t titleCol = droneCount > 0 ? NEON_RED : NEON_CYAN;
    fbTextCenter(3, title, titleCol, 2);
    fbShimmer(3, 18);

    int cx = 120, cy = 105;

    // Radar rings
    for (int r = 25; r <= 70; r += 22) {
        fbCircle(cx, cy, r, GRID_DIM);
    }
    fbLine(cx - 75, cy, cx + 75, cy, GRID_DIM);
    fbLine(cx, cy - 75, cx, cy + 75, GRID_DIM);

    // Sweep
    float sweep = frame * 0.08;
    for (int i = 0; i < 12; i++) {
        float a = sweep - i * 0.04;
        uint16_t col = (180 - i * 14) >> 3 << 6;
        fbLine(cx, cy, cx + cos(a) * 70, cy + sin(a) * 70, col);
    }

    // Entities
    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen > 30000) continue;
        int ex = entities[i].x, ey = entities[i].y - 15;
        float p = sin(frame * 0.2 + i) * 2;
        if (entities[i].isDrone) {
            fbFillCircle(ex, ey, 5 + p, NEON_RED);
        } else {
            fbFillCircle(ex, ey, 4 + p, NEON_GREEN);
        }
    }

    // You in center
    fbFillCircle(cx, cy, 5, NEON_PINK);

    // Stats - fits on screen
    fbLine(0, 185, 240, 185, NEON_CYAN);
    char buf[32];
    sprintf(buf, "%d", entityCount);
    fbTextBold(15, 195, buf, WHITE, 3);
    fbText(65, 205, "ENTITIES", NEON_GREEN, 1);

    sprintf(buf, "%d DRONES", droneCount);
    fbText(130, 200, buf, droneCount > 0 ? NEON_RED : DIM_GREEN, 1);
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
    fbTextCenter(140, "V15.0", NEON_PINK, 2);

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

// ============= NEW FACE 19: BODY SCANNER =============
// Bioimpedance Analysis visualization (Kyle 2004, Lukaski 1987)
void face_BodyScanner() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "BODY SCAN", NEON_CYAN, 2);
    fbShimmer(2, 18);

    // Simulate BIA measurements from wearer's baseline
    if (!bodyScan.calibrated) {
        // Initialize with typical values (Watson formula)
        bodyScan.totalBodyWater = 55.0 + sin(frame * 0.01) * 2;
        bodyScan.extracellularWater = bodyScan.totalBodyWater * 0.45;
        bodyScan.intracellularWater = bodyScan.totalBodyWater * 0.55;
        bodyScan.bodyFatPercent = 18.0 + sin(frame * 0.02) * 3;
        bodyScan.visceralFat = 6.0 + sin(frame * 0.015) * 2;
        bodyScan.adipocyteHydration = 0.15 + sin(frame * 0.03) * 0.05;
        bodyScan.lipidDropletSize = 45.0 + sin(frame * 0.025) * 15;
        bodyScan.phaseAngle = 6.5 + sin(frame * 0.02) * 1.5;
        bodyScan.leanMass = 58.0;
        bodyScan.skeletalMuscle = 42.0;
    }

    // Body outline with scanning line
    int bx = 120, by = 130;

    // Draw body silhouette
    fbFillCircle(bx, by - 55, 18, DIM_CYAN);  // Head
    fbRect(bx - 25, by - 35, 50, 60, DIM_CYAN);  // Torso
    fbRect(bx - 40, by - 30, 15, 45, DIM_CYAN);  // Left arm
    fbRect(bx + 25, by - 30, 15, 45, DIM_CYAN);  // Right arm
    fbRect(bx - 20, by + 25, 15, 50, DIM_CYAN);  // Left leg
    fbRect(bx + 5, by + 25, 15, 50, DIM_CYAN);  // Right leg

    // Animated scan line
    scanLineY = (frame * 3) % 140;
    int scanY = by - 70 + scanLineY;
    fbLine(bx - 50, scanY, bx + 50, scanY, NEON_GREEN);
    fbLine(bx - 50, scanY + 1, bx + 50, scanY + 1, NEON_CYAN);

    // Tissue layer visualization (left side)
    int layerY = 25;
    for (int i = 0; i < 6; i++) {
        int barW = tissueStack[i].fluidContent * 30;
        fbRect(5, layerY, barW, 8, tissueStack[i].color);
        fbText(40, layerY, tissueStack[i].name, tissueStack[i].color, 1);
        layerY += 12;
    }

    // Stats on right side
    sprintf(buf, "TBW:%.0f%%", bodyScan.totalBodyWater);
    fbText(160, 25, buf, NEON_CYAN, 1);

    sprintf(buf, "FAT:%.0f%%", bodyScan.bodyFatPercent);
    fbText(160, 38, buf, NEON_YELLOW, 1);

    sprintf(buf, "VISC:%d", (int)bodyScan.visceralFat);
    fbText(160, 51, buf, NEON_ORANGE, 1);

    sprintf(buf, "PHI:%.1f", bodyScan.phaseAngle);
    fbText(160, 64, buf, NEON_GREEN, 1);

    // Fat cell fluid analysis (bottom section)
    fbLine(0, 195, 240, 195, NEON_PURPLE);
    fbText(5, 200, "ADIPOCYTE FLUID", NEON_PINK, 1);

    // Fat cell visualization
    for (int i = 0; i < 5; i++) {
        int cx = 30 + i * 45;
        int cy = 220;
        int r = 8 + sin(frame * 0.1 + i) * 2;

        // Cell membrane
        fbCircle(cx, cy, r, NEON_YELLOW);

        // Lipid droplet (yellow center)
        int lipidR = r * (1.0 - bodyScan.adipocyteHydration);
        fbFillCircle(cx, cy, lipidR, NEON_ORANGE);

        // Water content (blue ring)
        if (bodyScan.adipocyteHydration > 0.1) {
            fbCircle(cx, cy, r - 2, NEON_CYAN);
        }
    }

    sprintf(buf, "H2O:%.0f%%", bodyScan.adipocyteHydration * 100);
    fbText(5, 215, buf, NEON_CYAN, 1);

    sprintf(buf, "DROP:%duM", (int)bodyScan.lipidDropletSize);
    fbText(160, 215, buf, NEON_YELLOW, 1);

    fbGlitch(1);
    fbScanlines();
}

// ============= NEW FACE 20: SONAR EMULATION =============
// WiFi Time-of-Flight based distance mapping (Ma 2019, Bahl 2000)
void face_Sonar() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "SONAR", NEON_GREEN, 2);
    fbShimmer(2, 18);

    int cx = 120, cy = 130;

    // Update sonar sweep
    sonarSweepAngle += 0.08;
    if (sonarSweepAngle > 2 * PI) sonarSweepAngle -= 2 * PI;

    // Draw sonar display (submarine style)
    // Range rings
    for (int r = 20; r <= 80; r += 20) {
        fbCircle(cx, cy, r, GRID_DIM);
    }

    // Cross hairs
    fbLine(cx - 85, cy, cx + 85, cy, GRID_DIM);
    fbLine(cx, cy - 85, cx, cy + 85, GRID_DIM);

    // Diagonal lines
    for (int a = 0; a < 8; a++) {
        float ang = a * PI / 4;
        fbLine(cx, cy, cx + cos(ang) * 80, cy + sin(ang) * 80, GRID_DIM);
    }

    // Sweep line with fade trail
    for (int trail = 0; trail < 20; trail++) {
        float trailAngle = sonarSweepAngle - trail * 0.05;
        int alpha = 255 - trail * 12;
        uint16_t trailColor = (alpha > 128) ? NEON_GREEN : DIM_GREEN;
        int ex = cx + cos(trailAngle) * 80;
        int ey = cy + sin(trailAngle) * 80;
        fbLine(cx, cy, ex, ey, trailColor);
    }

    // Map entities to sonar
    sonarTargetCount = 0;
    sonarNearestM = 99.9;

    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen > 5000) continue;

        float entAngle = atan2(entities[i].y - cy, entities[i].x - cx);
        float entDist = entities[i].distanceM;

        // Check if in current sweep sector (just passed)
        float angleDiff = sonarSweepAngle - entAngle;
        while (angleDiff < 0) angleDiff += 2 * PI;
        while (angleDiff > 2 * PI) angleDiff -= 2 * PI;

        if (angleDiff < 1.0) {  // Recently swept
            // Draw ping blip
            float r = min(75.0f, entDist * 8);
            int px = cx + cos(entAngle) * r;
            int py = cy + sin(entAngle) * r;

            // Pulse effect
            float pulse = 1.0 - angleDiff;
            int blipR = 3 + pulse * 4;

            uint16_t blipCol = entities[i].isDrone ? NEON_RED : NEON_CYAN;
            fbFillCircle(px, py, blipR, blipCol);

            sonarTargetCount++;
            if (entDist < sonarNearestM) sonarNearestM = entDist;
        }
    }

    // Sonar stats
    fbText(5, 22, "PING ACTIVE", NEON_GREEN, 1);

    sprintf(buf, "TARGETS:%d", sonarTargetCount);
    fbText(5, 210, buf, NEON_CYAN, 1);

    sprintf(buf, "NEAR:%.1fM", sonarNearestM < 99 ? sonarNearestM : 0);
    fbText(120, 210, buf, NEON_GREEN, 1);

    // Depth/range indicator
    sprintf(buf, "RNG:%dM", SONAR_RANGE_M);
    fbText(180, 22, buf, NEON_PURPLE, 1);

    // Ping waveform at bottom
    fbLine(0, 225, 240, 225, NEON_GREEN);
    for (int x = 0; x < 240; x++) {
        int pingPhase = (x + frame * 4) % 60;
        int h = (pingPhase < 10) ? (10 - pingPhase) : 0;
        if (h > 0) fbRect(x, 230 - h, 1, h, NEON_GREEN);
    }

    fbScanlines();
}

// ============= NEW FACE 21: QUANTUM DETECTOR =============
// Quantum-inspired superposition probability (Nielsen & Chuang 2010)
void face_Quantum() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "QUANTUM", NEON_PURPLE, 2);
    fbShimmer(2, 18);

    // Update quantum state from entity detection
    quantumIterations++;

    // Initialize qubits in superposition
    for (int q = 0; q < 8; q++) {
        if (!qubits[q].collapsed) {
            // Superposition: equal probability of |0⟩ and |1⟩
            qubits[q].alpha = 0.707 + sin(frame * 0.1 + q) * 0.1;
            qubits[q].beta = sqrt(1.0 - qubits[q].alpha * qubits[q].alpha);
            qubits[q].phase = (frame * 0.05 + q * 0.3);
        }
    }

    // Measure qubits based on entity presence
    int humanQubits = 0;
    int maxQubits = (entityCount < 8) ? entityCount : 8;
    for (int i = 0; i < maxQubits; i++) {
        if (millis() - entities[i].lastSeen > 5000) continue;
        if (!entities[i].isDrone) {
            // "Collapse" qubit to |1⟩ (human detected)
            qubits[i].collapsed = true;
            qubits[i].result = 1;
            humanQubits++;
        }
    }

    // Calculate quantum coherence (decoherence from environment)
    quantumCoherence = 1.0 - (entityCount * 0.05);
    quantumCoherence = max(0.1f, quantumCoherence);

    // Entanglement strength (correlations between qubits)
    entanglementStrength = (humanQubits > 1) ? 0.8 : 0.2;

    // Draw quantum register visualization
    int qy = 35;
    fbText(5, qy, "QUBITS |", NEON_CYAN, 1);

    for (int q = 0; q < 8; q++) {
        int qx = 70 + q * 20;

        // Bloch sphere mini representation
        int blobR = 8;
        fbCircle(qx, qy + 20, blobR, NEON_PURPLE);

        if (qubits[q].collapsed) {
            // Collapsed state
            uint16_t col = (qubits[q].result == 1) ? NEON_GREEN : NEON_RED;
            fbFillCircle(qx, qy + 20, 5, col);
            fbText(qx - 3, qy + 35, qubits[q].result ? "1" : "0", col, 1);
        } else {
            // Superposition - spinning indicator
            float px = cos(qubits[q].phase) * 5;
            float py = sin(qubits[q].phase) * 5;
            fbFillCircle(qx + px, qy + 20 + py, 3, NEON_CYAN);
            fbText(qx - 3, qy + 35, "?", NEON_PURPLE, 1);
        }
    }

    // Probability amplitudes
    fbText(5, 80, "AMPLITUDES:", NEON_PINK, 1);
    sprintf(buf, "|0>:%.2f |1>:%.2f", qubits[0].alpha, qubits[0].beta);
    fbText(5, 95, buf, NEON_CYAN, 1);

    // Wave function visualization
    fbText(5, 115, "PSI WAVE:", NEON_GREEN, 1);
    for (int x = 0; x < 220; x++) {
        float psi = sin(x * 0.1 + frame * 0.1) * cos(x * 0.05 - frame * 0.05);
        psi *= quantumCoherence;
        int y = 145 + psi * 20;
        uint16_t col = (psi > 0) ? NEON_CYAN : NEON_PURPLE;
        fbPixel(x + 10, y, col);
        fbPixel(x + 10, y + 1, col);
    }

    // Stats
    fbLine(0, 170, 240, 170, NEON_PURPLE);

    sprintf(buf, "COHERENCE:%.0f%%", quantumCoherence * 100);
    fbText(5, 175, buf, NEON_CYAN, 1);

    sprintf(buf, "ENTANGLE:%.0f%%", entanglementStrength * 100);
    fbText(5, 190, buf, NEON_PINK, 1);

    sprintf(buf, "HUMANS:|%d>", humanQubits);
    fbText(130, 175, buf, NEON_GREEN, 1);

    sprintf(buf, "ITER:%d", quantumIterations);
    fbText(130, 190, buf, NEON_PURPLE, 1);

    // Quantum uncertainty principle reminder
    fbText(5, 210, "DX*DP>=H/4PI", NEON_YELLOW, 1);
    fbText(5, 225, "HEISENBERG 1927", DIM_CYAN, 1);

    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 22: TERNARY LOGIC =============
// Three-valued logic detection (Lukasiewicz 1920, Kleene 1938)
void face_Ternary() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "TERNARY", NEON_YELLOW, 2);
    fbShimmer(2, 18);

    // Update ternary states from entities
    ternaryTrue = 0;
    ternaryFalse = 0;
    ternaryUnknown = 0;

    for (int i = 0; i < entityCount && i < MAX_ENTITIES; i++) {
        if (millis() - entities[i].lastSeen > 10000) {
            ternaryDetect[i].humanPresent = 0;  // Unknown
            ternaryDetect[i].confidence = 0;
        } else {
            float sim = signalSimilarity(&entities[i]);

            // Three-valued logic assignment
            if (sim > 0.7) {
                ternaryDetect[i].humanPresent = 1;   // True
                ternaryDetect[i].threatLevel = -1;   // Safe
                ternaryTrue++;
            } else if (sim < 0.3 || entities[i].isDrone) {
                ternaryDetect[i].humanPresent = -1;  // False (not human)
                ternaryDetect[i].threatLevel = 1;    // Threat
                ternaryFalse++;
            } else {
                ternaryDetect[i].humanPresent = 0;   // Unknown
                ternaryDetect[i].threatLevel = 0;    // Unknown
                ternaryUnknown++;
            }

            ternaryDetect[i].confidence = sim;
        }
    }

    // Ternary truth table visualization
    fbText(5, 25, "LOGIC STATE:", NEON_CYAN, 1);

    // Draw truth values
    int ty = 45;
    fbText(5, ty, "TRUE(+1)", NEON_GREEN, 1);
    sprintf(buf, "%d", ternaryTrue);
    fbText(100, ty, buf, NEON_GREEN, 2);

    ty += 25;
    fbText(5, ty, "FALSE(-1)", NEON_RED, 1);
    sprintf(buf, "%d", ternaryFalse);
    fbText(100, ty, buf, NEON_RED, 2);

    ty += 25;
    fbText(5, ty, "UNKNOWN(0)", NEON_YELLOW, 1);
    sprintf(buf, "%d", ternaryUnknown);
    fbText(100, ty, buf, NEON_YELLOW, 2);

    // Ternary bar visualization
    int barY = 125;
    int totalTern = ternaryTrue + ternaryFalse + ternaryUnknown;
    if (totalTern > 0) {
        int trueW = (ternaryTrue * 220) / totalTern;
        int falseW = (ternaryFalse * 220) / totalTern;
        int unkW = 220 - trueW - falseW;

        fbRect(10, barY, trueW, 15, NEON_GREEN);
        fbRect(10 + trueW, barY, falseW, 15, NEON_RED);
        fbRect(10 + trueW + falseW, barY, unkW, 15, NEON_YELLOW);
    }

    // Kleene logic operators
    fbText(5, 150, "KLEENE OPS:", NEON_PURPLE, 1);
    fbText(5, 165, "AND: MIN(A,B)", NEON_CYAN, 1);
    fbText(5, 180, "OR: MAX(A,B)", NEON_CYAN, 1);
    fbText(5, 195, "NOT: -A", NEON_CYAN, 1);

    // Entity confidence list
    fbLine(120, 145, 120, 235, NEON_PURPLE);
    fbText(125, 150, "ENTITIES:", NEON_PINK, 1);

    int ey = 165;
    int maxTern = (entityCount < 4) ? entityCount : 4;
    for (int i = 0; i < maxTern; i++) {
        TernaryState* ts = &ternaryDetect[i];
        const char* state = (ts->humanPresent == 1) ? "+1" :
                           (ts->humanPresent == -1) ? "-1" : " 0";
        uint16_t col = (ts->humanPresent == 1) ? NEON_GREEN :
                       (ts->humanPresent == -1) ? NEON_RED : NEON_YELLOW;

        sprintf(buf, "%s:%.0f%%", state, ts->confidence * 100);
        fbText(125, ey, buf, col, 1);
        ey += 15;
    }

    // Reference
    fbText(5, 225, "LUKASIEWICZ 1920", DIM_CYAN, 1);

    fbGlitch(1);
    fbScanlines();
}

// ============= NEW FACE 23: NEURAL NET CLASSIFIER =============
// TinyML entity classification (Rosenblatt 1958, Warden 2019)
void face_NeuralNet() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "NEURAL NET", NEON_PINK, 2);
    fbShimmer(2, 18);

    // Initialize NN weights if needed
    if (!nnTrained) {
        for (int i = 0; i < NN_INPUTS; i++) {
            for (int h = 0; h < NN_HIDDEN; h++) {
                nnWeights1[i][h] = (random(100) - 50) / 100.0;
            }
        }
        for (int h = 0; h < NN_HIDDEN; h++) {
            for (int o = 0; o < NN_OUTPUTS; o++) {
                nnWeights2[h][o] = (random(100) - 50) / 100.0;
            }
            nnBias1[h] = (random(100) - 50) / 100.0;
        }
        for (int o = 0; o < NN_OUTPUTS; o++) {
            nnBias2[o] = (random(100) - 50) / 100.0;
        }
        nnTrained = true;
    }

    // Run inference on nearest entity
    float inputs[NN_INPUTS] = {0};
    if (entityCount > 0) {
        Entity* e = &entities[0];
        inputs[0] = (e->rssi + 100) / 100.0;        // Normalize RSSI
        inputs[1] = e->distanceM / 20.0;            // Normalize distance
        inputs[2] = e->isDrone ? 1.0 : 0.0;
        inputs[3] = e->predHR / 100.0;
        inputs[4] = e->bioSimilarity;
        inputs[5] = (e->mac[0] + e->mac[5]) / 512.0;
    }

    // Forward pass - hidden layer
    float hidden[NN_HIDDEN];
    for (int h = 0; h < NN_HIDDEN; h++) {
        hidden[h] = nnBias1[h];
        for (int i = 0; i < NN_INPUTS; i++) {
            hidden[h] += inputs[i] * nnWeights1[i][h];
        }
        // ReLU activation
        hidden[h] = max(0.0f, hidden[h]);
    }

    // Output layer
    float maxOut = -999;
    int maxIdx = 0;
    for (int o = 0; o < NN_OUTPUTS; o++) {
        nnOutput[o] = nnBias2[o];
        for (int h = 0; h < NN_HIDDEN; h++) {
            nnOutput[o] += hidden[h] * nnWeights2[h][o];
        }
        // Softmax prep
        if (nnOutput[o] > maxOut) {
            maxOut = nnOutput[o];
            maxIdx = o;
        }
    }

    // Draw network visualization
    int netY = 70;

    // Input layer
    fbText(5, 25, "INPUTS", NEON_CYAN, 1);
    for (int i = 0; i < NN_INPUTS; i++) {
        int ix = 20, iy = 45 + i * 18;
        int brightness = inputs[i] * 255;
        uint16_t col = (brightness > 128) ? NEON_GREEN : DIM_GREEN;
        fbFillCircle(ix, iy, 6, col);

        // Connection lines to hidden
        for (int h = 0; h < NN_HIDDEN; h++) {
            int hx = 100, hy = 60 + h * 30;
            if (abs(nnWeights1[i][h]) > 0.3) {
                fbLine(ix + 6, iy, hx - 6, hy, GRID_DIM);
            }
        }
    }

    // Hidden layer
    fbText(80, 25, "HIDDEN", NEON_PURPLE, 1);
    for (int h = 0; h < NN_HIDDEN; h++) {
        int hx = 100, hy = 60 + h * 30;
        uint16_t col = (hidden[h] > 0.5) ? NEON_PURPLE : DIM_RED;
        fbFillCircle(hx, hy, 8, col);

        // Connection to output
        for (int o = 0; o < NN_OUTPUTS; o++) {
            int ox = 180, oy = 70 + o * 35;
            if (abs(nnWeights2[h][o]) > 0.3) {
                fbLine(hx + 8, hy, ox - 8, oy, GRID_DIM);
            }
        }
    }

    // Output layer
    fbText(160, 25, "OUTPUT", NEON_PINK, 1);
    const char* classes[] = {"HUMAN", "DRONE", "UNKNWN"};
    for (int o = 0; o < NN_OUTPUTS; o++) {
        int ox = 180, oy = 70 + o * 35;
        uint16_t col = (o == maxIdx) ? NEON_GREEN : DIM_CYAN;
        fbFillCircle(ox, oy, 10, col);
        fbText(ox + 15, oy - 4, classes[o], col, 1);
    }

    // Classification result
    fbLine(0, 175, 240, 175, NEON_PINK);

    fbText(5, 180, "CLASSIFY:", NEON_CYAN, 1);
    fbText(80, 180, classes[maxIdx], NEON_GREEN, 2);

    sprintf(buf, "CONF:%.0f%%", (maxOut + 1) * 50);
    fbText(5, 200, buf, NEON_PURPLE, 1);

    sprintf(buf, "ENTITIES:%d", entityCount);
    fbText(120, 200, buf, NEON_CYAN, 1);

    // Reference
    fbText(5, 225, "ROSENBLATT 1958", DIM_CYAN, 1);

    fbGlitch(1);
    fbScanlines();
}

// ============= NEW FACE 24: BRAINWAVE EEG =============
// EEG band visualization (Berger 1929)
void face_Brainwave() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "BRAINWAVE", NEON_CYAN, 2);
    fbShimmer(2, 18);

    // Simulate brainwave states from HRV/stress
    float relaxation = 1.0 - (stress / 100.0);
    alphaWave = 0.3 + relaxation * 0.5 + sin(frame * 0.05) * 0.1;
    betaWave = 0.2 + (stress / 100.0) * 0.4 + cos(frame * 0.07) * 0.1;
    thetaWave = 0.1 + sin(frame * 0.03) * 0.15;
    deltaWave = (sleepScore > 80) ? 0.4 : 0.1;
    gammaWave = 0.1 + sin(frame * 0.1) * 0.1;

    // Wave displays
    struct WaveBand { const char* name; float* val; float freq; uint16_t col; };
    WaveBand bands[] = {
        {"DELTA 0.5-4Hz", &deltaWave, 2, NEON_PURPLE},
        {"THETA 4-8Hz", &thetaWave, 6, NEON_PINK},
        {"ALPHA 8-13Hz", &alphaWave, 10, NEON_GREEN},
        {"BETA 14-30Hz", &betaWave, 22, NEON_CYAN},
        {"GAMMA 30+Hz", &gammaWave, 40, NEON_YELLOW}
    };

    int waveY = 30;
    for (int b = 0; b < 5; b++) {
        fbText(5, waveY, bands[b].name, bands[b].col, 1);

        // Draw wave
        for (int x = 0; x < 100; x++) {
            float phase = x * 0.1 * (bands[b].freq / 10.0) + frame * 0.1;
            int y = waveY + 12 + sin(phase) * (*bands[b].val) * 10;
            fbPixel(130 + x, y, bands[b].col);
        }

        // Power bar
        int barW = (*bands[b].val) * 80;
        fbRect(130, waveY + 22, barW, 4, bands[b].col);

        waveY += 38;
    }

    // Brain state interpretation
    fbLine(0, 210, 240, 210, NEON_CYAN);

    const char* state = "ALERT";
    uint16_t stateCol = NEON_CYAN;
    if (alphaWave > 0.6) { state = "RELAXED"; stateCol = NEON_GREEN; }
    else if (betaWave > 0.5) { state = "FOCUSED"; stateCol = NEON_YELLOW; }
    else if (thetaWave > 0.4) { state = "DROWSY"; stateCol = NEON_PURPLE; }
    else if (deltaWave > 0.5) { state = "DEEP SLEEP"; stateCol = NEON_PINK; }

    fbText(5, 215, "STATE:", NEON_CYAN, 1);
    fbText(60, 215, state, stateCol, 2);

    fbText(5, 230, "BERGER 1929", DIM_CYAN, 1);

    fbGlitch(1);
    fbScanlines();
}

// ============= NEW FACE 25: CHAKRA ENERGY =============
// Biofield visualization (fringe but spectacular)
void face_Chakra() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "CHAKRA", NEON_PURPLE, 2);
    fbShimmer(2, 18);

    // Update chakra energies based on biometrics
    chakraEnergy[0] = 0.5 + (100 - stress) / 200.0;  // Root - stability
    chakraEnergy[1] = 0.5 + sin(frame * 0.02) * 0.2;  // Sacral - creativity
    chakraEnergy[2] = 0.5 + (heartRate - 60) / 80.0;  // Solar - power
    chakraEnergy[3] = 0.6 + hrv / 150.0;              // Heart - love
    chakraEnergy[4] = 0.5 + breathRate / 40.0;        // Throat - expression
    chakraEnergy[5] = 0.5 + cos(frame * 0.03) * 0.2;  // Third eye - intuition
    chakraEnergy[6] = 0.5 + sleepScore / 200.0;       // Crown - consciousness

    // Draw body silhouette with chakra points
    int bx = 80, by = 130;

    // Simple body outline
    fbCircle(bx, by - 70, 15, DIM_CYAN);  // Head
    fbLine(bx, by - 55, bx, by + 20, DIM_CYAN);  // Spine
    fbLine(bx - 25, by - 40, bx + 25, by - 40, DIM_CYAN);  // Shoulders
    fbLine(bx, by + 20, bx - 15, by + 60, DIM_CYAN);  // Left leg
    fbLine(bx, by + 20, bx + 15, by + 60, DIM_CYAN);  // Right leg

    // Chakra points along spine
    int chakraY[] = {190, 170, 150, 130, 110, 75, 55};

    for (int c = 0; c < 7; c++) {
        float energy = chakraEnergy[c];
        int r = 8 + energy * 8 + sin(frame * 0.1 + c) * 3;

        // Glow effect
        for (int gr = r + 5; gr > r; gr--) {
            fbCircle(bx, chakraY[c], gr, chakraColors[c]);
        }
        fbFillCircle(bx, chakraY[c], r, chakraColors[c]);

        // Energy rays
        if (energy > 0.7) {
            for (int ray = 0; ray < 4; ray++) {
                float angle = ray * PI / 2 + frame * 0.05;
                int rx = bx + cos(angle) * (r + 10);
                int ry = chakraY[c] + sin(angle) * (r + 10);
                fbLine(bx, chakraY[c], rx, ry, chakraColors[c]);
            }
        }
    }

    // Energy levels on right
    int ey = 30;
    for (int c = 6; c >= 0; c--) {
        fbText(130, ey, chakraNames[c], chakraColors[c], 1);
        int barW = chakraEnergy[c] * 50;
        fbRect(185, ey, barW, 8, chakraColors[c]);
        ey += 18;
    }

    // Total energy
    float totalEnergy = 0;
    for (int c = 0; c < 7; c++) totalEnergy += chakraEnergy[c];
    totalEnergy /= 7;

    fbLine(0, 210, 240, 210, NEON_PURPLE);
    sprintf(buf, "BALANCE:%.0f%%", totalEnergy * 100);
    fbText(5, 215, buf, NEON_PINK, 2);

    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 26: AURA SCANNER =============
// Kirlian-style biofield visualization
void face_Aura() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "AURA SCAN", NEON_PINK, 2);
    fbShimmer(2, 18);

    // Update aura based on vitals
    auraStrength = 0.5 + hrv / 100.0 + (100 - stress) / 200.0;
    auraRadius = 30 + auraStrength * 20 + sin(frame * 0.05) * 5;

    // Aura color based on emotional state
    if (stress < 30) auraColor = NEON_GREEN;       // Calm
    else if (stress < 50) auraColor = NEON_CYAN;   // Balanced
    else if (stress < 70) auraColor = NEON_YELLOW; // Active
    else auraColor = NEON_RED;                      // Stressed

    int cx = 120, cy = 110;

    // Draw aura layers (outer to inner)
    for (int layer = 5; layer >= 0; layer--) {
        int r = auraRadius + layer * 12;
        uint16_t layerCol = auraColor;

        // Vary color by layer
        if (layer == 4) layerCol = NEON_PURPLE;
        if (layer == 3) layerCol = NEON_CYAN;
        if (layer == 2) layerCol = NEON_GREEN;

        // Animated irregular edge
        for (int a = 0; a < 360; a += 3) {
            float angle = a * PI / 180;
            float wobble = sin(angle * 5 + frame * 0.1 + layer) * (5 + layer * 2);
            int px = cx + cos(angle) * (r + wobble);
            int py = cy + sin(angle) * (r + wobble);
            fbPixel(px, py, layerCol);
        }
    }

    // Body silhouette in center
    fbFillCircle(cx, cy - 20, 12, WHITE);  // Head
    fbRect(cx - 15, cy - 5, 30, 35, WHITE);  // Body

    // Aura color meaning
    fbLine(0, 185, 240, 185, NEON_PINK);

    const char* meaning = "BALANCED";
    if (auraColor == NEON_GREEN) meaning = "PEACEFUL";
    else if (auraColor == NEON_YELLOW) meaning = "ENERGETIC";
    else if (auraColor == NEON_RED) meaning = "INTENSE";

    fbText(5, 190, "AURA:", NEON_CYAN, 1);
    fbText(50, 190, meaning, auraColor, 2);

    sprintf(buf, "STRENGTH:%.0f%%", auraStrength * 100);
    fbText(5, 210, buf, NEON_PURPLE, 1);

    sprintf(buf, "RADIUS:%dCM", (int)(auraRadius * 2));
    fbText(120, 210, buf, NEON_PINK, 1);

    fbText(5, 225, "KIRLIAN STYLE", DIM_CYAN, 1);

    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 27: CIRCADIAN RHYTHM =============
// Sleep/wake cycle (Czeisler 1999)
void face_Circadian() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "CIRCADIAN", NEON_CYAN, 2);
    fbShimmer(2, 18);

    // Calculate circadian phase from RTC
    circadianPhase = rtcHour + rtcMin / 60.0;

    // Melatonin: high at night (21:00-06:00)
    if (circadianPhase > 21 || circadianPhase < 6) {
        melatoninLevel = 0.8 + sin((circadianPhase - 3) * PI / 12) * 0.2;
    } else {
        melatoninLevel = 0.1 + sin(circadianPhase * PI / 24) * 0.1;
    }

    // Cortisol: peak in morning (06:00-09:00)
    if (circadianPhase > 6 && circadianPhase < 12) {
        cortisolLevel = 0.8 - (circadianPhase - 6) * 0.1;
    } else {
        cortisolLevel = 0.3 + sin(circadianPhase * PI / 24) * 0.2;
    }

    // 24-hour clock visualization
    int cx = 120, cy = 100;
    int cr = 60;

    fbCircle(cx, cy, cr, NEON_CYAN);
    fbCircle(cx, cy, cr + 1, NEON_CYAN);

    // Hour markers
    for (int h = 0; h < 24; h++) {
        float angle = (h - 6) * PI / 12;  // 6:00 at top
        int x1 = cx + cos(angle) * (cr - 5);
        int y1 = cy + sin(angle) * (cr - 5);
        int x2 = cx + cos(angle) * cr;
        int y2 = cy + sin(angle) * cr;
        fbLine(x1, y1, x2, y2, (h % 6 == 0) ? NEON_GREEN : GRID_DIM);
    }

    // Day/Night shading
    for (int h = 21; h < 24; h++) {
        float a1 = (h - 6) * PI / 12;
        float a2 = (h + 1 - 6) * PI / 12;
        for (float a = a1; a < a2; a += 0.05) {
            int px = cx + cos(a) * (cr - 15);
            int py = cy + sin(a) * (cr - 15);
            fbPixel(px, py, NEON_PURPLE);
        }
    }
    for (int h = 0; h < 6; h++) {
        float a1 = (h - 6) * PI / 12;
        float a2 = (h + 1 - 6) * PI / 12;
        for (float a = a1; a < a2; a += 0.05) {
            int px = cx + cos(a) * (cr - 15);
            int py = cy + sin(a) * (cr - 15);
            fbPixel(px, py, NEON_PURPLE);
        }
    }

    // Current time hand
    float timeAngle = (circadianPhase - 6) * PI / 12;
    int hx = cx + cos(timeAngle) * (cr - 20);
    int hy = cy + sin(timeAngle) * (cr - 20);
    fbLine(cx, cy, hx, hy, NEON_GREEN);
    fbFillCircle(hx, hy, 4, NEON_GREEN);

    // Hormone levels
    fbLine(0, 175, 240, 175, NEON_PURPLE);

    // Melatonin bar
    fbText(5, 180, "MELATONIN", NEON_PURPLE, 1);
    int melBar = melatoninLevel * 100;
    fbRect(90, 180, melBar, 10, NEON_PURPLE);

    // Cortisol bar
    fbText(5, 195, "CORTISOL", NEON_YELLOW, 1);
    int cortBar = cortisolLevel * 100;
    fbRect(90, 195, cortBar, 10, NEON_YELLOW);

    // Sleep recommendation
    const char* advice = "STAY AWAKE";
    if (melatoninLevel > 0.6) advice = "SLEEP TIME";
    else if (cortisolLevel > 0.6) advice = "PEAK ALERT";

    fbText(5, 215, "STATUS:", NEON_CYAN, 1);
    fbText(70, 215, advice, NEON_GREEN, 1);

    fbText(5, 230, "CZEISLER 1999", DIM_CYAN, 1);

    fbGlitch(1);
    fbScanlines();
}

// ============= NEW FACE 28: ELECTRODERMAL =============
// Skin conductance (Boucsein 2012)
void face_Electrodermal() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "EDA SCAN", NEON_YELLOW, 2);
    fbShimmer(2, 18);

    // Simulate EDA from stress/arousal
    edaTonic = 4.0 + stress / 25.0;  // Baseline 4-8 µS
    edaPhasic = sin(frame * 0.1) * (stress / 50.0);  // Event responses
    edaLevel = edaTonic + edaPhasic;

    // EDA waveform
    fbText(5, 25, "SKIN CONDUCT:", NEON_CYAN, 1);

    static float edaHistory[220] = {0};
    for (int i = 0; i < 219; i++) edaHistory[i] = edaHistory[i + 1];
    edaHistory[219] = edaLevel;

    // Draw waveform
    for (int x = 0; x < 220; x++) {
        int y = 90 - (edaHistory[x] - 3) * 8;
        y = constrain(y, 45, 90);
        fbPixel(10 + x, y, NEON_GREEN);
        fbPixel(10 + x, y + 1, NEON_GREEN);
    }

    // Grid lines
    for (int g = 0; g < 5; g++) {
        int gy = 45 + g * 12;
        fbLine(10, gy, 230, gy, GRID_DIM);
    }

    // Current value
    sprintf(buf, "%.1f uS", edaLevel);
    fbTextCenter(105, buf, NEON_GREEN, 3);

    // Components
    fbLine(0, 145, 240, 145, NEON_YELLOW);

    sprintf(buf, "TONIC:%.1f", edaTonic);
    fbText(5, 150, buf, NEON_CYAN, 1);

    sprintf(buf, "PHASIC:%.2f", edaPhasic);
    fbText(120, 150, buf, NEON_PINK, 1);

    // Arousal interpretation
    fbText(5, 175, "AROUSAL:", NEON_PURPLE, 1);

    const char* arousal = "LOW";
    uint16_t arousalCol = NEON_GREEN;
    if (edaLevel > 6) { arousal = "HIGH"; arousalCol = NEON_RED; }
    else if (edaLevel > 5) { arousal = "MEDIUM"; arousalCol = NEON_YELLOW; }

    fbText(80, 175, arousal, arousalCol, 2);

    // Bar graph
    int barW = (edaLevel - 2) * 25;
    barW = constrain(barW, 0, 200);
    fbRect(20, 200, barW, 15, arousalCol);
    fbRect(20, 200, 200, 15, GRID_DIM);

    fbText(5, 225, "BOUCSEIN 2012", DIM_CYAN, 1);

    fbGlitch(1);
    fbScanlines();
}

// ============= NEW FACE 29: DARK MATTER =============
// Fun cosmic detection (obviously simulated)
void face_DarkMatter() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "DARK MATTER", NEON_PURPLE, 2);
    fbShimmer(2, 18);

    // Simulate dark matter events from random fluctuations
    if (random(100) < 3) darkMatterEvents++;
    darkMatterDensity = 0.23 + sin(frame * 0.01) * 0.02;

    // Cosmic background
    for (int i = 0; i < 50; i++) {
        int x = random(240);
        int y = random(240);
        fbPixel(x, y, (random(3) == 0) ? NEON_PURPLE : GRID_DIM);
    }

    // Dark matter halo visualization
    int cx = 120, cy = 120;

    // Invisible mass visualization (purple glow)
    for (int r = 80; r > 20; r -= 5) {
        uint16_t col = (r > 60) ? DIM_RED : NEON_PURPLE;
        for (int a = 0; a < 360; a += 10) {
            float angle = a * PI / 180 + frame * 0.02;
            float wobble = sin(angle * 3 + r * 0.1) * 10;
            int px = cx + cos(angle) * (r + wobble);
            int py = cy + sin(angle) * (r + wobble);
            fbPixel(px, py, col);
        }
    }

    // Detection events (flashes)
    if (frame % 30 < 5 && darkMatterEvents > 0) {
        int ex = random(80, 160);
        int ey = random(80, 160);
        fbFillCircle(ex, ey, 5, WHITE);
        fbCircle(ex, ey, 10, NEON_PURPLE);
    }

    // Stats
    sprintf(buf, "DENSITY:%.0f%%", darkMatterDensity * 100);
    fbText(5, 25, buf, NEON_PURPLE, 1);

    sprintf(buf, "EVENTS:%d", darkMatterEvents);
    fbText(5, 40, buf, NEON_PINK, 1);

    fbLine(0, 200, 240, 200, NEON_PURPLE);

    fbText(5, 205, "WIMP SEARCH", NEON_CYAN, 1);
    fbText(5, 220, "SIMULATED", NEON_YELLOW, 1);

    fbText(140, 205, "23% OF", NEON_PURPLE, 1);
    fbText(140, 220, "UNIVERSE", NEON_PURPLE, 1);

    fbGlitch(3);
    fbScanlines();
}

// ============= NEW FACE 30: GRAVITATIONAL WAVE =============
// LIGO-inspired visualization
void face_GravWave() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "GRAV WAVE", NEON_CYAN, 2);
    fbShimmer(2, 18);

    // Simulate gravitational wave from entity motion
    gwAmplitude = 0;
    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen < 5000) {
            // Movement creates "ripples in spacetime"
            gwAmplitude += 1.0 / (entities[i].distanceM + 1);
        }
    }
    gwAmplitude = (gwAmplitude * 0.3f < 1.0f) ? gwAmplitude * 0.3f : 1.0f;
    gwFrequency = 50 + gwAmplitude * 200;

    // Spacetime grid distortion
    int cx = 120, cy = 110;

    for (int gx = -5; gx <= 5; gx++) {
        for (int gy = -5; gy <= 5; gy++) {
            float dist = sqrt(gx * gx + gy * gy);
            float wave = sin(dist * 0.5 - frame * 0.1) * gwAmplitude * 10;

            int px = cx + gx * 20 + wave;
            int py = cy + gy * 15 + wave * 0.5;

            fbPixel(px, py, NEON_CYAN);
            fbPixel(px + 1, py, NEON_CYAN);

            // Grid lines
            if (gx < 5) {
                float wave2 = sin((dist + 1) * 0.5 - frame * 0.1) * gwAmplitude * 10;
                int px2 = cx + (gx + 1) * 20 + wave2;
                fbLine(px, py, px2, py, GRID_DIM);
            }
            if (gy < 5) {
                float wave2 = sin(sqrt(gx * gx + (gy + 1) * (gy + 1)) * 0.5 - frame * 0.1) * gwAmplitude * 10;
                int py2 = cy + (gy + 1) * 15 + wave2 * 0.5;
                fbLine(px, py, px, py2, GRID_DIM);
            }
        }
    }

    // Waveform
    fbText(5, 180, "STRAIN h:", NEON_GREEN, 1);
    for (int x = 0; x < 220; x++) {
        float h = sin(x * gwFrequency / 1000.0 + frame * 0.1) * gwAmplitude;
        int y = 205 + h * 20;
        fbPixel(10 + x, y, NEON_GREEN);
    }

    sprintf(buf, "AMP:%.2f", gwAmplitude);
    fbText(5, 25, buf, NEON_CYAN, 1);

    sprintf(buf, "FREQ:%.0fHz", gwFrequency);
    fbText(140, 25, buf, NEON_PURPLE, 1);

    fbText(5, 225, "LIGO STYLE", DIM_CYAN, 1);

    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 31: NEUTRINO DETECTOR =============
// Super-Kamiokande inspired
void face_Neutrino() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "NEUTRINO", NEON_CYAN, 2);
    fbShimmer(2, 18);

    // Simulate neutrino events
    if (random(100) < 5) {
        neutrinoCount++;
        neutrinoEnergy = random(10, 1000) / 10.0;  // MeV
    }

    // Water tank visualization (Cherenkov detector style)
    int cx = 120, cy = 110;

    // Tank outline
    fbCircle(cx, cy, 70, NEON_CYAN);
    fbCircle(cx, cy, 65, DIM_CYAN);

    // PMT array (photomultipliers)
    for (int a = 0; a < 360; a += 15) {
        float angle = a * PI / 180;
        int px = cx + cos(angle) * 67;
        int py = cy + sin(angle) * 67;
        fbFillCircle(px, py, 3, (a == (frame % 360)) ? NEON_GREEN : DIM_GREEN);
    }

    // Cherenkov cone (when event detected)
    if (frame % 60 < 20) {
        int evtX = cx + random(-30, 30);
        int evtY = cy + random(-30, 30);

        // Cone of light
        for (int r = 5; r < 40; r += 3) {
            float cone = r * 0.4;
            for (int a = -30; a <= 30; a += 5) {
                float angle = a * PI / 180 + random(10) / 100.0;
                int px = evtX + cos(angle) * r;
                int py = evtY + sin(angle) * r + r / 2;
                fbPixel(px, py, NEON_CYAN);
            }
        }

        fbFillCircle(evtX, evtY, 3, WHITE);
    }

    // Stats
    sprintf(buf, "EVENTS:%d", neutrinoCount);
    fbText(5, 25, buf, NEON_GREEN, 1);

    sprintf(buf, "E:%.1f MeV", neutrinoEnergy);
    fbText(140, 25, buf, NEON_PURPLE, 1);

    fbLine(0, 195, 240, 195, NEON_CYAN);

    fbText(5, 200, "FLUX:", NEON_CYAN, 1);
    sprintf(buf, "%.0f /cm2/s", 65000.0 + sin(frame * 0.02) * 1000);
    fbText(50, 200, buf, NEON_GREEN, 1);

    fbText(5, 215, "CHERENKOV", NEON_PURPLE, 1);
    fbText(5, 228, "SUPER-K STYLE", DIM_CYAN, 1);

    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 32: MORPHIC FIELD =============
// Sheldrake fringe theory visualization
void face_Morphic() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "MORPHIC", NEON_PINK, 2);
    fbShimmer(2, 18);

    // Update morphic resonance from entity patterns
    morphicField = 0.5;
    collectiveMemoryHits = 0;

    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen < 5000) {
            // Similar signatures = morphic resonance
            if (entities[i].bioSimilarity > 0.6) {
                morphicField += 0.1;
                collectiveMemoryHits++;
            }
        }
    }
    morphicField = min(1.0f, morphicField);

    // Field visualization - interconnected web
    int cx = 120, cy = 110;

    // Central field
    for (int r = 20; r <= 70; r += 10) {
        float wobble = sin(frame * 0.05 + r * 0.1) * morphicField * 15;
        for (int a = 0; a < 360; a += 5) {
            float angle = a * PI / 180;
            int px = cx + cos(angle) * (r + wobble);
            int py = cy + sin(angle) * (r + wobble);
            fbPixel(px, py, NEON_PINK);
        }
    }

    // Entity connections (morphic links)
    int maxMorphic = (entityCount < 6) ? entityCount : 6;
    for (int i = 0; i < maxMorphic; i++) {
        if (millis() - entities[i].lastSeen > 5000) continue;

        float angle = i * PI / 3;
        int ex = cx + cos(angle) * 50;
        int ey = cy + sin(angle) * 50;

        fbFillCircle(ex, ey, 5, NEON_CYAN);

        // Resonance link to center
        if (entities[i].bioSimilarity > 0.5) {
            for (int p = 0; p < 10; p++) {
                float t = p / 10.0 + sin(frame * 0.1) * 0.1;
                int lx = cx + (ex - cx) * t;
                int ly = cy + (ey - cy) * t;
                fbPixel(lx, ly, NEON_PINK);
            }
        }
    }

    // Stats
    sprintf(buf, "FIELD:%.0f%%", morphicField * 100);
    fbText(5, 25, buf, NEON_PINK, 1);

    sprintf(buf, "RESONANCE:%d", collectiveMemoryHits);
    fbText(130, 25, buf, NEON_CYAN, 1);

    fbLine(0, 195, 240, 195, NEON_PURPLE);

    fbText(5, 200, "COLLECTIVE", NEON_CYAN, 1);
    fbText(5, 215, "MEMORY FIELD", NEON_PINK, 1);

    fbText(130, 200, "SHELDRAKE", DIM_CYAN, 1);
    fbText(130, 215, "HYPOTHESIS", DIM_CYAN, 1);

    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 33: TIMELINE CONVERGENCE =============
// Probability visualization
void face_Timeline() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "TIMELINE", NEON_GREEN, 2);
    fbShimmer(2, 18);

    // Calculate timeline convergence from entity stability
    timelineConvergence = 0.7;
    parallelTimelines = 1;

    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen < 3000) {
            timelineConvergence += 0.05;
            if (entities[i].bioSimilarity > 0.8) parallelTimelines++;
        }
    }
    timelineConvergence = min(1.0f, timelineConvergence);
    fateIndex = 0.5 + timelineConvergence * 0.4 + sin(frame * 0.02) * 0.1;

    // Timeline visualization
    int cx = 120;

    // Past -> Present -> Future
    fbLine(20, 80, 220, 80, GRID_DIM);
    fbText(15, 85, "PAST", DIM_CYAN, 1);
    fbText(100, 85, "NOW", NEON_GREEN, 1);
    fbText(185, 85, "FUTURE", DIM_CYAN, 1);

    // Branching timelines
    for (int t = 0; t < parallelTimelines && t < 5; t++) {
        int startX = 120;
        int y = 100 + t * 25;

        float divergence = (1.0 - timelineConvergence) * 50;

        // Draw timeline branch
        for (int x = 0; x < 100; x++) {
            float branch = sin(x * 0.1 + t) * divergence * (x / 100.0);
            int py = y + branch;
            uint16_t col = (t == 0) ? NEON_GREEN : NEON_CYAN;
            fbPixel(startX + x, py, col);
        }

        // Branch point
        fbFillCircle(startX, y, 3, NEON_PINK);
    }

    // Convergence indicator
    fbLine(0, 180, 240, 180, NEON_PURPLE);

    fbText(5, 185, "CONVERGENCE:", NEON_CYAN, 1);
    int convBar = timelineConvergence * 120;
    fbRect(110, 185, convBar, 12, NEON_GREEN);
    fbRect(110, 185, 120, 12, GRID_DIM);

    sprintf(buf, "FATE INDEX:%.0f%%", fateIndex * 100);
    fbText(5, 205, buf, NEON_PINK, 1);

    sprintf(buf, "BRANCHES:%d", parallelTimelines);
    fbText(140, 205, buf, NEON_PURPLE, 1);

    fbText(5, 225, "PROBABILITY", DIM_CYAN, 1);

    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 34: BIOELECTRIC FIELD =============
// Body electric field visualization
void face_Bioelectric() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "BIOELECTRIC", NEON_CYAN, 2);
    fbShimmer(2, 18);

    int cx = 120, cy = 110;

    // Body outline
    fbCircle(cx, cy - 50, 15, NEON_PINK);  // Head
    fbRect(cx - 20, cy - 30, 40, 50, DIM_CYAN);  // Torso

    // Electric field lines
    for (int field = 0; field < 12; field++) {
        float baseAngle = field * PI / 6;
        float amplitude = 40 + sin(frame * 0.1 + field) * 10;

        // Field line
        for (float t = 0; t < 1; t += 0.02) {
            float angle = baseAngle + sin(t * PI * 2 + frame * 0.05) * 0.3;
            float r = 30 + t * amplitude;
            int px = cx + cos(angle) * r;
            int py = cy + sin(angle) * r;

            // Color gradient based on field strength
            uint16_t col = (t < 0.3) ? NEON_CYAN :
                          (t < 0.6) ? NEON_GREEN : NEON_YELLOW;
            fbPixel(px, py, col);
        }
    }

    // Charge points
    fbFillCircle(cx, cy - 50, 5, NEON_PINK);   // Head +
    fbFillCircle(cx - 25, cy - 10, 4, NEON_CYAN);  // Heart
    fbFillCircle(cx + 25, cy - 10, 4, NEON_GREEN); // Solar plexus

    // Field strength meter
    float fieldStrength = 50 + hrv / 2 + sin(frame * 0.05) * 10;

    fbLine(0, 185, 240, 185, NEON_CYAN);

    sprintf(buf, "FIELD:%.0f mV", fieldStrength);
    fbText(5, 190, buf, NEON_GREEN, 2);

    sprintf(buf, "POLARITY:+/-");
    fbText(5, 215, buf, NEON_CYAN, 1);

    sprintf(buf, "FREQ:%.1f Hz", 7.83 + sin(frame * 0.02) * 0.5);
    fbText(120, 215, buf, NEON_PURPLE, 1);

    fbText(5, 230, "SCHUMANN RES", DIM_CYAN, 1);

    fbGlitch(1);
    fbScanlines();
}

// ============= NEW FACE 35: DNA RESONANCE =============
// Double helix visualization
void face_DNA() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "DNA SCAN", NEON_GREEN, 2);
    fbShimmer(2, 18);

    // Rotating double helix
    int cx = 120;

    for (int y = 30; y < 180; y += 3) {
        float phase = y * 0.1 + frame * 0.1;

        // Strand 1
        int x1 = cx + sin(phase) * 40;
        fbFillCircle(x1, y, 4, NEON_CYAN);

        // Strand 2 (180° out of phase)
        int x2 = cx + sin(phase + PI) * 40;
        fbFillCircle(x2, y, 4, NEON_PINK);

        // Base pairs (rungs)
        if (y % 9 < 3) {
            fbLine(x1, y, x2, y, NEON_GREEN);
            // Base pair letters
            const char* bases[] = {"A-T", "G-C", "T-A", "C-G"};
            int baseIdx = (y / 9) % 4;
            fbText(cx - 10, y - 3, bases[baseIdx], NEON_YELLOW, 1);
        }
    }

    // Stats
    fbLine(0, 190, 240, 190, NEON_GREEN);

    fbText(5, 195, "TELOMERE:", NEON_CYAN, 1);
    int telomere = 8000 - (frame % 100);
    sprintf(buf, "%d bp", telomere);
    fbText(90, 195, buf, NEON_GREEN, 1);

    fbText(5, 210, "METHYLATION:", NEON_PURPLE, 1);
    sprintf(buf, "%.0f%%", 75 + sin(frame * 0.01) * 5);
    fbText(110, 210, buf, NEON_PINK, 1);

    fbText(5, 225, "WATSON-CRICK", DIM_CYAN, 1);

    fbGlitch(1);
    fbScanlines();
}

// ============= NEW FACE 36: HOLOGRAPHIC UNIVERSE =============
// Holographic principle visualization
void face_Holographic() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "HOLOGRAPH", NEON_PURPLE, 2);
    fbShimmer(2, 18);

    // Holographic boundary
    int cx = 120, cy = 110;

    // 2D surface encoding 3D reality
    for (int r = 20; r <= 70; r += 5) {
        for (int a = 0; a < 360; a += 3) {
            float angle = a * PI / 180;

            // Information bits on surface
            float info = sin(angle * 10 + r * 0.3 + frame * 0.05);
            uint16_t col = (info > 0) ? NEON_CYAN : NEON_PURPLE;

            int px = cx + cos(angle) * r;
            int py = cy + sin(angle) * r;
            fbPixel(px, py, col);
        }
    }

    // Inner "projection"
    for (int i = 0; i < 20; i++) {
        float angle = random(360) * PI / 180;
        float r = random(15, 50);
        int px = cx + cos(angle) * r;
        int py = cy + sin(angle) * r;

        if (frame % 20 < 10) {
            fbPixel(px, py, NEON_GREEN);
            fbPixel(px + 1, py, NEON_GREEN);
        }
    }

    // Information density
    float infoDensity = entityCount * 10 + 50;

    fbLine(0, 185, 240, 185, NEON_PURPLE);

    sprintf(buf, "INFO:%.0f bits", infoDensity);
    fbText(5, 190, buf, NEON_CYAN, 1);

    fbText(5, 210, "BEKENSTEIN", NEON_GREEN, 1);
    fbText(5, 225, "BOUND", NEON_GREEN, 1);

    fbText(120, 190, "SURFACE=", NEON_PURPLE, 1);
    fbText(120, 205, "VOLUME", NEON_PURPLE, 1);
    fbText(120, 220, "'T HOOFT 1993", DIM_CYAN, 1);

    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 37: SCHRODINGER STATE =============
// Quantum superposition cat state
void face_Schrodinger() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "SCHRODINGER", NEON_CYAN, 2);
    fbShimmer(2, 18);

    // Superposition state (both alive and dead until observed)
    bool collapsed = touchPressed;  // Observation collapses state

    int cx = 120, cy = 100;

    // Box
    fbRect(cx - 50, cy - 40, 100, 80, NEON_CYAN);

    if (!collapsed) {
        // Superposition - flickering cat
        if ((frame / 5) % 2 == 0) {
            // Alive state
            fbText(cx - 20, cy - 10, ":3", NEON_GREEN, 3);
            fbText(cx - 35, cy + 20, "ALIVE", NEON_GREEN, 1);
        } else {
            // Dead state
            fbText(cx - 20, cy - 10, "X_X", NEON_RED, 3);
            fbText(cx - 30, cy + 20, "DEAD", NEON_RED, 1);
        }

        fbText(cx - 45, cy - 35, "?+?", NEON_PURPLE, 2);
    } else {
        // Collapsed state
        if (random(2) == 0) {
            fbText(cx - 20, cy - 10, ":3", NEON_GREEN, 3);
            fbText(cx - 35, cy + 20, "ALIVE!", NEON_GREEN, 2);
        } else {
            fbText(cx - 20, cy - 10, "X_X", NEON_RED, 3);
            fbText(cx - 30, cy + 20, "DEAD!", NEON_RED, 2);
        }
    }

    // Wave function
    fbText(5, 170, "PSI:", NEON_CYAN, 1);
    fbText(40, 170, collapsed ? "COLLAPSED" : "SUPERPOSED", collapsed ? NEON_GREEN : NEON_PURPLE, 1);

    // Probability
    fbText(5, 190, "P(ALIVE):", NEON_GREEN, 1);
    fbText(90, 190, collapsed ? "100% or 0%" : "50%", NEON_GREEN, 1);

    fbText(5, 210, "TAP TO", NEON_YELLOW, 1);
    fbText(60, 210, "OBSERVE", NEON_YELLOW, 1);

    fbText(5, 228, "1935 PARADOX", DIM_CYAN, 1);

    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 38: ENTROPY METER =============
// Thermodynamic entropy visualization
void face_Entropy() {
    fbClear(BLACK);
    char buf[32];

    fbTextCenter(2, "ENTROPY", NEON_RED, 2);
    fbShimmer(2, 18);

    // Calculate local entropy from entity disorder
    float entropy = 0;
    for (int i = 0; i < entityCount; i++) {
        if (millis() - entities[i].lastSeen < 5000) {
            // More varied distances = higher entropy
            entropy += abs(entities[i].distanceM - avgDistanceM);
        }
    }
    entropy = min(100.0f, entropy * 10 + 30);

    // Particle visualization
    int cx = 120, cy = 100;

    for (int p = 0; p < 50; p++) {
        float speed = entropy / 50.0;
        float angle = p * 0.5 + frame * speed * 0.1;
        float r = 20 + (p % 5) * 15 + sin(frame * 0.1 + p) * 5 * speed;

        int px = cx + cos(angle) * r;
        int py = cy + sin(angle) * r;

        uint16_t col = (entropy > 70) ? NEON_RED :
                       (entropy > 40) ? NEON_YELLOW : NEON_CYAN;
        fbFillCircle(px, py, 2, col);
    }

    // Container
    fbCircle(cx, cy, 75, GRID_DIM);

    // Stats
    fbLine(0, 180, 240, 180, NEON_RED);

    sprintf(buf, "S:%.1f J/K", entropy);
    fbText(5, 185, buf, NEON_CYAN, 2);

    fbText(5, 210, "DISORDER:", NEON_PURPLE, 1);
    const char* disorder = (entropy > 70) ? "HIGH" :
                          (entropy > 40) ? "MEDIUM" : "LOW";
    uint16_t disCol = (entropy > 70) ? NEON_RED :
                      (entropy > 40) ? NEON_YELLOW : NEON_GREEN;
    fbText(90, 210, disorder, disCol, 1);

    fbText(5, 228, "BOLTZMANN", DIM_CYAN, 1);

    fbGlitch(1);
    fbScanlines();
}

// Forward declarations for remaining faces
void face_Precognition();
void face_Telepathy();
void face_Akashic();
void face_Vortex();
void face_Plasma();
void face_Antimatter();
void face_StringTheory();
void face_Multiverse();
void face_ZeroPoint();
void face_Tachyon();
void face_Resonance();
void face_Synchronicity();
void face_Matrix();
void face_Simulation();
void face_Consciousness();
void face_Psi();
void face_Remote();
void face_Astral();
void face_Lucid();
void face_OBE();

// ============= NEW FACE 39: PRECOGNITION =============
void face_Precognition() {
    fbClear(BLACK);
    char buf[32];
    fbTextCenter(2, "PRECOG", NEON_PINK, 2);
    fbShimmer(2, 18);

    // Predict future entity positions
    fbText(5, 30, "FUTURE SCAN:", NEON_CYAN, 1);

    int predCount = 0;
    for (int t = 1; t <= 3; t++) {
        int ty = 50 + (t - 1) * 55;
        sprintf(buf, "+%d SEC:", t);
        fbText(5, ty, buf, NEON_PURPLE, 1);

        // Predicted entity positions
        int maxPred = (entityCount < 3) ? entityCount : 3;
        for (int i = 0; i < maxPred; i++) {
            if (millis() - entities[i].lastSeen > 5000) continue;

            float futureX = entities[i].x + cos(frame * 0.02 + i) * t * 5;
            float futureY = entities[i].y + sin(frame * 0.02 + i) * t * 5;
            float futureDist = entities[i].distanceM + sin(frame * 0.01) * t * 0.5;

            sprintf(buf, "E%d:%.1fM", i, futureDist);
            fbText(70 + i * 60, ty, buf, NEON_GREEN, 1);

            predCount++;
        }

        // Probability confidence
        int conf = 95 - t * 20;
        sprintf(buf, "%d%%", conf);
        fbText(200, ty, buf, (conf > 50) ? NEON_GREEN : NEON_YELLOW, 1);
    }

    fbLine(0, 200, 240, 200, NEON_PINK);
    sprintf(buf, "PREDICTIONS:%d", predCount);
    fbText(5, 205, buf, NEON_CYAN, 1);
    fbText(5, 222, "BAYESIAN PROJ", DIM_CYAN, 1);
    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 40: TELEPATHY =============
void face_Telepathy() {
    fbClear(BLACK);
    char buf[32];
    fbTextCenter(2, "TELEPATHY", NEON_PURPLE, 2);
    fbShimmer(2, 18);

    // Mind-to-mind "connection" visualization
    int cx = 120, cy = 100;

    // Two brain silhouettes
    fbCircle(50, cy, 25, NEON_CYAN);
    fbCircle(190, cy, 25, NEON_PINK);

    // Thought wave between them
    for (int x = 80; x < 160; x++) {
        float wave = sin(x * 0.2 - frame * 0.2) * 15;
        int y = cy + wave;
        uint16_t col = ((x + frame) % 20 < 10) ? NEON_CYAN : NEON_PINK;
        fbPixel(x, y, col);
        fbPixel(x, y + 1, col);
    }

    // Connection strength based on nearby entities
    float telepathyStr = 0;
    for (int i = 0; i < entityCount; i++) {
        if (entities[i].bioSimilarity > 0.6) telepathyStr += 0.2;
    }
    telepathyStr = min(1.0f, telepathyStr);

    sprintf(buf, "SYNC:%.0f%%", telepathyStr * 100);
    fbTextCenter(150, buf, NEON_GREEN, 2);

    fbText(5, 180, "BRAINWAVE", NEON_CYAN, 1);
    fbText(5, 195, "COHERENCE", NEON_CYAN, 1);
    fbText(120, 180, "GANZFELD", DIM_CYAN, 1);
    fbText(120, 195, "PROTOCOL", DIM_CYAN, 1);

    fbLine(0, 210, 240, 210, NEON_PURPLE);
    fbText(5, 215, "PSI RESEARCH", NEON_PINK, 1);
    fbGlitch(2);
    fbScanlines();
}

// ============= NEW FACE 41: AKASHIC RECORDS =============
void face_Akashic() {
    fbClear(BLACK);
    char buf[32];
    fbTextCenter(2, "AKASHIC", NEON_YELLOW, 2);
    fbShimmer(2, 18);

    // Cosmic library visualization
    int cx = 120, cy = 100;

    // Glowing orb of knowledge
    for (int r = 50; r > 10; r -= 5) {
        uint16_t col = (r > 40) ? NEON_PURPLE : (r > 25) ? NEON_PINK : NEON_YELLOW;
        fbCircle(cx, cy, r, col);
    }

    // Data streams flowing into orb
    for (int stream = 0; stream < 6; stream++) {
        float angle = stream * PI / 3 + frame * 0.02;
        for (int d = 60; d > 15; d -= 4) {
            int px = cx + cos(angle) * d;
            int py = cy + sin(angle) * d;
            fbPixel(px, py, NEON_CYAN);
        }
    }

    // Records count
    unsigned long records = millis() / 10 + entityCount * 1000;
    sprintf(buf, "RECORDS:");
    fbText(5, 170, buf, NEON_PURPLE, 1);
    sprintf(buf, "%lu", records);
    fbText(80, 170, buf, NEON_CYAN, 1);

    fbText(5, 190, "UNIVERSAL", NEON_YELLOW, 1);
    fbText(5, 205, "MEMORY FIELD", NEON_YELLOW, 1);

    fbText(120, 190, "THEOSOPHIC", DIM_CYAN, 1);
    fbText(120, 205, "CONCEPT", DIM_CYAN, 1);

    fbLine(0, 220, 240, 220, NEON_PURPLE);
    fbText(5, 225, "ALL KNOWLEDGE", NEON_PINK, 1);
    fbGlitch(3);
    fbScanlines();
}

// ============= NEW FACE 42-58: Quick spectacular faces =============

void face_Vortex() {
    fbClear(BLACK);
    fbTextCenter(2, "VORTEX", NEON_CYAN, 2);
    fbShimmer(2, 18);
    int cx = 120, cy = 120;
    for (int r = 10; r < 100; r += 3) {
        float angle = r * 0.15 + frame * 0.1;
        int px = cx + cos(angle) * r;
        int py = cy + sin(angle) * r;
        uint16_t col = (r % 9 < 3) ? NEON_CYAN : (r % 9 < 6) ? NEON_PURPLE : NEON_PINK;
        fbFillCircle(px, py, 3, col);
    }
    fbText(5, 220, "ENERGY SPIRAL", NEON_GREEN, 1);
    fbScanlines();
}

void face_Plasma() {
    fbClear(BLACK);
    fbTextCenter(2, "PLASMA", NEON_ORANGE, 2);
    fbShimmer(2, 18);
    for (int y = 30; y < 200; y += 4) {
        for (int x = 10; x < 230; x += 4) {
            float v = sin(x * 0.05 + frame * 0.1) + sin(y * 0.05 + frame * 0.08);
            v += sin((x + y) * 0.03 + frame * 0.05);
            int intensity = (v + 3) * 40;
            uint16_t col = (intensity > 150) ? NEON_ORANGE : (intensity > 100) ? NEON_YELLOW : NEON_RED;
            fbRect(x, y, 3, 3, col);
        }
    }
    fbText(5, 210, "STATE IV MATTER", NEON_CYAN, 1);
    fbScanlines();
}

void face_Antimatter() {
    fbClear(BLACK);
    fbTextCenter(2, "ANTIMATTER", NEON_RED, 2);
    fbShimmer(2, 18);
    int cx = 120, cy = 100;
    fbCircle(cx - 30, cy, 20, NEON_CYAN);
    fbText(cx - 35, cy - 5, "e-", NEON_CYAN, 1);
    fbCircle(cx + 30, cy, 20, NEON_RED);
    fbText(cx + 25, cy - 5, "e+", NEON_RED, 1);
    if ((frame / 10) % 3 == 0) {
        for (int r = 5; r < 30; r += 5) {
            fbCircle(cx, cy, r, NEON_YELLOW);
        }
        fbText(cx - 25, cy + 30, "ANNIHILATION!", NEON_YELLOW, 1);
    }
    fbText(5, 180, "DIRAC 1928", DIM_CYAN, 1);
    fbText(5, 200, "E=MC2 RELEASE", NEON_PINK, 1);
    fbScanlines();
}

void face_StringTheory() {
    fbClear(BLACK);
    fbTextCenter(2, "STRINGS", NEON_GREEN, 2);
    fbShimmer(2, 18);
    for (int s = 0; s < 8; s++) {
        int sy = 40 + s * 22;
        for (int x = 20; x < 220; x++) {
            float vib = sin(x * 0.1 + frame * 0.15 + s * 0.5) * 8;
            fbPixel(x, sy + vib, chakraColors[s % 7]);
        }
    }
    fbText(5, 200, "11 DIMENSIONS", NEON_CYAN, 1);
    fbText(5, 215, "WITTEN M-THEORY", DIM_CYAN, 1);
    fbScanlines();
}

void face_Multiverse() {
    fbClear(BLACK);
    fbTextCenter(2, "MULTIVERSE", NEON_PURPLE, 2);
    fbShimmer(2, 18);
    for (int u = 0; u < 5; u++) {
        int ux = 40 + u * 45;
        int uy = 100 + sin(frame * 0.05 + u) * 20;
        int ur = 15 + u * 3;
        fbCircle(ux, uy, ur, chakraColors[u]);
        if (u < 4) fbLine(ux + ur, uy, ux + 45 - ur, uy, GRID_DIM);
    }
    char buf[32];
    sprintf(buf, "UNIVERSE:%d", (frame / 50) % 999 + 1);
    fbText(5, 180, buf, NEON_CYAN, 1);
    fbText(5, 200, "EVERETT III", DIM_CYAN, 1);
    fbText(5, 215, "MANY WORLDS", NEON_PINK, 1);
    fbScanlines();
}

void face_ZeroPoint() {
    fbClear(BLACK);
    fbTextCenter(2, "ZERO POINT", NEON_CYAN, 2);
    fbShimmer(2, 18);
    int cx = 120, cy = 100;
    for (int i = 0; i < 100; i++) {
        float angle = random(360) * PI / 180;
        float r = random(5, 60);
        int px = cx + cos(angle) * r;
        int py = cy + sin(angle) * r;
        fbPixel(px, py, (random(2) == 0) ? NEON_CYAN : NEON_PURPLE);
    }
    fbCircle(cx, cy, 60, NEON_GREEN);
    fbText(5, 180, "VACUUM ENERGY", NEON_YELLOW, 1);
    fbText(5, 200, "CASIMIR EFFECT", NEON_CYAN, 1);
    char buf[32];
    sprintf(buf, "E:%.2e J/m3", 1.0e113);
    fbText(5, 215, buf, NEON_PINK, 1);
    fbScanlines();
}

void face_Tachyon() {
    fbClear(BLACK);
    fbTextCenter(2, "TACHYON", NEON_YELLOW, 2);
    fbShimmer(2, 18);
    for (int t = 0; t < 10; t++) {
        int tx = (240 - (frame * 8 + t * 20) % 280);
        int ty = 60 + t * 15;
        for (int trail = 0; trail < 30; trail++) {
            fbPixel(tx + trail, ty, (trail < 10) ? NEON_YELLOW : NEON_ORANGE);
        }
        fbFillCircle(tx, ty, 3, WHITE);
    }
    fbText(5, 190, "V > C", NEON_RED, 2);
    fbText(5, 215, "FTL PARTICLE", NEON_CYAN, 1);
    fbScanlines();
}

void face_Resonance() {
    fbClear(BLACK);
    fbTextCenter(2, "RESONANCE", NEON_GREEN, 2);
    fbShimmer(2, 18);
    int cx = 120, cy = 100;
    for (int h = 1; h <= 5; h++) {
        float freq = h * 0.2;
        for (int x = 20; x < 220; x++) {
            float y = sin(x * freq + frame * 0.1 * h) * (30 / h);
            fbPixel(x, cy + y, chakraColors[h]);
        }
    }
    fbText(5, 180, "HARMONICS", NEON_CYAN, 1);
    fbText(5, 200, "SCHUMANN 7.83Hz", NEON_GREEN, 1);
    char buf[32];
    sprintf(buf, "FREQ:%.2f Hz", 7.83 + sin(frame * 0.01) * 0.5);
    fbText(5, 215, buf, NEON_PINK, 1);
    fbScanlines();
}

void face_Synchronicity() {
    fbClear(BLACK);
    fbTextCenter(2, "SYNCHRO", NEON_PINK, 2);
    fbShimmer(2, 18);
    char buf[32];
    sprintf(buf, "%02d:%02d:%02d", rtcHour, rtcMin, rtcSec);
    fbTextCenter(60, buf, NEON_CYAN, 3);
    bool sync = (rtcHour == rtcMin) || (rtcMin == rtcSec) || (rtcHour == 11 && rtcMin == 11);
    if (sync) {
        fbTextCenter(110, "SYNC EVENT!", NEON_GREEN, 2);
        for (int r = 30; r < 60; r += 5) fbCircle(120, 90, r, NEON_PINK);
    }
    int events = (millis() / 60000) % 100;
    sprintf(buf, "EVENTS:%d", events);
    fbText(5, 180, buf, NEON_PURPLE, 1);
    fbText(5, 200, "JUNG 1952", DIM_CYAN, 1);
    fbText(5, 215, "MEANINGFUL", NEON_CYAN, 1);
    fbScanlines();
}

void face_Matrix() {
    fbClear(BLACK);
    fbTextCenter(2, "MATRIX", NEON_GREEN, 2);
    static int drops[24] = {0};
    for (int col = 0; col < 24; col++) {
        drops[col] += 3 + random(5);
        if (drops[col] > 240) drops[col] = random(50);
        for (int trail = 0; trail < 15; trail++) {
            int y = drops[col] - trail * 10;
            if (y > 20 && y < 240) {
                char c = 'A' + random(26);
                uint16_t col_c = (trail == 0) ? WHITE : (trail < 5) ? NEON_GREEN : DIM_GREEN;
                fbChar(col * 10, y, c, col_c, 1);
            }
        }
    }
    fbText(5, 225, "SIMULATION?", NEON_CYAN, 1);
    fbScanlines();
}

void face_Simulation() {
    fbClear(BLACK);
    fbTextCenter(2, "SIMULATN", NEON_CYAN, 2);
    fbShimmer(2, 18);
    for (int y = 30; y < 180; y += 20) {
        for (int x = 10; x < 230; x += 20) {
            bool glitch = (random(100) < 5);
            if (glitch) {
                fbRect(x, y, 18, 18, NEON_RED);
                fbText(x + 2, y + 5, "ERR", NEON_RED, 1);
            } else {
                fbRect(x, y, 18, 18, GRID_DIM);
            }
        }
    }
    char buf[32];
    sprintf(buf, "RENDER:%.1f FPS", 1000.0 / 40);
    fbText(5, 190, buf, NEON_GREEN, 1);
    fbText(5, 210, "BOSTROM 2003", DIM_CYAN, 1);
    fbText(5, 225, "HYPOTHESIS", NEON_PURPLE, 1);
    fbScanlines();
}

void face_Consciousness() {
    fbClear(BLACK);
    fbTextCenter(2, "CONSCIOUS", NEON_PURPLE, 2);
    fbShimmer(2, 18);
    int cx = 120, cy = 100;
    for (int layer = 0; layer < 5; layer++) {
        int r = 20 + layer * 15;
        for (int a = 0; a < 360; a += 5) {
            float angle = a * PI / 180 + frame * 0.02 * (layer + 1);
            int px = cx + cos(angle) * r;
            int py = cy + sin(angle) * r;
            fbPixel(px, py, chakraColors[layer]);
        }
    }
    fbFillCircle(cx, cy, 10, WHITE);
    fbText(5, 180, "PHI:", NEON_CYAN, 1);
    char buf[32];
    sprintf(buf, "%.2f", 3.5 + sin(frame * 0.01));
    fbText(40, 180, buf, NEON_GREEN, 1);
    fbText(5, 200, "TONONI IIT", DIM_CYAN, 1);
    fbText(5, 215, "INTEGRATED INFO", NEON_PINK, 1);
    fbScanlines();
}

void face_Psi() {
    fbClear(BLACK);
    fbTextCenter(2, "PSI FIELD", NEON_PINK, 2);
    fbShimmer(2, 18);
    int cx = 120, cy = 100;
    fbText(cx - 15, cy - 10, "PSI", NEON_CYAN, 3);
    for (int wave = 0; wave < 360; wave += 10) {
        float angle = wave * PI / 180;
        float r = 50 + sin(wave * 0.1 + frame * 0.1) * 20;
        int px = cx + cos(angle) * r;
        int py = cy + sin(angle) * r;
        fbPixel(px, py, NEON_PURPLE);
    }
    char buf[32];
    float psiStr = 0.5 + sin(frame * 0.02) * 0.3;
    sprintf(buf, "FIELD:%.0f%%", psiStr * 100);
    fbText(5, 180, buf, NEON_GREEN, 1);
    fbText(5, 200, "PARAPSYCH", DIM_CYAN, 1);
    fbText(5, 215, "RESEARCH", NEON_CYAN, 1);
    fbScanlines();
}

void face_Remote() {
    fbClear(BLACK);
    fbTextCenter(2, "REMOTE VW", NEON_CYAN, 2);
    fbShimmer(2, 18);
    fbRect(40, 50, 160, 100, NEON_PURPLE);
    int targetX = 120 + sin(frame * 0.03) * 50;
    int targetY = 100 + cos(frame * 0.04) * 30;
    fbCircle(targetX, targetY, 15, NEON_GREEN);
    fbLine(targetX - 20, targetY, targetX + 20, targetY, NEON_GREEN);
    fbLine(targetX, targetY - 20, targetX, targetY + 20, NEON_GREEN);
    char buf[32];
    sprintf(buf, "TARGET:%03d", (frame / 10) % 1000);
    fbText(5, 170, buf, NEON_PINK, 1);
    fbText(5, 190, "SRI PROGRAM", DIM_CYAN, 1);
    fbText(5, 210, "STARGATE", NEON_YELLOW, 1);
    fbScanlines();
}

void face_Astral() {
    fbClear(BLACK);
    fbTextCenter(2, "ASTRAL", NEON_PURPLE, 2);
    fbShimmer(2, 18);
    int cx = 120, cy = 90;
    fbCircle(cx, cy, 20, NEON_CYAN);
    fbFillCircle(cx, cy, 15, DIM_CYAN);
    int astralX = cx + sin(frame * 0.05) * 40;
    int astralY = cy - 30 + cos(frame * 0.03) * 20;
    for (int r = 15; r > 5; r -= 2) {
        fbCircle(astralX, astralY, r, NEON_PINK);
    }
    for (int i = 0; i < 10; i++) {
        float t = i / 10.0;
        int lx = cx + (astralX - cx) * t;
        int ly = cy + (astralY - cy) * t;
        fbPixel(lx, ly, NEON_PURPLE);
    }
    fbText(5, 160, "SILVER CORD", NEON_CYAN, 1);
    fbText(5, 180, "PROJECTION", NEON_PINK, 1);
    fbText(5, 200, "MONROE 1971", DIM_CYAN, 1);
    fbScanlines();
}

void face_Lucid() {
    fbClear(BLACK);
    fbTextCenter(2, "LUCID", NEON_GREEN, 2);
    fbShimmer(2, 18);
    fbCircle(120, 90, 40, NEON_CYAN);
    fbCircle(120, 90, 35, NEON_PURPLE);
    if ((frame / 20) % 2 == 0) {
        fbFillCircle(120, 90, 30, BLACK);
        fbText(105, 82, "REM", NEON_GREEN, 1);
    } else {
        fbFillCircle(120, 90, 30, DIM_CYAN);
        fbText(100, 82, "WAKE", NEON_GREEN, 1);
    }
    char buf[32];
    int lucidity = 50 + sin(frame * 0.02) * 40;
    sprintf(buf, "LUCID:%d%%", lucidity);
    fbText(5, 160, buf, NEON_PINK, 1);
    fbText(5, 180, "LABERGE 1985", DIM_CYAN, 1);
    fbText(5, 200, "DREAM AWARE", NEON_CYAN, 1);
    fbScanlines();
}

void face_OBE() {
    fbClear(BLACK);
    fbTextCenter(2, "OBE", NEON_CYAN, 2);
    fbShimmer(2, 18);
    int bodyX = 80, bodyY = 120;
    fbRect(bodyX - 10, bodyY - 30, 20, 60, DIM_CYAN);
    fbCircle(bodyX, bodyY - 40, 10, DIM_CYAN);
    float separation = sin(frame * 0.03) * 30 + 40;
    int spiritX = bodyX + separation;
    int spiritY = bodyY - separation / 2;
    fbRect(spiritX - 10, spiritY - 30, 20, 60, NEON_PURPLE);
    fbCircle(spiritX, spiritY - 40, 10, NEON_PURPLE);
    for (int i = 0; i < 5; i++) {
        float t = i / 5.0;
        fbPixel(bodyX + (spiritX - bodyX) * t, bodyY + (spiritY - bodyY) * t, NEON_PINK);
    }
    char buf[32];
    sprintf(buf, "SEP:%.0fcm", separation * 2);
    fbText(5, 180, buf, NEON_GREEN, 1);
    fbText(5, 200, "BLANKE 2002", DIM_CYAN, 1);
    fbText(5, 215, "TPJ RESEARCH", NEON_CYAN, 1);
    fbScanlines();
}

void face_Debug() {
    fbClear(BLACK);
    fbText(10, 2, "TOUCH DEBUG", NEON_GREEN, 2);

    char buf[48];

    // Read live data using ft6336ReadData (reads 5 bytes from reg 0x02)
    // LilyGO method: touchData[0]=TD_STATUS, [1-2]=X, [3-4]=Y
    uint8_t vendorId = 0, chipId = 0, tdStatus = 0;
    if (touchAddr && ft6336ReadData()) {
        tdStatus = touchData[0] & 0x0F;  // Touch points now at index 0
        vendorId = ft6336Read(FT6336_REG_VENDOR_ID);
        chipId = ft6336Read(FT6336_REG_CHIP_ID);
    }

    // Show I2C devices found on touch bus
    sprintf(buf, "I2C 39/40: %d devs", i2cFoundCount);
    fbText(10, 22, buf, NEON_CYAN, 1);
    for (int i = 0; i < i2cFoundCount && i < 4; i++) {
        sprintf(buf, "%02X", i2cFound[i]);
        fbText(130 + i * 25, 22, buf, NEON_GREEN, 1);
    }

    // Show touch controller status with vendor and chip ID
    sprintf(buf, "VID:%02X CID:%02X %s", vendorId, chipId, touchAddr ? "OK" : "NO");
    fbText(10, 38, buf, touchAddr ? NEON_GREEN : NEON_RED, 1);

    // Show raw buffer data (first 8 bytes)
    sprintf(buf, "RAW:%02X %02X %02X %02X", touchData[0], touchData[1], touchData[2], touchData[3]);
    fbText(10, 52, buf, NEON_YELLOW, 1);
    sprintf(buf, "    %02X %02X %02X %02X", touchData[4], touchData[5], touchData[6], touchData[7]);
    fbText(10, 64, buf, NEON_YELLOW, 1);

    // GPIO16 interrupt pin state
    bool intState = digitalRead(TOUCH_INT);
    sprintf(buf, "INT:%s TD:%d", intState ? "HIGH" : "LOW!", tdStatus);
    fbText(10, 80, buf, intState ? DIM_CYAN : NEON_YELLOW, 1);

    // Touch state - BIG display
    sprintf(buf, "TOUCH: %s", touchPressed ? "YES!" : "NO");
    fbText(10, 98, buf, touchPressed ? NEON_GREEN : WHITE, 2);

    // Coordinates
    sprintf(buf, "X:%3d Y:%3d", touchX, touchY);
    fbText(10, 120, buf, NEON_CYAN, 2);

    // Draw touch zone indicator
    fbRect(10, 145, 220, 70, DIM_CYAN);
    fbText(80, 148, "TOUCH AREA", DIM_GREEN, 1);

    // Draw touch point if touching (scaled to box)
    if (touchPressed || tdStatus > 0) {
        // Map 0-240 to box area (10-230 x, 145-215 y)
        int dx = 10 + (touchX * 220 / 240);
        int dy = 145 + (touchY * 70 / 240);
        if (dx > 10 && dx < 230 && dy > 145 && dy < 215) {
            fbFillCircle(dx, dy, 8, NEON_RED);
            fbCircle(dx, dy, 12, NEON_YELLOW);
        }
    }

    // Swipe and face info
    sprintf(buf, "SWP:%d FACE:%d BTN=NEXT", swipeDir, currentFace);
    fbText(10, 222, buf, NEON_GREEN, 1);
}

void drawCurrentFace() {
    switch (currentFace) {
        case 0: face_Debug(); break;  // Debug first for testing
        case 1: face_Clock(); break;
        case 2: face_WalkieTalkie(); break;
        case 3: face_Vitals(); break;
        case 4: face_Radar(); break;
        case 5: face_Proximity(); break;
        case 6: face_Alien(); break;
        case 7: face_RubbleDetector(); break;
        case 8: face_Echolocation(); break;
        case 9: face_HealthMap(); break;
        case 10: face_TribeFinder(); break;
        case 11: face_Radiation(); break;
        case 12: face_Drone(); break;
        case 13: face_PartnerLoc(); break;
        case 14: face_Bio(); break;
        case 15: face_Profile(); break;
        case 16: face_Neural(); break;
        case 17: face_System(); break;
        case 18: face_About(); break;
        // NEW 40 SPECTACULAR FACES
        case 19: face_BodyScanner(); break;
        case 20: face_Sonar(); break;
        case 21: face_Quantum(); break;
        case 22: face_Ternary(); break;
        case 23: face_NeuralNet(); break;
        case 24: face_Brainwave(); break;
        case 25: face_Chakra(); break;
        case 26: face_Aura(); break;
        case 27: face_Circadian(); break;
        case 28: face_Electrodermal(); break;
        case 29: face_DarkMatter(); break;
        case 30: face_GravWave(); break;
        case 31: face_Neutrino(); break;
        case 32: face_Morphic(); break;
        case 33: face_Timeline(); break;
        case 34: face_Bioelectric(); break;
        case 35: face_DNA(); break;
        case 36: face_Holographic(); break;
        case 37: face_Schrodinger(); break;
        case 38: face_Entropy(); break;
        case 39: face_Precognition(); break;
        case 40: face_Telepathy(); break;
        case 41: face_Akashic(); break;
        case 42: face_Vortex(); break;
        case 43: face_Plasma(); break;
        case 44: face_Antimatter(); break;
        case 45: face_StringTheory(); break;
        case 46: face_Multiverse(); break;
        case 47: face_ZeroPoint(); break;
        case 48: face_Tachyon(); break;
        case 49: face_Resonance(); break;
        case 50: face_Synchronicity(); break;
        case 51: face_Matrix(); break;
        case 52: face_Simulation(); break;
        case 53: face_Consciousness(); break;
        case 54: face_Psi(); break;
        case 55: face_Remote(); break;
        case 56: face_Astral(); break;
        case 57: face_Lucid(); break;
        case 58: face_OBE(); break;
        case 59: face_Triangulation(); break;  // Dual-watch target tracking
        default: face_Debug(); break;
    }
    pushFramebuffer();
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("HYPERLOG v15.0 BOOT");

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    Serial.println("I2C OK");
    initPMIC();
    Serial.println("PMIC OK");
    delay(100);

    rtcWrite(21, 50, 0, 8, 1, 26);
    Serial.println("RTC OK");

    initDisplay();
    Serial.println("DISPLAY OK");
    initTouch();
    Serial.println("TOUCH OK");
    initLoRa();
    Serial.println("LORA OK");
    startSniffer();
    Serial.println("SNIFFER OK");

    pinMode(BTN_1, INPUT_PULLUP);

    // BLACK MIRROR laughing/winking smiley - exact style from the show
    for (int anim = 0; anim < 80; anim++) {  // 80 frames animation
        fbClear(BLACK);

        int smx = 120, smy = 60;

        // Black Mirror style - white face on black background
        int faceR = 45 + sin(anim * 0.15) * 3;

        // Face circle - thick outline like Black Mirror
        for (int r = faceR; r > faceR - 3; r--) {
            fbCircle(smx, smy, r, WHITE);
        }

        // Wink cycle: frames 20-35 left wink, frames 50-65 right wink
        bool leftWink = (anim > 20 && anim < 35);
        bool rightWink = (anim > 50 && anim < 65);

        // LEFT EYE - Black Mirror laughing style (curved ^_^)
        if (leftWink) {
            // Winking shut - curved line like ^
            for (int i = -8; i <= 8; i++) {
                int ey = smy - 12 + abs(i) / 2;
                fbPixel(smx - 15 + i, ey, WHITE);
                fbPixel(smx - 15 + i, ey + 1, WHITE);
            }
        } else {
            // Open laughing eye - curved up ◠
            for (int i = -7; i <= 7; i++) {
                int ey = smy - 10 - (7 - abs(i)) / 2;
                fbPixel(smx - 15 + i, ey, WHITE);
                fbPixel(smx - 15 + i, ey + 1, WHITE);
            }
        }

        // RIGHT EYE
        if (rightWink) {
            // Winking shut
            for (int i = -8; i <= 8; i++) {
                int ey = smy - 12 + abs(i) / 2;
                fbPixel(smx + 15 + i, ey, WHITE);
                fbPixel(smx + 15 + i, ey + 1, WHITE);
            }
        } else {
            // Open laughing eye
            for (int i = -7; i <= 7; i++) {
                int ey = smy - 10 - (7 - abs(i)) / 2;
                fbPixel(smx + 15 + i, ey, WHITE);
                fbPixel(smx + 15 + i, ey + 1, WHITE);
            }
        }

        // BIG LAUGHING SMILE - Black Mirror style ◡
        for (int i = -22; i <= 22; i++) {
            int curve = (22 - abs(i)) / 3;  // Deep curve = big smile
            int my = smy + 15 + curve;
            fbPixel(smx + i, my, WHITE);
            fbPixel(smx + i, my + 1, WHITE);
            fbPixel(smx + i, my + 2, WHITE);
        }

        // Teeth hint (Black Mirror has slight teeth)
        if (anim % 20 < 15) {
            fbLine(smx - 10, smy + 18, smx + 10, smy + 18, BLACK);
        }

        // Glitch effect - horizontal displacement
        if (anim % 12 < 3) {
            int gy = smy - 30 + random(60);
            int shift = random(-15, 15);
            for (int gx = smx - 40; gx < smx + 40; gx++) {
                if (random(3) == 0) {
                    uint16_t gc = (random(2) == 0) ? NEON_CYAN : NEON_PURPLE;
                    fbPixel(gx + shift, gy, gc);
                }
            }
        }

        // Scanline flicker
        if (anim % 6 < 2) {
            int scanY = (anim * 5) % 50 + smy - 25;
            fbLine(smx - 45, scanY, smx + 45, scanY, NEON_CYAN);
        }

        // HYPERLOG BIOELECTRIC heading
        fbTextCenter(115, "HYPERLOG", NEON_PURPLE, 2);
        fbTextCenter(140, "BIOELECTRIC", NEON_CYAN, 2);
        fbTextCenter(165, "NEURAL OCEAN", NEON_GREEN, 2);
        fbTextCenter(195, "V15.0", NEON_PINK, 2);

        // Laughing/winking text smiley synced with face
        const char* smileys[] = {":D", ";D", ":D", ";)"};
        int sIdx = 0;
        if (leftWink) sIdx = 1;
        else if (rightWink) sIdx = 3;
        else sIdx = (anim / 10) % 2 == 0 ? 0 : 2;
        fbTextCenter(220, smileys[sIdx], WHITE, 2);

        fbScanlines();
        pushFramebuffer();
        delay(42);  // ~24 FPS smooth
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
            Serial.println("Swipe: Next face");
        } else if (swipeDir == 1) {  // Swipe right = prev
            currentFace = (currentFace + TOTAL_FACES - 1) % TOTAL_FACES;
            lastFaceChange = millis();
            Serial.println("Swipe: Prev face");
        }
    }

    // Handle tap - any tap cycles to next face
    static bool lastTouchState = false;
    static unsigned long touchStartTime = 0;
    if (touchPressed && !lastTouchState) {
        touchStartTime = millis();
    }
    if (!touchPressed && lastTouchState) {
        // Touch released - check if it was a tap (short touch, no movement)
        unsigned long touchDur = millis() - touchStartTime;
        int dx = abs(touchX - lastTouchX);
        int dy = abs(touchY - lastTouchY);
        if (touchDur < 300 && dx < 30 && dy < 30) {
            // Short tap with little movement = cycle face
            currentFace = (currentFace + 1) % TOTAL_FACES;
            lastFaceChange = millis();
            Serial.println("Tap: Next face");
        }
    }
    lastTouchState = touchPressed;

    // Handle PMU button (AXP2101)
    if (readPMUButton()) {
        currentFace = (currentFace + 1) % TOTAL_FACES;
        lastFaceChange = millis();
        Serial.println("Button: Next face");
    }

    // Also check GPIO0 as fallback (some boards have it)
    static bool gpio0Pressed = false;
    bool gpio0State = (digitalRead(BTN_1) == LOW);
    if (gpio0State && !gpio0Pressed) {
        gpio0Pressed = true;
    }
    if (!gpio0State && gpio0Pressed) {
        gpio0Pressed = false;
        currentFace = (currentFace + 1) % TOTAL_FACES;
        lastFaceChange = millis();
        Serial.println("GPIO0: Next face");
    }

    // AGGRESSIVE AUTO-PAIRING: Rapidly search for partner
    // Random interval 200-500ms to avoid collision
    if (!loraPaired) {
        pairingMode = true;
        static unsigned long lastPairBeacon = 0;
        static unsigned long nextInterval = 300;
        if (millis() - lastPairBeacon > nextInterval) {
            sendPairRequest();
            lastPairBeacon = millis();
            nextInterval = 200 + (esp_random() % 300);  // 200-500ms random
            // Only log occasionally to avoid spam
            static int beaconCount = 0;
            if (++beaconCount % 10 == 0) {
                Serial.printf("LoRa: Beacon #%d (interval %dms)\n", beaconCount, nextInterval);
            }
        }
    }

    // Process LoRa messages - ALWAYS check, even during TX
    processLoRa();

    // When paired, exchange data regularly
    static unsigned long lastLocSend = 0;
    static unsigned long lastSensorSend = 0;
    static unsigned long lastHeartbeat = 0;

    if (loraPaired) {
        // Send location every 5 seconds
        if (millis() - lastLocSend > 5000) {
            sendLocation();
            lastLocSend = millis();
        }

        // Send sensor data every 3 seconds
        if (millis() - lastSensorSend > 3000) {
            sendSensorData();
            lastSensorSend = millis();
        }

        // Send heartbeat every 2 seconds
        if (millis() - lastHeartbeat > 2000) {
            sendHeartbeat();
            lastHeartbeat = millis();
        }

        // Check if partner went silent (unpair after 30s)
        if (millis() - lastPartnerUpdate > 30000) {
            loraPaired = false;
            partnerSensors.valid = false;
            Serial.println("LoRa: Partner timeout - unpairing");
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
