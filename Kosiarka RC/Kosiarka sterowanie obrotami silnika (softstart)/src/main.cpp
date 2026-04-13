#include <Arduino.h>
#include <esp_system.h>
#include <math.h>
#include <Wire.h>

// =========================
// PINY SPI - WYŚWIETLACZ TFT
// =========================
#include <TFT_eSPI.h>

// =========================
// USTAWIENIA - ESP32 WROOM
// =========================

// Opcjonalne wyjscie PWM (w tym wariancie sterowanie odbywa sie przez X9C)
static const int PWM_PIN = 33;
static const int PWM_CHANNEL = 0;
static const int PWM_RESOLUTION = 10;
static const uint32_t FREQ_MIN = 0;
static const uint32_t FREQ_MAX = 15000;
static const bool ENABLE_PWM_OUTPUT = false;

// X9C103S (cyfrowy potencjometr 100K) - dostosowane dla ESP32 WROOM
static const int X9C_CS_PIN = 14;
static const int X9C_INC_PIN = 26;
static const int X9C_UD_PIN = 25;
static const int X9C_MAX_STEPS = 99;
// Twardy limit wyjscia X9C, zeby nie przekroczyc bezpiecznego napiecia na sterowaniu silnika.
static const int X9C_OUTPUT_MAX_STEP = 85;
// Skala tylko do wyswietlania procentow na TFT. Ustawiona tak, aby realne maksimum pracy pokazywalo 100%.
static const int X9C_DISPLAY_FULL_SCALE_STEP = 27;
static const int RC_INPUT_PIN = 27;
static const int RC_RELAY_INPUT_PIN = 4;
static const int RELAY_OUTPUT_PIN = 13;
static const bool RELAY_ACTIVE_HIGH = true;
static const uint16_t RC_PULSE_VALID_MIN_US = 900;
static const uint16_t RC_PULSE_VALID_MAX_US = 2100;
static const uint16_t RC_PULSE_MIN_US = 1000;
static const uint16_t RC_PULSE_MAX_US = 2000;
static const uint16_t RC_PULSE_ZERO_THRESHOLD_US = 1050;
static const uint16_t RC_SWITCH_ON_THRESHOLD_US = 1600;
static const uint16_t RC_SWITCH_OFF_THRESHOLD_US = 1400;
// Zakres RC, na ktory rozciagamy caly bezpieczny zakres potencjometru X9C.
static const uint16_t RC_PULSE_X9C_ACTIVE_MIN_US = 1000;
static const uint16_t RC_PULSE_X9C_ACTIVE_MAX_US = 4000;
static const uint32_t RC_PULSE_READ_TIMEOUT_US = 30000;
static const uint8_t RC_FILTER_SMOOTHING_SHIFT = 2;  // 1/4 nowej probki
// Ogranicza szybkosc zmian X9C; wieksza wartosc = szybsza reakcja silnika.
static const uint8_t X9C_MAX_STEP_CHANGE_PER_UPDATE = 1;
static const uint16_t X9C_STEP_UPDATE_INTERVAL_MS = 150;
static const int GYRO_MODE_RC_PIN = 34;
static const uint32_t GYRO_MODE_PULSE_READ_TIMEOUT_US = 30000;
static const uint16_t GYRO_MODE_ON_THRESHOLD_US = 1600;
static const uint16_t GYRO_MODE_OFF_THRESHOLD_US = 1400;
static const uint16_t GYRO_MODE_READ_INTERVAL_MS = 100;

// BMI160 - zyroskop do prostego trybu "heading hold"
static const int BMI160_SDA_PIN = 21;
static const int BMI160_SCL_PIN = 32;
static const uint32_t BMI160_I2C_CLOCK_HZ = 400000;
static const uint8_t BMI160_I2C_ADDR_PRIMARY = 0x69;
static const uint8_t BMI160_I2C_ADDR_SECONDARY = 0x68;
static const uint16_t BMI160_CALIBRATION_SAMPLES = 400;
static const float BMI160_GYRO_Z_LSB_PER_DPS = 131.2f;
static const float BMI160_GYRO_Z_DEADBAND_DPS = 0.15f;
static const float HEADING_HOLD_P_GAIN = 2.5f;
static const float HEADING_HOLD_MAX_CORRECTION = 60.0f;
static const float HEADING_DISPLAY_JITTER_DEG = 0.05f;

// UART do hoverboarda STM32 (ESP wysyla tylko korekte zyroskopowa)
static const int HOVER_UART_TX_PIN = 2;
static const uint32_t HOVER_UART_BAUD = 115200;
static const uint16_t HOVER_UART_START_FRAME = 0xABCD;
static const uint16_t HOVER_UART_SEND_INTERVAL_MS = 20;
static const float HOVER_UART_CORRECTION_GAIN = 4.0f;
static const int16_t HOVER_UART_CORRECTION_MAX = 250;

// Temperatura
static const uint8_t TEMP_SENSOR_COUNT = 3;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 500;
static const uint16_t TFT_UPDATE_INTERVAL_MS = 250;
static const uint16_t TEMP_SAMPLE_INTERVAL_MS = 500;
static const float TEMP_DISPLAY_CHANGE_THRESHOLD_C = 0.5f;
static const uint8_t NTC_SAMPLE_COUNT = 8;
static const float NTC_R25 = 10000.0f;               // NTC 10k przy 25 C
static const float NTC_BETA = 3950.0f;               // Zmien, jesli Twoj NTC ma inny wspolczynnik B
static const float NTC_SERIES_RESISTOR = 10000.0f;   // Rezystor dzielnika 10k
static const float ADC_REFERENCE_VOLTAGE = 3.3f;
static const float TEMP_CALIBRATION_OFFSET_C = 2.0f;
static const float NTC_ADC_CLAMP_MIN = 1.0f;
static const float NTC_ADC_CLAMP_MAX = 3299.0f;
static const bool NTC_TO_GND = false;                // false: NTC do 3.3V, rezystor 10k do GND
static const bool DIAGNOSTIC_DISPLAY_ONLY = false;
static const int TFT_BL_PIN = 22;
static const bool TFT_BL_INVERT = false;

// TFT Display
TFT_eSPI tft = TFT_eSPI();
HardwareSerial HoverGyroSerial(1);
static const uint32_t TFT_BG_COLOR = TFT_BLACK;
static const uint32_t TFT_TEXT_COLOR = TFT_WHITE;
static const uint32_t TFT_ALERT_COLOR = TFT_RED;
static const uint32_t TFT_OK_COLOR = TFT_GREEN;

// Rejestry BMI160 wykorzystywane w prostym odczycie po I2C
static const uint8_t BMI160_REG_CHIP_ID = 0x00;
static const uint8_t BMI160_REG_GYRO_DATA = 0x0C;
static const uint8_t BMI160_REG_COMMAND = 0x7E;
static const uint8_t BMI160_REG_GYRO_RANGE = 0x43;
static const uint8_t BMI160_CHIP_ID = 0xD1;
static const uint8_t BMI160_CMD_SOFT_RESET = 0xB6;
static const uint8_t BMI160_CMD_ACCEL_NORMAL = 0x11;
static const uint8_t BMI160_CMD_GYRO_NORMAL = 0x15;
static const uint8_t BMI160_GYRO_RANGE_250_DPS = 0x03;

struct GyroHoldState {
    bool sensorFound = false;
    bool holdEnabled = false;
    bool switchSignalPresent = false;
    uint8_t i2cAddress = 0;
    uint16_t switchPulseUs = 0;
    float gyroZBiasDps = 0.0f;
    float headingDeg = 0.0f;
    float targetHeadingDeg = 0.0f;
    float headingErrorDeg = 0.0f;
    float correction = 0.0f;
    uint32_t lastUpdateUs = 0;
    uint32_t lastSwitchReadMs = 0;
};

struct HoverSerialCommand {
    uint16_t start;
    int16_t steer;
    int16_t speed;
    uint16_t checksum;
};

struct RcPulseCapture {
    int pin;
    volatile uint32_t riseUs;
    volatile uint32_t pulseUs;
    volatile uint32_t lastEdgeUs;
};

struct TempSensorConfig {
    const char* label;
    int pin;
    float shutdownThresholdC;
    float hysteresisC;
};

static const TempSensorConfig TEMP_SENSORS[TEMP_SENSOR_COUNT] = {
    {"T1_E", 35, 85.0f, 5.0f},
    // GPIO33 i GPIO36 sa stabilniejszymi wejsciami ADC niz piny strapujace GPIO2/GPIO0.
    {"T2_SE", 33, 55.0f, 5.0f},
    {"T2S", 36, 55.0f, 5.0f},
};

// Zmienne stanu
uint32_t currentFreq = 0;
uint8_t currentDutyPercent = 50;
uint32_t lastStatusPrint = 0;
uint32_t lastTftUpdate = 0;

// Zmienne do sweepowania czestotliwosci
bool isSweeping = false;
uint32_t sweepStartTime = 0;
static const uint32_t SWEEP_DURATION_MS = 3000;
bool sweepDirection = true;

// Zmienne X9C103S
uint8_t x9cCurrentStep = 0;
bool startupUnlocked = false;
bool startupMessageShown = false;

// Zmienne temperatury
bool thermalShutdownActive = false;
uint16_t lastTempAdcRaw[TEMP_SENSOR_COUNT] = {0};
bool relayEnabled = false;
int thermalShutdownSensorIndex = -1;
GyroHoldState gyroHold;
RcPulseCapture rcInputCapture = {RC_INPUT_PIN, 0, 0, 0};
RcPulseCapture gyroModeCapture = {GYRO_MODE_RC_PIN, 0, 0, 0};
RcPulseCapture relayInputCapture = {RC_RELAY_INPUT_PIN, 0, 0, 0};

void printBootStage(const char* message) {
    Serial.print("[BOOT] ");
    Serial.println(message);
}

void initBacklight() {
    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, TFT_BL_INVERT ? LOW : HIGH);
}

String formatFrequencyLabel(uint32_t freqHz) {
    if (freqHz <= 999) {
        return String(freqHz) + "Hz";
    }

    float freqKHz = freqHz / 1000.0f;
    return String(freqKHz, 1) + "kHz";
}

bool isTemperatureValid(float tempC) {
    return !isnan(tempC) && tempC > -100.0f && tempC < 200.0f;
}

String formatTemperatureLabel(float tempC) {
    if (!isTemperatureValid(tempC)) {
        return "--.-C";
    }

    return String(tempC, 1) + "C";
}

// =========================
// FUNKCJE - Obsługa wyświetlacza
// =========================
void tftInit() {
    tft.init();
    tft.setRotation(1);  // 0=portrait, 1=landscape, 2=reverse portrait, 3=reverse landscape
    tft.fillScreen(TFT_BG_COLOR);
    tft.setTextColor(TFT_TEXT_COLOR, TFT_BG_COLOR);
    tft.setTextSize(1);
}

void tftClearScreen() {
    tft.fillScreen(TFT_BG_COLOR);
}

void tftShowDiagnosticScreen(const float temps[], uint32_t color, const char* colorName) {
    tft.fillScreen(color);
    tft.setTextSize(2);
    tft.setTextColor(TFT_TEXT_COLOR, color);
    tft.setCursor(8, 8);
    tft.println("TEST TFT");

    tft.setTextSize(1);
    tft.setCursor(8, 36);
    tft.print("Kolor: ");
    tft.println(colorName);

    for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
        tft.setCursor(8, 52 + (i * 16));
        tft.print(TEMP_SENSORS[i].label);
        tft.print(": ");
        tft.print(isTemperatureValid(temps[i]) ? String(temps[i], 1) : String("--.-"));
        tft.print("C ");
        tft.print(lastTempAdcRaw[i]);
    }

    tft.setCursor(8, 104);
    tft.println("PWM/X9C wylaczone");

    tft.setCursor(8, 120);
    tft.println("GPIO TFT: CS5 DC16 RST17");

    tft.setCursor(8, 136);
    tft.println("Jesli nic nie widac:");

    tft.setCursor(8, 152);
    tft.println("1. zly driver TFT");

    tft.setCursor(8, 168);
    tft.println("2. brak podswietlenia");

    tft.setCursor(8, 184);
    tft.println("3. zle polaczenie");
}

void tftPrintStatus(uint16_t inputPulseUs, uint8_t targetStep, uint8_t outputStep, const float temps[], float maxTemp, uint16_t relayPulseUs, bool relayState, uint32_t pwmFreq, bool displayStartupUnlocked, bool displaySweeping) {
    static bool layoutDrawn = false;
    static int lastInputPct = -1;
    static int lastInputPulseUs = -1;
    static int lastOutputPct = -1;
    static int lastOutputStep = -1;
    static int lastTempDeciC[TEMP_SENSOR_COUNT] = {-10000, -10000, -10000};
    static int lastMaxTempDeciC = -10000;
    static bool lastThermalShutdown = false;
    static uint32_t lastPwmFreq = 0xFFFFFFFF;
    static int lastStartupUnlocked = -1;
    static int lastSweeping = -1;
    static int lastStartState = -1;
    static int lastRelayPulseUs = -1;
    static int lastRelayState = -1;
    static int lastGyroState = -1;
    static int lastGyroSwitchState = -1;
    static int lastHeadingDeciDeg = 100000;
    static int lastCorrectionDeci = 100000;
    const uint16_t panel = tft.color565(20, 24, 34);
    const uint16_t panelSoft = tft.color565(38, 44, 58);
    const uint16_t accentIn = tft.color565(0, 200, 175);
    const uint16_t accentOut = tft.color565(255, 170, 70);
    const uint16_t accentTemp = thermalShutdownActive ? TFT_RED : tft.color565(90, 220, 120);
    const uint16_t textDim = tft.color565(165, 175, 188);
    const uint16_t startWaitColor = tft.color565(255, 165, 0);
    const uint16_t accentGyro = gyroHold.sensorFound ? (gyroHold.holdEnabled ? tft.color565(70, 180, 255) : tft.color565(90, 120, 150)) : tft.color565(120, 70, 70);

    const int inputPct = min(100, (targetStep * 100) / X9C_DISPLAY_FULL_SCALE_STEP);
    const int outputPct = min(100, (outputStep * 100) / X9C_DISPLAY_FULL_SCALE_STEP);
    const int barIn = (112 * inputPct) / 100;
    const int barOut = (112 * outputPct) / 100;
    int tempDeciC[TEMP_SENSOR_COUNT];
    bool tempChanged = false;
    for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
        tempDeciC[i] = (int)roundf(temps[i] * 10.0f);
        if (tempDeciC[i] != lastTempDeciC[i]) {
            tempChanged = true;
        }
    }
    const int maxTempDeciC = isTemperatureValid(maxTemp) ? (int)roundf(maxTemp * 10.0f) : -10000;
    const int startState = thermalShutdownActive ? 0 : ((displayStartupUnlocked && relayState) ? 2 : 1);
    const char* startLabel = (startState == 0) ? "OFF" : ((startState == 1) ? "WAIT" : "GO");
    const uint16_t startColor = (startState == 0) ? TFT_RED : ((startState == 1) ? startWaitColor : TFT_GREEN);
    const int gyroState = gyroHold.sensorFound ? (gyroHold.holdEnabled ? 2 : 1) : 0;
    const int gyroSwitchState = gyroHold.switchSignalPresent ? 1 : 0;
    const int headingDeciDeg = (int)roundf(gyroHold.headingDeg * 10.0f);
    const int correctionDeci = (int)roundf(gyroHold.correction * 10.0f);

    if (!layoutDrawn) {
        tft.fillScreen(TFT_BG_COLOR);

        tft.fillRoundRect(8, 8, 264, 24, 8, panel);
        tft.drawRoundRect(8, 8, 264, 24, 8, panelSoft);
        tft.setTextColor(TFT_TEXT_COLOR, panel);
        tft.drawString("STEROWANIE SILNIKA", 16, 13, 2);

        tft.fillRoundRect(8, 40, 128, 72, 12, panel);
        tft.drawRoundRect(8, 40, 128, 72, 12, accentIn);
        tft.setTextColor(textDim, panel);
        tft.drawString("WEJSCIE", 18, 50, 2);

        tft.fillRoundRect(144, 40, 128, 72, 12, panel);
        tft.drawRoundRect(144, 40, 128, 72, 12, accentOut);
        tft.setTextColor(textDim, panel);
        tft.drawString("WYJSCIE", 154, 50, 2);

        tft.fillRoundRect(8, 240, 264, 28, 10, panel);
        tft.drawRoundRect(8, 240, 264, 28, 10, accentGyro);

        layoutDrawn = true;
    }

    if (inputPct != lastInputPct || inputPulseUs != lastInputPulseUs) {
        tft.fillRect(18, 68, 94, 22, panel);
        tft.setTextColor(TFT_TEXT_COLOR, panel);
        tft.drawString(String(inputPct), 18, 72, 2);
        tft.drawString("%", 54, 72, 2);
        tft.fillRect(18, 92, 112, 16, panel);
        tft.setTextColor(textDim, panel);
        if (inputPulseUs == 0) {
            tft.drawString("BRAK RC", 18, 93, 2);
        } else {
            tft.drawString("RC " + String(inputPulseUs) + "us", 18, 93, 2);
        }
        tft.fillRoundRect(18, 106, 112, 4, 2, panelSoft);
        if (barIn > 0) {
            tft.fillRoundRect(18, 106, barIn, 4, 2, accentIn);
        }
        lastInputPct = inputPct;
        lastInputPulseUs = inputPulseUs;
    }

    if (outputPct != lastOutputPct || outputStep != lastOutputStep) {
        tft.fillRect(154, 68, 94, 22, panel);
        tft.setTextColor(TFT_TEXT_COLOR, panel);
        tft.drawString(String(outputPct), 154, 72, 2);
        tft.drawString("%", 190, 72, 2);
        tft.fillRect(154, 92, 112, 16, panel);
        tft.setTextColor(textDim, panel);
        tft.drawString("ST " + String(outputStep), 154, 93, 2);
        tft.fillRoundRect(154, 106, 112, 4, 2, panelSoft);
        if (barOut > 0) {
            tft.fillRoundRect(154, 106, barOut, 4, 2, accentOut);
        }
        lastOutputPct = outputPct;
        lastOutputStep = outputStep;
    }

    if (tempChanged || maxTempDeciC != lastMaxTempDeciC || thermalShutdownActive != lastThermalShutdown || relayPulseUs != lastRelayPulseUs || (int)relayState != lastRelayState) {
        tft.fillRoundRect(8, 120, 264, 84, 12, panel);
        tft.drawRoundRect(8, 120, 264, 84, 12, accentTemp);
        tft.setTextColor(textDim, panel);
        tft.drawString("TEMPERATURY", 18, 126, 2);
        tft.setTextColor(accentTemp, panel);
        tft.drawString(String(TEMP_SENSORS[0].label) + " " + formatTemperatureLabel(temps[0]), 18, 146, 2);
        tft.drawString(String(TEMP_SENSORS[1].label) + " " + formatTemperatureLabel(temps[1]), 18, 164, 2);
        tft.drawString(String(TEMP_SENSORS[2].label) + " " + formatTemperatureLabel(temps[2]), 18, 182, 2);
        tft.setTextColor(TFT_TEXT_COLOR, panel);
        tft.drawString("MAX " + formatTemperatureLabel(maxTemp), 150, 146, 2);
        tft.drawString(relayState ? "RELAY ON" : "RELAY OFF", 150, 164, 2);
        if (relayPulseUs == 0) {
            tft.drawString("SW BRAK", 150, 182, 2);
        } else {
            tft.drawString("SW " + String(relayPulseUs) + "us", 150, 182, 2);
        }
        tft.setTextColor(thermalShutdownActive ? TFT_ALERT_COLOR : TFT_OK_COLOR, panel);
        if (thermalShutdownActive && thermalShutdownSensorIndex >= 0) {
            tft.drawString(String("HOT ") + TEMP_SENSORS[thermalShutdownSensorIndex].label, 186, 126, 2);
        } else {
            tft.drawString("OK", 226, 126, 2);
        }
        for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
            lastTempDeciC[i] = tempDeciC[i];
        }
        lastMaxTempDeciC = maxTempDeciC;
        lastThermalShutdown = thermalShutdownActive;
        lastRelayPulseUs = relayPulseUs;
        lastRelayState = relayState ? 1 : 0;
    }

    if (pwmFreq != lastPwmFreq || (int)displayStartupUnlocked != lastStartupUnlocked || (int)displaySweeping != lastSweeping || startState != lastStartState) {
        tft.fillRoundRect(8, 212, 264, 22, 8, panel);
        tft.drawRoundRect(8, 212, 264, 22, 8, panelSoft);
        tft.setTextColor(textDim, panel);
        if (ENABLE_PWM_OUTPUT) {
            tft.drawString("PWM " + formatFrequencyLabel(pwmFreq), 14, 217, 2);
            tft.drawString("START", 100, 217, 2);
            tft.setTextColor(startColor, panel);
            tft.drawString(startLabel, 142, 217, 2);
        } else {
            tft.drawString("X9C RC", 14, 217, 2);
            tft.drawString("START", 100, 217, 2);
            tft.setTextColor(startColor, panel);
            tft.drawString(startLabel, 142, 217, 2);
        }
        tft.setTextColor(textDim, panel);
        tft.drawString(ENABLE_PWM_OUTPUT ? (displaySweeping ? "SWEEP" : "MANUAL") : "DIRECT", 206, 217, 2);
        lastPwmFreq = pwmFreq;
        lastStartupUnlocked = displayStartupUnlocked ? 1 : 0;
        lastSweeping = displaySweeping ? 1 : 0;
        lastStartState = startState;
    }

    if (gyroState != lastGyroState || gyroSwitchState != lastGyroSwitchState || headingDeciDeg != lastHeadingDeciDeg || correctionDeci != lastCorrectionDeci) {
        const char* gyroLabel = gyroState == 2 ? "GYRO HOLD" : (gyroState == 1 ? "GYRO READY" : "GYRO OFF");
        const uint16_t gyroColor = gyroState == 2 ? tft.color565(70, 180, 255) : (gyroState == 1 ? textDim : TFT_ALERT_COLOR);

        tft.fillRoundRect(8, 240, 264, 28, 10, panel);
        tft.drawRoundRect(8, 240, 264, 28, 10, accentGyro);
        tft.setTextColor(gyroColor, panel);
        tft.drawString(gyroLabel, 14, 247, 2);
        tft.setTextColor(textDim, panel);
        tft.drawString("HDG " + String(gyroHold.headingDeg, 1), 104, 247, 2);
        tft.drawString("COR " + String(gyroHold.correction, 1), 182, 247, 2);
        if (!gyroHold.switchSignalPresent) {
            tft.drawString("SW?", 235, 247, 2);
        }

        lastGyroState = gyroState;
        lastGyroSwitchState = gyroSwitchState;
        lastHeadingDeciDeg = headingDeciDeg;
        lastCorrectionDeci = correctionDeci;
    }
}

// =========================
// FUNKCJE
// =========================
void x9cSetStep(uint8_t targetStep);
void printStatus(uint16_t inputPulseUs, uint8_t targetStep, const float temps[], float maxTemp, uint16_t relayPulseUs, bool relayState);
uint16_t readGyroModePulseUs();
void updateGyroModeSwitch();
bool bmi160Begin();
bool calibrateBmi160Gyro();
bool readBmi160GyroZ(float& gyroZDps);
void updateGyroHeadingHold();
float normalizeAngleDeg(float angleDeg);
void resetGyroHoldTarget();
void initHoverGyroUart();
void sendHoverGyroCorrection();
void initRcPulseCapture();
void processUsbSerialCommands();
void processUsbSerialCommand(const char* commandLine);
uint16_t readPulseUs(volatile RcPulseCapture& capture, uint32_t timeoutUs);

uint32_t dutyPercentToRaw(uint8_t percent, int resolutionBits) {
    uint32_t maxDuty = (1UL << resolutionBits) - 1;
    return (maxDuty * percent) / 100;
}

void stopPwm() {
    if (ENABLE_PWM_OUTPUT) {
        ledcWrite(PWM_CHANNEL, 0);
        digitalWrite(PWM_PIN, LOW);
    }
    currentFreq = 0;
}

void resetControlToSafeState() {
    x9cSetStep(0);
    stopPwm();
}

void setRelayState(bool enabled) {
    relayEnabled = enabled;
    digitalWrite(RELAY_OUTPUT_PIN, (enabled == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

// ========== FUNKCJE X9C103S ==========
void x9cInit() {
    pinMode(X9C_CS_PIN, OUTPUT);
    pinMode(X9C_INC_PIN, OUTPUT);
    pinMode(X9C_UD_PIN, OUTPUT);

    digitalWrite(X9C_CS_PIN, HIGH);
    digitalWrite(X9C_INC_PIN, HIGH);
    digitalWrite(X9C_UD_PIN, HIGH);
}

void x9cPulseIncrement() {
    digitalWrite(X9C_CS_PIN, LOW);
    delayMicroseconds(1);

    digitalWrite(X9C_INC_PIN, LOW);
    delayMicroseconds(100);
    digitalWrite(X9C_INC_PIN, HIGH);
    delayMicroseconds(100);

    digitalWrite(X9C_CS_PIN, HIGH);
    delayMicroseconds(1);
}

void x9cForceToZero() {
    digitalWrite(X9C_UD_PIN, LOW);

    // Po starcie nie znamy zapisanej pozycji X9C, wiec zjezdzamy do ogranicznika.
    for (uint8_t i = 0; i <= X9C_MAX_STEPS; i++) {
        x9cPulseIncrement();
    }

    x9cCurrentStep = 0;
    Serial.println("X9C ustawiony na 0");
}

void x9cSetStep(uint8_t targetStep) {
    if (targetStep > X9C_OUTPUT_MAX_STEP) {
        targetStep = X9C_OUTPUT_MAX_STEP;
    }

    if (targetStep == x9cCurrentStep) {
        return;
    }

    if (targetStep > x9cCurrentStep) {
        digitalWrite(X9C_UD_PIN, HIGH);
    } else {
        digitalWrite(X9C_UD_PIN, LOW);
    }

    uint8_t steps = abs(targetStep - x9cCurrentStep);

    for (uint8_t i = 0; i < steps; i++) {
        x9cPulseIncrement();
    }

    x9cCurrentStep = targetStep;
}

bool hasValidReceiverSignal(uint16_t pulseUs) {
    return pulseUs >= RC_PULSE_VALID_MIN_US && pulseUs <= RC_PULSE_VALID_MAX_US;
}

void IRAM_ATTR handleRcInputInterrupt() {
    uint32_t nowUs = micros();
    rcInputCapture.lastEdgeUs = nowUs;
    if (digitalRead(RC_INPUT_PIN)) {
        rcInputCapture.riseUs = nowUs;
    } else if (rcInputCapture.riseUs != 0) {
        rcInputCapture.pulseUs = nowUs - rcInputCapture.riseUs;
    }
}

void IRAM_ATTR handleGyroModeInterrupt() {
    uint32_t nowUs = micros();
    gyroModeCapture.lastEdgeUs = nowUs;
    if (digitalRead(GYRO_MODE_RC_PIN)) {
        gyroModeCapture.riseUs = nowUs;
    } else if (gyroModeCapture.riseUs != 0) {
        gyroModeCapture.pulseUs = nowUs - gyroModeCapture.riseUs;
    }
}

void IRAM_ATTR handleRelayInputInterrupt() {
    uint32_t nowUs = micros();
    relayInputCapture.lastEdgeUs = nowUs;
    if (digitalRead(RC_RELAY_INPUT_PIN)) {
        relayInputCapture.riseUs = nowUs;
    } else if (relayInputCapture.riseUs != 0) {
        relayInputCapture.pulseUs = nowUs - relayInputCapture.riseUs;
    }
}

void initRcPulseCapture() {
    pinMode(RC_INPUT_PIN, INPUT);
    pinMode(GYRO_MODE_RC_PIN, INPUT);
    pinMode(RC_RELAY_INPUT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RC_INPUT_PIN), handleRcInputInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(GYRO_MODE_RC_PIN), handleGyroModeInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_RELAY_INPUT_PIN), handleRelayInputInterrupt, CHANGE);
}

uint16_t readPulseUs(volatile RcPulseCapture& capture, uint32_t timeoutUs) {
    uint32_t pulseUs;
    uint32_t lastEdgeUs;
    noInterrupts();
    pulseUs = capture.pulseUs;
    lastEdgeUs = capture.lastEdgeUs;
    interrupts();

    if (lastEdgeUs == 0 || (micros() - lastEdgeUs) > timeoutUs) {
        return 0;
    }

    if (!hasValidReceiverSignal((uint16_t)pulseUs)) {
        return 0;
    }

    return (uint16_t)pulseUs;
}

uint16_t readReceiverPulseUs() {
    return readPulseUs(rcInputCapture, RC_PULSE_READ_TIMEOUT_US);
}

uint16_t readGyroModePulseUs() {
    return readPulseUs(gyroModeCapture, GYRO_MODE_PULSE_READ_TIMEOUT_US);
}

uint16_t readRelaySwitchPulseUs() {
    return readPulseUs(relayInputCapture, RC_PULSE_READ_TIMEOUT_US);
}

uint8_t inputPulseToX9cStep(uint16_t pulseUs) {
    if (!hasValidReceiverSignal(pulseUs) || pulseUs <= RC_PULSE_ZERO_THRESHOLD_US) {
        return 0;
    }

    uint16_t clampedPulseUs = pulseUs;
    if (clampedPulseUs < RC_PULSE_X9C_ACTIVE_MIN_US) {
        clampedPulseUs = RC_PULSE_X9C_ACTIVE_MIN_US;
    }
    if (clampedPulseUs > RC_PULSE_X9C_ACTIVE_MAX_US) {
        clampedPulseUs = RC_PULSE_X9C_ACTIVE_MAX_US;
    }

    return (uint8_t)((uint32_t)(clampedPulseUs - RC_PULSE_X9C_ACTIVE_MIN_US) * X9C_OUTPUT_MAX_STEP / (RC_PULSE_X9C_ACTIVE_MAX_US - RC_PULSE_X9C_ACTIVE_MIN_US));
}

uint16_t smoothReceiverPulseUs(uint16_t previousPulseUs, uint16_t newPulseUs) {
    if (!hasValidReceiverSignal(newPulseUs)) {
        return 0;
    }

    if (!hasValidReceiverSignal(previousPulseUs)) {
        return newPulseUs;
    }

    return (uint16_t)(((uint32_t)previousPulseUs * ((1U << RC_FILTER_SMOOTHING_SHIFT) - 1U) + newPulseUs) >> RC_FILTER_SMOOTHING_SHIFT);
}

uint8_t limitX9cStepChange(uint8_t currentStep, uint8_t targetStep) {
    if (targetStep > currentStep + X9C_MAX_STEP_CHANGE_PER_UPDATE) {
        return currentStep + X9C_MAX_STEP_CHANGE_PER_UPDATE;
    }

    if (currentStep > targetStep + X9C_MAX_STEP_CHANGE_PER_UPDATE) {
        return currentStep - X9C_MAX_STEP_CHANGE_PER_UPDATE;
    }

    return targetStep;
}

uint32_t inputPulseToFreq(uint16_t pulseUs) {
    if (!hasValidReceiverSignal(pulseUs) || pulseUs <= RC_PULSE_ZERO_THRESHOLD_US) {
        return 0;
    }

    uint16_t clampedPulseUs = pulseUs;
    if (clampedPulseUs > RC_PULSE_MAX_US) {
        clampedPulseUs = RC_PULSE_MAX_US;
    }

    return (uint32_t)((uint64_t)(clampedPulseUs - RC_PULSE_ZERO_THRESHOLD_US) * FREQ_MAX / (RC_PULSE_MAX_US - RC_PULSE_ZERO_THRESHOLD_US));
}

bool isInputAtZero(uint16_t pulseUs) {
    return hasValidReceiverSignal(pulseUs) && pulseUs <= RC_PULSE_ZERO_THRESHOLD_US;
}

float readTemperatureSensor(uint8_t sensorIndex) {
    uint32_t adcRawSum = 0;
    uint32_t voltageMvSum = 0;
    for (uint8_t i = 0; i < NTC_SAMPLE_COUNT; i++) {
        adcRawSum += analogRead(TEMP_SENSORS[sensorIndex].pin);
        voltageMvSum += analogReadMilliVolts(TEMP_SENSORS[sensorIndex].pin);
    }

    float adcRaw = (float)adcRawSum / NTC_SAMPLE_COUNT;
    lastTempAdcRaw[sensorIndex] = (uint16_t)adcRaw;

    float voltageMv = (float)voltageMvSum / NTC_SAMPLE_COUNT;
    // Liczymy temperature z faktycznego napiecia na dzielniku.
    // Dla NTC 10k + rezystor 10k oczekujemy okolo 1.65 V przy 25 C.
    voltageMv = constrain(voltageMv, NTC_ADC_CLAMP_MIN, NTC_ADC_CLAMP_MAX);
    float voltage = voltageMv / 1000.0f;

    float resistance;
    if (NTC_TO_GND) {
        resistance = NTC_SERIES_RESISTOR * (voltage / (ADC_REFERENCE_VOLTAGE - voltage));
    } else {
        resistance = NTC_SERIES_RESISTOR * ((ADC_REFERENCE_VOLTAGE - voltage) / voltage);
    }

    if (!(resistance > 0.0f)) {
        return NAN;
    }

    float tempK = 1.0f / (1.0f / 298.15f + (1.0f / NTC_BETA) * log(resistance / NTC_R25));
    return (tempK - 273.15f) + TEMP_CALIBRATION_OFFSET_C;
}

void readAllTemperatures(float temps[]) {
    for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
        temps[i] = readTemperatureSensor(i);
    }
}

float getMaxTemperature(const float temps[]) {
    float maxTemp = NAN;
    for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
        if (!isTemperatureValid(temps[i])) {
            continue;
        }

        if (!isTemperatureValid(maxTemp) || temps[i] > maxTemp) {
            maxTemp = temps[i];
        }
    }
    return maxTemp;
}

void updateThermalProtection(const float temps[]) {
    if (!thermalShutdownActive) {
        for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
            if (!isTemperatureValid(temps[i])) {
                continue;
            }

            if (temps[i] >= TEMP_SENSORS[i].shutdownThresholdC) {
                thermalShutdownActive = true;
                thermalShutdownSensorIndex = i;
                startupUnlocked = false;
                startupMessageShown = false;
                resetControlToSafeState();
                setRelayState(false);

                Serial.print("Przegrzanie! ");
                Serial.print(TEMP_SENSORS[i].label);
                Serial.print("=");
                Serial.print(temps[i], 1);
                Serial.print(" C. Limit=");
                Serial.print(TEMP_SENSORS[i].shutdownThresholdC, 1);
                Serial.println(" C. Wyjscie ustawione na 0.");
                Serial.println("Po schlodzeniu ustaw potencjometr na 0, aby wznowic prace.");
                return;
            }
        }
    }

    if (thermalShutdownActive && thermalShutdownSensorIndex >= 0) {
        const TempSensorConfig& shutdownSensor = TEMP_SENSORS[thermalShutdownSensorIndex];
        const float releaseThreshold = shutdownSensor.shutdownThresholdC - shutdownSensor.hysteresisC;
        if (isTemperatureValid(temps[thermalShutdownSensorIndex]) && temps[thermalShutdownSensorIndex] <= releaseThreshold) {
            thermalShutdownActive = false;
            Serial.print("Temperatura ");
            Serial.print(shutdownSensor.label);
            Serial.print(" spadla do ");
            Serial.print(temps[thermalShutdownSensorIndex], 1);
            Serial.print(" C. Zabezpieczenie termiczne zwolnione ponizej ");
            Serial.print(releaseThreshold, 1);
            Serial.println(" C.");
            thermalShutdownSensorIndex = -1;
        }
    }

    if (thermalShutdownActive) {
        resetControlToSafeState();
        setRelayState(false);
    }
}

float normalizeAngleDeg(float angleDeg) {
    while (angleDeg > 180.0f) {
        angleDeg -= 360.0f;
    }
    while (angleDeg < -180.0f) {
        angleDeg += 360.0f;
    }
    return angleDeg;
}

bool bmi160WriteRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(gyroHold.i2cAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool bmi160ReadRegisters(uint8_t reg, uint8_t* data, size_t len) {
    Wire.beginTransmission(gyroHold.i2cAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    size_t received = Wire.requestFrom((int)gyroHold.i2cAddress, (int)len);
    if (received != len) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }

    return true;
}

bool detectBmi160AtAddress(uint8_t address) {
    uint8_t chipId = 0;

    gyroHold.i2cAddress = address;
    if (!bmi160ReadRegisters(BMI160_REG_CHIP_ID, &chipId, 1)) {
        return false;
    }

    return chipId == BMI160_CHIP_ID;
}

bool bmi160Begin() {
    Wire.begin(BMI160_SDA_PIN, BMI160_SCL_PIN, BMI160_I2C_CLOCK_HZ);

    if (detectBmi160AtAddress(BMI160_I2C_ADDR_PRIMARY)) {
        gyroHold.i2cAddress = BMI160_I2C_ADDR_PRIMARY;
    } else if (detectBmi160AtAddress(BMI160_I2C_ADDR_SECONDARY)) {
        gyroHold.i2cAddress = BMI160_I2C_ADDR_SECONDARY;
    } else {
        gyroHold.sensorFound = false;
        return false;
    }

    if (!bmi160WriteRegister(BMI160_REG_COMMAND, BMI160_CMD_SOFT_RESET)) {
        return false;
    }
    delay(20);

    if (!bmi160WriteRegister(BMI160_REG_COMMAND, BMI160_CMD_ACCEL_NORMAL)) {
        return false;
    }
    delay(50);

    if (!bmi160WriteRegister(BMI160_REG_COMMAND, BMI160_CMD_GYRO_NORMAL)) {
        return false;
    }
    delay(100);

    if (!bmi160WriteRegister(BMI160_REG_GYRO_RANGE, BMI160_GYRO_RANGE_250_DPS)) {
        return false;
    }

    gyroHold.sensorFound = true;
    gyroHold.holdEnabled = false;
    gyroHold.headingDeg = 0.0f;
    gyroHold.targetHeadingDeg = 0.0f;
    gyroHold.headingErrorDeg = 0.0f;
    gyroHold.correction = 0.0f;
    gyroHold.lastUpdateUs = micros();
    return true;
}

bool readBmi160GyroZ(float& gyroZDps) {
    uint8_t raw[6] = {0};
    if (!gyroHold.sensorFound || !bmi160ReadRegisters(BMI160_REG_GYRO_DATA, raw, sizeof(raw))) {
        return false;
    }

    int16_t gyroZRaw = (int16_t)((raw[5] << 8) | raw[4]);
    gyroZDps = (float)gyroZRaw / BMI160_GYRO_Z_LSB_PER_DPS;
    return true;
}

bool calibrateBmi160Gyro() {
    if (!gyroHold.sensorFound) {
        return false;
    }

    float biasSum = 0.0f;
    uint16_t validSamples = 0;

    for (uint16_t i = 0; i < BMI160_CALIBRATION_SAMPLES; i++) {
        float gyroZDps = 0.0f;
        if (readBmi160GyroZ(gyroZDps)) {
            biasSum += gyroZDps;
            validSamples++;
        }
        delay(2);
    }

    if (validSamples == 0) {
        return false;
    }

    gyroHold.gyroZBiasDps = biasSum / validSamples;
    gyroHold.headingDeg = 0.0f;
    gyroHold.targetHeadingDeg = 0.0f;
    gyroHold.headingErrorDeg = 0.0f;
    gyroHold.correction = 0.0f;
    gyroHold.lastUpdateUs = micros();
    return true;
}

void resetGyroHoldTarget() {
    gyroHold.targetHeadingDeg = gyroHold.headingDeg;
    gyroHold.headingErrorDeg = 0.0f;
    gyroHold.correction = 0.0f;
}

void initHoverGyroUart() {
    HoverGyroSerial.begin(HOVER_UART_BAUD, SERIAL_8N1, -1, HOVER_UART_TX_PIN);
}

void sendHoverGyroCorrection() {
    static uint32_t lastSendMs = 0;
    uint32_t now = millis();
    if (now - lastSendMs < HOVER_UART_SEND_INTERVAL_MS) {
        return;
    }
    lastSendMs = now;

    HoverSerialCommand command = {};
    command.start = HOVER_UART_START_FRAME;

    if (gyroHold.sensorFound && gyroHold.holdEnabled) {
        const float scaledCorrection = gyroHold.correction * HOVER_UART_CORRECTION_GAIN;
        command.steer = (int16_t)constrain((int)lroundf(scaledCorrection), (int)-HOVER_UART_CORRECTION_MAX, (int)HOVER_UART_CORRECTION_MAX);
        command.speed = 1;  // speed > 0 aktywuje korekte po stronie STM32
    } else {
        command.steer = 0;
        command.speed = 0;
    }

    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);
    HoverGyroSerial.write((const uint8_t *)&command, sizeof(command));
}

void updateGyroModeSwitch() {
    uint32_t now = millis();
    if (now - gyroHold.lastSwitchReadMs < GYRO_MODE_READ_INTERVAL_MS) {
        return;
    }

    gyroHold.lastSwitchReadMs = now;
    gyroHold.switchPulseUs = readGyroModePulseUs();
    gyroHold.switchSignalPresent = hasValidReceiverSignal(gyroHold.switchPulseUs);

    if (!gyroHold.switchSignalPresent) {
        gyroHold.holdEnabled = false;
        gyroHold.headingErrorDeg = 0.0f;
        gyroHold.correction = 0.0f;
        return;
    }

    if (gyroHold.switchPulseUs >= GYRO_MODE_ON_THRESHOLD_US) {
        if (!gyroHold.holdEnabled) {
            gyroHold.holdEnabled = true;
            resetGyroHoldTarget();
        }
    } else if (gyroHold.switchPulseUs <= GYRO_MODE_OFF_THRESHOLD_US) {
        gyroHold.holdEnabled = false;
        gyroHold.headingErrorDeg = 0.0f;
        gyroHold.correction = 0.0f;
    }
}

void updateGyroHeadingHold() {
    if (!gyroHold.sensorFound) {
        gyroHold.correction = 0.0f;
        return;
    }

    float gyroZDps = 0.0f;
    if (!readBmi160GyroZ(gyroZDps)) {
        return;
    }

    uint32_t nowUs = micros();
    float dt = (nowUs - gyroHold.lastUpdateUs) / 1000000.0f;
    gyroHold.lastUpdateUs = nowUs;
    if (dt <= 0.0f || dt > 0.25f) {
        return;
    }

    float correctedGyroZDps = gyroZDps - gyroHold.gyroZBiasDps;
    if (fabsf(correctedGyroZDps) < BMI160_GYRO_Z_DEADBAND_DPS) {
        correctedGyroZDps = 0.0f;
    }

    gyroHold.headingDeg = normalizeAngleDeg(gyroHold.headingDeg + correctedGyroZDps * dt);
    if (fabsf(gyroHold.headingDeg) < HEADING_DISPLAY_JITTER_DEG) {
        gyroHold.headingDeg = 0.0f;
    }

    if (!gyroHold.holdEnabled) {
        gyroHold.headingErrorDeg = 0.0f;
        gyroHold.correction = 0.0f;
        return;
    }

    gyroHold.headingErrorDeg = normalizeAngleDeg(gyroHold.targetHeadingDeg - gyroHold.headingDeg);
    gyroHold.correction = constrain(gyroHold.headingErrorDeg * HEADING_HOLD_P_GAIN, -HEADING_HOLD_MAX_CORRECTION, HEADING_HOLD_MAX_CORRECTION);
}

void updateRelayFromRc(uint16_t relayPulseUs) {
    if (thermalShutdownActive || !startupUnlocked) {
        setRelayState(false);
        return;
    }

    if (!hasValidReceiverSignal(relayPulseUs)) {
        setRelayState(false);
        return;
    }

    if (relayPulseUs >= RC_SWITCH_ON_THRESHOLD_US) {
        setRelayState(true);
    } else if (relayPulseUs <= RC_SWITCH_OFF_THRESHOLD_US) {
        setRelayState(false);
    }
}

void printStatus(uint16_t inputPulseUs, uint8_t targetStep, const float temps[], float maxTemp, uint16_t relayPulseUs, bool relayState) {
    Serial.print("IN_RC=");
    if (inputPulseUs == 0) {
        Serial.print("NO_SIG");
    } else {
        Serial.print(inputPulseUs);
        Serial.print("us");
    }
    Serial.print(" IN_STEP=");
    Serial.print(targetStep);
    Serial.print(" OUT_STEP=");
    Serial.print(x9cCurrentStep);
    Serial.print(" T1_E=");
    if (isTemperatureValid(temps[0])) {
        Serial.print(temps[0], 1);
    } else {
        Serial.print("---.-");
    }
    Serial.print("C T2_SE=");
    if (isTemperatureValid(temps[1])) {
        Serial.print(temps[1], 1);
    } else {
        Serial.print("---.-");
    }
    Serial.print("C T2S=");
    if (isTemperatureValid(temps[2])) {
        Serial.print(temps[2], 1);
    } else {
        Serial.print("---.-");
    }
    Serial.print("C TMAX=");
    if (isTemperatureValid(maxTemp)) {
        Serial.print(maxTemp, 1);
    } else {
        Serial.print("---.-");
    }
    Serial.print("C RELAY_SW=");
    if (relayPulseUs == 0) {
        Serial.print("NO_SIG");
    } else {
        Serial.print(relayPulseUs);
        Serial.print("us");
    }
    Serial.print(" RELAY=");
    Serial.print(relayState ? "ON" : "OFF");
    if (ENABLE_PWM_OUTPUT) {
        Serial.print(" PWM=");
        Serial.print(currentFreq);
        Serial.print("Hz START=");
    } else {
        Serial.print(" MODE=X9C START=");
    }
    Serial.print(startupUnlocked ? "ON" : "WAIT");
    Serial.print(" THERM=");
    Serial.print(thermalShutdownActive ? "LOCK" : "OK");
    Serial.print(" GYRO=");
    if (!gyroHold.sensorFound) {
        Serial.print("NO_SENSOR");
    } else if (gyroHold.holdEnabled) {
        Serial.print("HOLD");
    } else {
        Serial.print("READY");
    }
    Serial.print(" SW=");
    if (!gyroHold.switchSignalPresent) {
        Serial.print("NO_SIG");
    } else {
        Serial.print(gyroHold.switchPulseUs);
        Serial.print("us");
    }
    Serial.print(" HDG=");
    Serial.print(gyroHold.headingDeg, 2);
    Serial.print(" ERR=");
    Serial.print(gyroHold.headingErrorDeg, 2);
    Serial.print(" COR=");
    Serial.println(gyroHold.correction, 2);
}

bool startOrUpdatePwm(uint32_t freq, uint8_t dutyPercent) {
    if (!ENABLE_PWM_OUTPUT) {
        stopPwm();
        return true;
    }

    if (freq == 0) {
        stopPwm();
        return true;
    }

    if (freq > FREQ_MAX) {
        return false;
    }

    bool ok = ledcSetup(PWM_CHANNEL, freq, PWM_RESOLUTION);
    if (!ok) {
        return false;
    }

    ledcAttachPin(PWM_PIN, PWM_CHANNEL);

    uint32_t dutyRaw = dutyPercentToRaw(dutyPercent, PWM_RESOLUTION);
    ledcWrite(PWM_CHANNEL, dutyRaw);

    return true;
}

void processUsbSerialCommand(const char* commandLine) {
    String input(commandLine);
    input.trim();

    if (input.length() == 0) {
        return;
    }

    if (input.equalsIgnoreCase("GYRORESET")) {
        resetGyroHoldTarget();
        Serial.println("Zresetowano target heading do aktualnego kata.");
        return;
    }

    if (!ENABLE_PWM_OUTPUT) {
        Serial.println("Tryb X9C: komendy PWM START/STOP/Hz sa wylaczone.");
        return;
    }

    if (input.equalsIgnoreCase("STOP")) {
        isSweeping = false;
        stopPwm();
        Serial.println("Sweep zatrzymany");
        return;
    }

    if (thermalShutdownActive) {
        Serial.println("Blokada termiczna aktywna. Poczekaj az silnik ostygnie.");
        return;
    }

    if (!startupUnlocked) {
        Serial.println("Blokada startu: najpierw ustaw potencjometr na 0.");
        return;
    }

    if (input.equalsIgnoreCase("START")) {
        isSweeping = true;
        sweepDirection = true;
        sweepStartTime = millis();
        Serial.println("Sweep wznowiony (0 -> 15kHz)");
        return;
    }

    long newFreq = input.toInt();

    if (newFreq < (long)FREQ_MIN || newFreq > (long)FREQ_MAX) {
        Serial.println("Zakres 0-15000 Hz");
        Serial.println("Lub wyslij START/STOP");
        return;
    }

    isSweeping = false;
    currentFreq = (uint32_t)newFreq;

    if (!startOrUpdatePwm(currentFreq, currentDutyPercent)) {
        Serial.println("Blad PWM");
    }
}

void processUsbSerialCommands() {
    static char inputBuffer[32];
    static size_t inputLen = 0;

    while (Serial.available() > 0) {
        char ch = (char)Serial.read();

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            inputBuffer[inputLen] = '\0';
            processUsbSerialCommand(inputBuffer);
            inputLen = 0;
            continue;
        }

        if (inputLen < (sizeof(inputBuffer) - 1)) {
            inputBuffer[inputLen++] = ch;
        } else {
            inputLen = 0;
        }
    }
}

void setup() {
    Serial.begin(115200);
    initHoverGyroUart();
    delay(300);
    printBootStage("Start programu");
    Serial.print("[BOOT] Reset reason=");
    Serial.println((int)esp_reset_reason());
    Serial.println("[BOOT] Jesli tu nic nie widac, ESP32 nie startuje poprawnie.");
    printBootStage("Init BL");
    initBacklight();
    analogReadResolution(12);
    for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
        analogSetPinAttenuation(TEMP_SENSORS[i].pin, ADC_11db);
    }
    if (DIAGNOSTIC_DISPLAY_ONLY) {
        printBootStage("Tryb diagnostyczny TFT");
        printBootStage("Init TFT");
        tftInit();
        tftClearScreen();
        printBootStage("TFT gotowy");
        float diagTemps[TEMP_SENSOR_COUNT];
        readAllTemperatures(diagTemps);
        tftShowDiagnosticScreen(diagTemps, TFT_RED, "CZERWONY");
        Serial.println("[BOOT] Tryb diagnostyczny aktywny: PWM/X9C pominiete.");
        printBootStage("Setup zakonczony");
        return;
    }
    
    // Inicjalizacja pinów
    printBootStage("Init PWM");
    if (ENABLE_PWM_OUTPUT) {
        pinMode(PWM_PIN, OUTPUT);
        digitalWrite(PWM_PIN, LOW);
    }
    initRcPulseCapture();
    pinMode(RELAY_OUTPUT_PIN, OUTPUT);
    setRelayState(false);

    // Inicjalizacja X9C103S
    printBootStage("Init X9C");
    x9cInit();
    x9cForceToZero();
    stopPwm();
    printBootStage("Init BMI160");
    if (bmi160Begin()) {
        if (calibrateBmi160Gyro()) {
            Serial.print("BMI160 gotowy na adresie 0x");
            Serial.print(gyroHold.i2cAddress, HEX);
            Serial.print(", bias Z=");
            Serial.print(gyroHold.gyroZBiasDps, 4);
            Serial.println(" dps");
        } else {
            Serial.println("BMI160 wykryty, ale kalibracja nie powiodla sie.");
            gyroHold.sensorFound = false;
        }
    } else {
        Serial.println("BMI160 nie wykryty. Softstart bedzie dzialal bez trybu zyroskopu.");
    }
    
    // Inicjalizacja wyświetlacza TFT
    printBootStage("Init TFT");
    tftInit();
    tftClearScreen();
    printBootStage("TFT gotowy");
    
    Serial.println("Blokada startu aktywna. Ustaw potencjometr na 0.");
    printBootStage("Setup zakonczony");
}

void loop() {
    static bool tempInitialized = false;
    static float currentTemps[TEMP_SENSOR_COUNT] = {0.0f, 0.0f, 0.0f};
    static float displayTemps[TEMP_SENSOR_COUNT] = {0.0f, 0.0f, 0.0f};
    static float currentMaxTemp = 0.0f;
    static float displayMaxTemp = 0.0f;
    static uint32_t lastTempSample = 0;
    static uint16_t displayInputPulseUs = 0;
    static uint8_t displayTargetStep = 0;
    static uint8_t displayOutputStep = 0;
    static uint16_t displayRelayPulseUs = 0;
    static bool displayRelayEnabled = false;
    static uint32_t displayPwmFreq = 0;
    static bool displayStartupUnlocked = false;
    static bool displaySweeping = true;
    static bool signalLossLatched = false;
    static uint16_t filteredInputPulseUs = 0;
    static uint32_t lastX9cStepUpdate = 0;

    if (DIAGNOSTIC_DISPLAY_ONLY) {
        static uint32_t lastDiagUpdate = 0;
        static uint8_t colorIndex = 0;

        if (millis() - lastDiagUpdate >= 1000) {
            float temps[TEMP_SENSOR_COUNT];
            readAllTemperatures(temps);
            uint32_t color = TFT_BLACK;
            const char* colorName = "CZARNY";

            switch (colorIndex) {
                case 0:
                    color = TFT_RED;
                    colorName = "CZERWONY";
                    break;
                case 1:
                    color = TFT_GREEN;
                    colorName = "ZIELONY";
                    break;
                case 2:
                    color = TFT_BLUE;
                    colorName = "NIEBIESKI";
                    break;
                default:
                    color = TFT_BLACK;
                    colorName = "CZARNY";
                    break;
            }

            tftShowDiagnosticScreen(temps, color, colorName);
            Serial.print("[DIAG] TFT kolor=");
            Serial.print(colorName);
            for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
                Serial.print(" ");
                Serial.print(TEMP_SENSORS[i].label);
                Serial.print("=");
                Serial.print(temps[i], 1);
                Serial.print("C");
            }
            Serial.println();

            colorIndex = (colorIndex + 1) % 4;
            lastDiagUpdate = millis();
        }

        delay(20);
        return;
    }

    uint16_t inputPulseUs = readReceiverPulseUs();
    uint16_t relayPulseUs = readRelaySwitchPulseUs();
    filteredInputPulseUs = smoothReceiverPulseUs(filteredInputPulseUs, inputPulseUs);
    uint8_t targetStep = inputPulseToX9cStep(filteredInputPulseUs);
    uint32_t now = millis();
    updateGyroModeSwitch();
    updateGyroHeadingHold();
    sendHoverGyroCorrection();

    if (!tempInitialized || (now - lastTempSample >= TEMP_SAMPLE_INTERVAL_MS)) {
        readAllTemperatures(currentTemps);
        currentMaxTemp = getMaxTemperature(currentTemps);
        updateThermalProtection(currentTemps);

        for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
            if (!tempInitialized || fabsf(currentTemps[i] - displayTemps[i]) >= TEMP_DISPLAY_CHANGE_THRESHOLD_C) {
                displayTemps[i] = currentTemps[i];
            }
        }
        displayMaxTemp = getMaxTemperature(displayTemps);

        tempInitialized = true;
        lastTempSample = now;
    }

    // Aktualizacja wyświetlacza TFT

    // Drukowanie statusu na UART
    if (millis() - lastStatusPrint >= STATUS_PRINT_INTERVAL_MS) {
        printStatus(filteredInputPulseUs, targetStep, currentTemps, currentMaxTemp, relayPulseUs, relayEnabled);
        lastStatusPrint = millis();
    }

    if (!startupUnlocked) {
        resetControlToSafeState();
        setRelayState(false);
        filteredInputPulseUs = hasValidReceiverSignal(inputPulseUs) ? inputPulseUs : 0;

        if (isInputAtZero(inputPulseUs)) {
            startupUnlocked = true;
            startupMessageShown = false;
            signalLossLatched = false;
            Serial.println("Wejscie jest na 0. Sterowanie odblokowane.");
        } else if (!hasValidReceiverSignal(inputPulseUs)) {
            if (!startupMessageShown) {
                startupMessageShown = true;
                Serial.println("Brak poprawnego sygnalu RC. Oczekuje 1000-2000us / 50Hz.");
            }
        } else if (!startupMessageShown) {
            startupMessageShown = true;
            Serial.println("Czekam na zejscie gazu RC do minimum...");
        }
    } else {
        if (!hasValidReceiverSignal(inputPulseUs)) {
            if (!signalLossLatched) {
                signalLossLatched = true;
                startupUnlocked = false;
                startupMessageShown = false;
                Serial.println("Utrata sygnalu RC. Powrot do blokady startu.");
            }
            filteredInputPulseUs = 0;
            resetControlToSafeState();
            setRelayState(false);
        } else {
            signalLossLatched = false;
            if (now - lastX9cStepUpdate >= X9C_STEP_UPDATE_INTERVAL_MS) {
                x9cSetStep(limitX9cStepChange(x9cCurrentStep, targetStep));
                lastX9cStepUpdate = now;
            }
            updateRelayFromRc(relayPulseUs);

            if (ENABLE_PWM_OUTPUT && isSweeping) {
                uint32_t elapsedTime = millis() - sweepStartTime;

                if (elapsedTime >= SWEEP_DURATION_MS) {
                    sweepDirection = !sweepDirection;
                    sweepStartTime = millis();

                    if (sweepDirection) {
                        Serial.println("Sweep UP: 0 -> 15kHz");
                    } else {
                        Serial.println("Sweep DOWN: 15kHz -> 0");
                    }
                } else {
                    uint32_t newFreq;

                    if (sweepDirection) {
                        newFreq = FREQ_MIN + (uint64_t)(FREQ_MAX - FREQ_MIN) * elapsedTime / SWEEP_DURATION_MS;
                    } else {
                        newFreq = FREQ_MAX - (uint64_t)(FREQ_MAX - FREQ_MIN) * elapsedTime / SWEEP_DURATION_MS;
                    }

                    currentFreq = newFreq;
                    startOrUpdatePwm(currentFreq, currentDutyPercent);
                }
            } else if (ENABLE_PWM_OUTPUT) {
                currentFreq = inputPulseToFreq(inputPulseUs);
                startOrUpdatePwm(currentFreq, currentDutyPercent);
            } else {
                stopPwm();
            }
        }
    }

    if (now - lastTftUpdate >= TFT_UPDATE_INTERVAL_MS) {
        displayInputPulseUs = filteredInputPulseUs;
        displayTargetStep = targetStep;
        displayOutputStep = x9cCurrentStep;
        displayRelayPulseUs = relayPulseUs;
        displayRelayEnabled = relayEnabled;
        displayPwmFreq = currentFreq;
        displayStartupUnlocked = startupUnlocked;
        displaySweeping = isSweeping;
        lastTftUpdate = now;
    }

    tftPrintStatus(displayInputPulseUs, displayTargetStep, displayOutputStep, displayTemps, displayMaxTemp, displayRelayPulseUs, displayRelayEnabled, displayPwmFreq, displayStartupUnlocked, displaySweeping);

    processUsbSerialCommands();
}

