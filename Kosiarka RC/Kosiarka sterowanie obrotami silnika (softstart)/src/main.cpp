#include <Arduino.h>
#include <esp_system.h>
#include <math.h>

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
static const uint16_t RC_PULSE_VALID_MIN_US = 900;
static const uint16_t RC_PULSE_VALID_MAX_US = 2100;
static const uint16_t RC_PULSE_MIN_US = 1000;
static const uint16_t RC_PULSE_MAX_US = 2000;
static const uint16_t RC_PULSE_ZERO_THRESHOLD_US = 1050;
// Zakres RC, na ktory rozciagamy caly bezpieczny zakres potencjometru X9C.
static const uint16_t RC_PULSE_X9C_ACTIVE_MIN_US = 1000;
static const uint16_t RC_PULSE_X9C_ACTIVE_MAX_US = 4000;
static const uint32_t RC_PULSE_READ_TIMEOUT_US = 30000;
static const uint8_t RC_FILTER_SMOOTHING_SHIFT = 2;  // 1/4 nowej probki
// Ogranicza szybkosc zmian X9C; wieksza wartosc = szybsza reakcja silnika.
static const uint8_t X9C_MAX_STEP_CHANGE_PER_UPDATE = 1;
static const uint16_t X9C_STEP_UPDATE_INTERVAL_MS = 150;

// Temperatura
static const int TEMP_PIN = 35;
static const float TEMP_THRESHOLD = 80.0;
static const float TEMP_HYSTERESIS = 5.0;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 500;
static const uint16_t TFT_UPDATE_INTERVAL_MS = 250;
static const uint16_t TEMP_SAMPLE_INTERVAL_MS = 500;
static const float TEMP_DISPLAY_CHANGE_THRESHOLD_C = 0.5f;
static const uint8_t NTC_SAMPLE_COUNT = 8;
static const float NTC_R25 = 10000.0f;               // NTC 10k przy 25 C
static const float NTC_BETA = 3950.0f;               // Zmien, jesli Twoj NTC ma inny wspolczynnik B
static const float NTC_SERIES_RESISTOR = 10000.0f;   // Rezystor dzielnika 10k
static const float TEMP_CALIBRATION_OFFSET_C = -5.0f;
static const bool NTC_TO_GND = true;                 // true: NTC do GND, rezystor 10k do 3.3V
static const bool DIAGNOSTIC_DISPLAY_ONLY = false;
static const int TFT_BL_PIN = 22;
static const bool TFT_BL_INVERT = false;

// TFT Display
TFT_eSPI tft = TFT_eSPI();
static const uint32_t TFT_BG_COLOR = TFT_BLACK;
static const uint32_t TFT_TEXT_COLOR = TFT_WHITE;
static const uint32_t TFT_ALERT_COLOR = TFT_RED;
static const uint32_t TFT_OK_COLOR = TFT_GREEN;

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
uint16_t lastTempAdcRaw = 0;

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

void tftShowDiagnosticScreen(float temp, uint32_t color, const char* colorName) {
    tft.fillScreen(color);
    tft.setTextSize(2);
    tft.setTextColor(TFT_TEXT_COLOR, color);
    tft.setCursor(8, 8);
    tft.println("TEST TFT");

    tft.setTextSize(1);
    tft.setCursor(8, 36);
    tft.print("Kolor: ");
    tft.println(colorName);

    tft.setCursor(8, 52);
    tft.print("Temp: ");
    tft.print(temp, 1);
    tft.println(" C");

    tft.setCursor(8, 68);
    tft.print("ADC TMP: ");
    tft.println(lastTempAdcRaw);

    tft.setCursor(8, 84);
    tft.println("PWM/X9C wylaczone");

    tft.setCursor(8, 100);
    tft.println("GPIO TFT: CS5 DC16 RST17");

    tft.setCursor(8, 116);
    tft.println("Jesli nic nie widac:");

    tft.setCursor(8, 132);
    tft.println("1. zly driver TFT");

    tft.setCursor(8, 148);
    tft.println("2. brak podswietlenia");

    tft.setCursor(8, 164);
    tft.println("3. zle polaczenie");
}

void tftPrintStatus(uint16_t inputPulseUs, uint8_t targetStep, uint8_t outputStep, float temp, uint32_t pwmFreq, bool displayStartupUnlocked, bool displaySweeping) {
    static bool layoutDrawn = false;
    static int lastInputPct = -1;
    static int lastInputPulseUs = -1;
    static int lastOutputPct = -1;
    static int lastOutputStep = -1;
    static int lastTempDeciC = -10000;
    static int lastTempBar = -1;
    static bool lastThermalShutdown = false;
    static uint32_t lastPwmFreq = 0xFFFFFFFF;
    static int lastStartupUnlocked = -1;
    static int lastSweeping = -1;
    static int lastStartState = -1;
    const uint16_t panel = tft.color565(20, 24, 34);
    const uint16_t panelSoft = tft.color565(38, 44, 58);
    const uint16_t accentIn = tft.color565(0, 200, 175);
    const uint16_t accentOut = tft.color565(255, 170, 70);
    const uint16_t accentTemp = thermalShutdownActive ? TFT_RED : tft.color565(90, 220, 120);
    const uint16_t textDim = tft.color565(165, 175, 188);
    const uint16_t startWaitColor = tft.color565(255, 165, 0);

    const int inputPct = min(100, (targetStep * 100) / X9C_DISPLAY_FULL_SCALE_STEP);
    const int outputPct = min(100, (outputStep * 100) / X9C_DISPLAY_FULL_SCALE_STEP);
    const int barIn = (112 * inputPct) / 100;
    const int barOut = (112 * outputPct) / 100;
    const float tempClamped = min(max(temp, 0.0f), TEMP_THRESHOLD);
    const int tempBar = (int)((200.0f * tempClamped) / TEMP_THRESHOLD);
    const int tempDeciC = (int)roundf(temp * 10.0f);
    const int startState = thermalShutdownActive ? 0 : (displayStartupUnlocked ? 2 : 1);
    const char* startLabel = (startState == 0) ? "OFF" : ((startState == 1) ? "WAIT" : "GO");
    const uint16_t startColor = (startState == 0) ? TFT_RED : ((startState == 1) ? startWaitColor : TFT_GREEN);

    if (!layoutDrawn) {
        tft.fillScreen(TFT_BG_COLOR);

        tft.fillRoundRect(8, 8, 264, 24, 8, panel);
        tft.drawRoundRect(8, 8, 264, 24, 8, panelSoft);
        tft.setTextColor(TFT_TEXT_COLOR, panel);
        tft.drawString("STEROWANIE SILNIKA", 16, 13, 2);

        tft.fillRoundRect(8, 40, 128, 86, 12, panel);
        tft.drawRoundRect(8, 40, 128, 86, 12, accentIn);
        tft.setTextColor(textDim, panel);
        tft.drawString("WEJSCIE", 18, 50, 2);

        tft.fillRoundRect(144, 40, 128, 86, 12, panel);
        tft.drawRoundRect(144, 40, 128, 86, 12, accentOut);
        tft.setTextColor(textDim, panel);
        tft.drawString("WYJSCIE", 154, 50, 2);

        layoutDrawn = true;
    }

    if (inputPct != lastInputPct || inputPulseUs != lastInputPulseUs) {
        tft.fillRect(18, 70, 84, 26, panel);
        tft.setTextColor(TFT_TEXT_COLOR, panel);
        tft.drawString(String(inputPct), 18, 74, 4);
        tft.drawString("%", 98, 84, 2);
        tft.fillRect(18, 104, 100, 12, panel);
        tft.setTextColor(textDim, panel);
        if (inputPulseUs == 0) {
            tft.drawString("RC BRAK", 18, 104, 2);
        } else {
            tft.drawString("RC " + String(inputPulseUs) + "us", 18, 104, 2);
        }
        tft.fillRoundRect(18, 118, 112, 6, 3, panelSoft);
        if (barIn > 0) {
            tft.fillRoundRect(18, 118, barIn, 6, 3, accentIn);
        }
        lastInputPct = inputPct;
        lastInputPulseUs = inputPulseUs;
    }

    if (outputPct != lastOutputPct || outputStep != lastOutputStep) {
        tft.fillRect(154, 70, 84, 26, panel);
        tft.setTextColor(TFT_TEXT_COLOR, panel);
        tft.drawString(String(outputPct), 154, 74, 4);
        tft.drawString("%", 234, 84, 2);
        tft.fillRect(154, 104, 100, 12, panel);
        tft.setTextColor(textDim, panel);
        tft.drawString("STEP " + String(outputStep), 154, 104, 2);
        tft.fillRoundRect(154, 118, 112, 6, 3, panelSoft);
        if (barOut > 0) {
            tft.fillRoundRect(154, 118, barOut, 6, 3, accentOut);
        }
        lastOutputPct = outputPct;
        lastOutputStep = outputStep;
    }

    if (tempDeciC != lastTempDeciC || tempBar != lastTempBar || thermalShutdownActive != lastThermalShutdown) {
        tft.fillRoundRect(8, 138, 264, 66, 12, panel);
        tft.drawRoundRect(8, 138, 264, 66, 12, accentTemp);
        tft.setTextColor(textDim, panel);
        tft.drawString("TEMPERATURA", 18, 148, 2);
        tft.setTextColor(accentTemp, panel);
        tft.drawString(String(temp, 1) + "C", 18, 166, 4);
        tft.fillRoundRect(18, 190, 200, 7, 3, panelSoft);
        if (tempBar > 0) {
            tft.fillRoundRect(18, 190, tempBar, 7, 3, accentTemp);
        }
        tft.setTextColor(thermalShutdownActive ? TFT_ALERT_COLOR : TFT_OK_COLOR, panel);
        tft.drawString(thermalShutdownActive ? "PRZEGRZANIE" : "OK", 220, 172, 2);
        lastTempDeciC = tempDeciC;
        lastTempBar = tempBar;
        lastThermalShutdown = thermalShutdownActive;
    }

    if (pwmFreq != lastPwmFreq || (int)displayStartupUnlocked != lastStartupUnlocked || (int)displaySweeping != lastSweeping || startState != lastStartState) {
        tft.fillRoundRect(8, 212, 264, 20, 8, panel);
        tft.drawRoundRect(8, 212, 264, 20, 8, panelSoft);
        tft.setTextColor(textDim, panel);
        if (ENABLE_PWM_OUTPUT) {
            tft.drawString("PWM " + formatFrequencyLabel(pwmFreq), 14, 216, 2);
            tft.drawString("START", 100, 216, 2);
            tft.setTextColor(startColor, panel);
            tft.drawString(startLabel, 142, 216, 2);
        } else {
            tft.drawString("X9C RC", 14, 216, 2);
            tft.drawString("START", 100, 216, 2);
            tft.setTextColor(startColor, panel);
            tft.drawString(startLabel, 142, 216, 2);
        }
        tft.setTextColor(textDim, panel);
        tft.drawString(ENABLE_PWM_OUTPUT ? (displaySweeping ? "SWEEP" : "MANUAL") : "DIRECT", 206, 216, 2);
        lastPwmFreq = pwmFreq;
        lastStartupUnlocked = displayStartupUnlocked ? 1 : 0;
        lastSweeping = displaySweeping ? 1 : 0;
        lastStartState = startState;
    }
}

// =========================
// FUNKCJE
// =========================
void x9cSetStep(uint8_t targetStep);
void printStatus(uint16_t inputPulseUs, uint8_t targetStep, float temp);

uint32_t dutyPercentToRaw(uint8_t percent, int resolutionBits) {
    uint32_t maxDuty = (1UL << resolutionBits) - 1;
    return (maxDuty * percent) / 100;
}

void stopPwm() {
    if (ENABLE_PWM_OUTPUT) {
        ledcWrite(PWM_CHANNEL, 0);
    }
    digitalWrite(PWM_PIN, LOW);
    currentFreq = 0;
}

void resetControlToSafeState() {
    x9cSetStep(0);
    stopPwm();
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

uint16_t readReceiverPulseUs() {
    uint32_t pulseUs = pulseIn(RC_INPUT_PIN, HIGH, RC_PULSE_READ_TIMEOUT_US);

    if (!hasValidReceiverSignal((uint16_t)pulseUs)) {
        return 0;
    }

    return (uint16_t)pulseUs;
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

float readTemperature() {
    uint32_t adcSum = 0;
    for (uint8_t i = 0; i < NTC_SAMPLE_COUNT; i++) {
        adcSum += analogRead(TEMP_PIN);
    }

    float adc = (float)adcSum / NTC_SAMPLE_COUNT;
    lastTempAdcRaw = (uint16_t)adc;
    if (adc <= 0.0f || adc >= 4095.0f) {
        return 999.0;
    }

    float voltage = adc * 3.3f / 4095.0f;
    if (voltage <= 0.0f || voltage >= 3.3f) {
        return 999.0;
    }

    float resistance;
    if (NTC_TO_GND) {
        resistance = NTC_SERIES_RESISTOR * (voltage / (3.3f - voltage));
    } else {
        resistance = NTC_SERIES_RESISTOR * ((3.3f - voltage) / voltage);
    }

    if (!(resistance > 0.0f)) {
        return 999.0;
    }

    float tempK = 1.0f / (1.0f / 298.15f + (1.0f / NTC_BETA) * log(resistance / NTC_R25));
    return (tempK - 273.15f) + TEMP_CALIBRATION_OFFSET_C;
}

void updateThermalProtection(float temp) {
    if (!thermalShutdownActive && temp >= TEMP_THRESHOLD) {
        thermalShutdownActive = true;
        startupUnlocked = false;
        startupMessageShown = false;
        resetControlToSafeState();

        Serial.print("Przegrzanie! Temp: ");
        Serial.print(temp);
        Serial.println(" C. Wyjscie ustawione na 0.");
        Serial.println("Po schlodzeniu ustaw potencjometr na 0, aby wznowic prace.");
        return;
    }

    if (thermalShutdownActive && temp <= (TEMP_THRESHOLD - TEMP_HYSTERESIS)) {
        thermalShutdownActive = false;
        Serial.print("Temperatura spadla do ");
        Serial.print(temp);
        Serial.println(" C. Zabezpieczenie termiczne zwolnione.");
    }

    if (thermalShutdownActive) {
        resetControlToSafeState();
    }
}

void printStatus(uint16_t inputPulseUs, uint8_t targetStep, float temp) {
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
    Serial.print(" TEMP=");
    Serial.print(temp, 1);
    if (ENABLE_PWM_OUTPUT) {
        Serial.print("C PWM=");
        Serial.print(currentFreq);
        Serial.print("Hz START=");
    } else {
        Serial.print("C MODE=X9C START=");
    }
    Serial.print(startupUnlocked ? "ON" : "WAIT");
    Serial.print(" THERM=");
    Serial.println(thermalShutdownActive ? "LOCK" : "OK");
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

void setup() {
    Serial.begin(115200);
    delay(300);
    printBootStage("Start programu");
    Serial.print("[BOOT] Reset reason=");
    Serial.println((int)esp_reset_reason());
    Serial.println("[BOOT] Jesli tu nic nie widac, ESP32 nie startuje poprawnie.");
    printBootStage("Init BL");
    initBacklight();
    analogReadResolution(12);
    analogSetPinAttenuation(TEMP_PIN, ADC_11db);
    if (DIAGNOSTIC_DISPLAY_ONLY) {
        printBootStage("Tryb diagnostyczny TFT");
        printBootStage("Init TFT");
        tftInit();
        tftClearScreen();
        printBootStage("TFT gotowy");
        tftShowDiagnosticScreen(readTemperature(), TFT_RED, "CZERWONY");
        Serial.println("[BOOT] Tryb diagnostyczny aktywny: PWM/X9C pominiete.");
        printBootStage("Setup zakonczony");
        return;
    }
    
    // Inicjalizacja pinów
    printBootStage("Init PWM");
    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW);
    pinMode(RC_INPUT_PIN, INPUT);

    // Inicjalizacja X9C103S
    printBootStage("Init X9C");
    x9cInit();
    x9cForceToZero();
    stopPwm();
    
    // Inicjalizacja wyświetlacza TFT
    printBootStage("Init TFT");
    tftInit();
    tftClearScreen();
    printBootStage("TFT gotowy");
    
    tft.setTextColor(TFT_TEXT_COLOR, TFT_BG_COLOR);
    tft.setTextSize(1);
    tft.setCursor(10, 10);
    tft.println("INICJALIZACJA...");
    tft.setCursor(10, 35);
    tft.println("Blokada startu aktywna");
    tft.setCursor(10, 55);
    tft.println("Ustaw potencjometr na 0");

    Serial.println("Blokada startu aktywna. Ustaw potencjometr na 0.");
    printBootStage("Setup zakonczony");
}

void loop() {
    static bool tempInitialized = false;
    static float currentTemp = 0.0f;
    static float displayTemp = 0.0f;
    static uint32_t lastTempSample = 0;
    static uint16_t displayInputPulseUs = 0;
    static uint8_t displayTargetStep = 0;
    static uint8_t displayOutputStep = 0;
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
            float temp = readTemperature();
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

            tftShowDiagnosticScreen(temp, color, colorName);
            Serial.print("[DIAG] TFT kolor=");
            Serial.print(colorName);
            Serial.print(" TEMP=");
            Serial.print(temp, 1);
            Serial.println("C");

            colorIndex = (colorIndex + 1) % 4;
            lastDiagUpdate = millis();
        }

        delay(20);
        return;
    }

    uint16_t inputPulseUs = readReceiverPulseUs();
    filteredInputPulseUs = smoothReceiverPulseUs(filteredInputPulseUs, inputPulseUs);
    uint8_t targetStep = inputPulseToX9cStep(filteredInputPulseUs);
    uint32_t now = millis();

    if (!tempInitialized || (now - lastTempSample >= TEMP_SAMPLE_INTERVAL_MS)) {
        currentTemp = readTemperature();
        updateThermalProtection(currentTemp);

        if (!tempInitialized || fabsf(currentTemp - displayTemp) >= TEMP_DISPLAY_CHANGE_THRESHOLD_C) {
            displayTemp = currentTemp;
        }

        tempInitialized = true;
        lastTempSample = now;
    }

    // Aktualizacja wyświetlacza TFT

    // Drukowanie statusu na UART
    if (millis() - lastStatusPrint >= STATUS_PRINT_INTERVAL_MS) {
        printStatus(filteredInputPulseUs, targetStep, currentTemp);
        lastStatusPrint = millis();
    }

    if (!startupUnlocked) {
        resetControlToSafeState();
        filteredInputPulseUs = hasValidReceiverSignal(inputPulseUs) ? inputPulseUs : 0;

        if (thermalShutdownActive) {
            delay(50);
            return;
        }

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
        } else {
            signalLossLatched = false;
            if (now - lastX9cStepUpdate >= X9C_STEP_UPDATE_INTERVAL_MS) {
                x9cSetStep(limitX9cStepChange(x9cCurrentStep, targetStep));
                lastX9cStepUpdate = now;
            }

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
        displayPwmFreq = currentFreq;
        displayStartupUnlocked = startupUnlocked;
        displaySweeping = isSweeping;
        lastTftUpdate = now;
    }

    tftPrintStatus(displayInputPulseUs, displayTargetStep, displayOutputStep, displayTemp, displayPwmFreq, displayStartupUnlocked, displaySweeping);

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() == 0) {
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

    delay(50);
}

