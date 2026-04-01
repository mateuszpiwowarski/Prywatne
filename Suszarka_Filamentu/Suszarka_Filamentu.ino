#include <Arduino.h>
#include <math.h>

#include <SuplaDevice.h>
#include <supla/network/esp_wifi.h>
#include <supla/control/relay.h>
#include <supla/control/virtual_relay.h>
#include <supla/sensor/virtual_thermometer.h>

#if __has_include(<supla/sensor/virtual_hygromometer.h>)
  #include <supla/sensor/virtual_hygromometer.h>
#elif __has_include(<supla/sensor/virtual_hygro_meter.h>)
  #include <supla/sensor/virtual_hygro_meter.h>
#else
  #error "Brak naglowka VirtualHygroMeter. Zaktualizuj biblioteke SuplaDevice lub sprawdz nazwe pliku w .../libraries/SuplaDevice/src/supla/sensor/"
#endif

#include <DHT.h>

// ==== USTAWIENIA ====
const char* WIFI_SSID    = "UPC3524997";
const char* WIFI_PASS    = "Bae5p4ukeprj";
const char* SUPLA_SERVER = "svr122.supla.org";
const char* SUPLA_EMAIL  = "mateuszpiwowarskioppl@gmail.com";

char GUID[SUPLA_GUID_SIZE] = {0xA0,0x37,0xC3,0xE4,0x2F,0x42,0x10,0x7F,0x28,0x96,0x10,0x57,0x45,0x5D,0xCB,0xA1};
char AUTHKEY[SUPLA_AUTHKEY_SIZE] = {0xD5,0xC6,0xD3,0xEC,0xCD,0x30,0xF4,0x61,0x69,0x19,0xF9,0xC3,0x04,0x17,0xEE,0x4E};

// WiFi
Supla::ESPWifi wifi(WIFI_SSID, WIFI_PASS);

// ===================== PINY (ESP32-WROOM-32) =====================
static constexpr uint8_t NTC_ADC_PIN = 32;
static constexpr uint8_t DHT_PIN     = 27;
static constexpr uint8_t HEATER_PIN  = 26;

static constexpr bool HEATER_ACTIVE_HIGH = true;

// ===================== DHT =====================
static constexpr uint8_t DHT_TYPE = DHT22;
DHT dht(DHT_PIN, DHT_TYPE);

// ===================== OFFSETY =====================
static constexpr float NTC_OFFSET_C       = +1.1f;
static constexpr float DHT_TEMP_OFFSET_C  = +0.0f;
static constexpr float DHT_HUM_OFFSET_PCT = -2.0f;

// ===================== NTC10k parametry =====================
static constexpr float VCC     = 3.3f;
static constexpr float R_FIXED = 10000.0f;
static constexpr float R0      = 10000.0f;
static constexpr float T0_K    = 298.15f;
static constexpr float BETA    = 3950.0f;

// ===================== LOGIKA GRZAŁKI =====================
static constexpr float TEMP_CUTOFF_C = 70.0f;
static constexpr float TEMP_RESET_C  = 50.0f;

static constexpr float HUM_ON_PCT  = 35.0f;
static constexpr float HUM_OFF_PCT = 30.0f;

static constexpr uint32_t TEMP_MAX_AGE_MS = 4000;   // trochę luźniej
static constexpr uint32_t HUM_MAX_AGE_MS  = 15000;

// ===================== ODCIĄŻENIE: progi wysyłania do Supli =====================
static constexpr float NTC_SEND_DELTA_C   = 0.2f;   // wysyłaj gdy zmiana >= 0.2°C
static constexpr float DHT_T_SEND_DELTA_C = 0.3f;   // >= 0.3°C
static constexpr float HUM_SEND_DELTA_PCT = 0.5f;   // >= 1%

// ===================== SUPLA kanały =====================
Supla::Control::Relay *heater = nullptr;
Supla::Control::VirtualRelay *autoMode = nullptr;

Supla::Sensor::VirtualThermometer *ntcThermo = nullptr;
Supla::Sensor::VirtualThermometer *dhtThermo = nullptr;
Supla::Sensor::VirtualHygroMeter  *dhtHygro  = nullptr;

// ===================== STAN =====================
static float lastTempC   = NAN;
static float lastHumPct  = NAN;
static uint32_t lastTempMs = 0;
static uint32_t lastHumMs  = 0;

static bool overTempLock = false;
static bool heaterState  = false;

// ostatnio wysłane wartości (żeby nie spamować Supli)
static float sentNtcC   = NAN;
static float sentDhtC   = NAN;
static float sentHumPct = NAN;

// ===================== POMOCNICZE =====================
static bool actualHeaterIsOn() {
  int level = digitalRead(HEATER_PIN);
  return HEATER_ACTIVE_HIGH ? (level == HIGH) : (level == LOW);
}

static void setHeater(bool on) {
  if (!heater) return;
  if (on) heater->turnOn();
  else    heater->turnOff();
  heaterState = on;
}

static float readNtcTemperatureC() {
  const int N = 12;
  uint32_t sum_mv = 0;
  for (int i = 0; i < N; i++) {
    sum_mv += analogReadMilliVolts(NTC_ADC_PIN);
  }

  float v = (sum_mv / (float)N) / 1000.0f;
  if (v <= 0.01f || v >= (VCC - 0.01f)) return NAN;

  float r_ntc = R_FIXED * v / (VCC - v);
  float invT = (1.0f / T0_K) + (1.0f / BETA) * logf(r_ntc / R0);
  return (1.0f / invT) - 273.15f;
}

static void applyHeaterControl() {
  if (!heater) return;

  heaterState = actualHeaterIsOn();

  uint32_t now = millis();

  bool tempValid = (!isnan(lastTempC) && (now - lastTempMs <= TEMP_MAX_AGE_MS));
  bool humValid  = (!isnan(lastHumPct) && (now - lastHumMs  <= HUM_MAX_AGE_MS));

  bool autoEnabled = true;
  if (autoMode) autoEnabled = autoMode->isOn();

  // FAILSAFE: brak temperatury -> OFF + lock
  if (!tempValid) {
    overTempLock = true;
    setHeater(false);
    return;
  }

  // Blokada temp z histerezą
  if (!overTempLock && lastTempC >= TEMP_CUTOFF_C) overTempLock = true;
  if (overTempLock && lastTempC <= TEMP_RESET_C)   overTempLock = false;

  if (overTempLock) {
    setHeater(false);
    return;
  }

  // MANUAL: nie ruszaj (poza zabezpieczeniem temp)
  if (!autoEnabled) return;

  // AUTO: brak wilgotności -> OFF
  if (!humValid) {
    setHeater(false);
    return;
  }

  if (lastHumPct >= HUM_ON_PCT) {
    setHeater(true);
  } else if (lastHumPct <= HUM_OFF_PCT) {
    setHeater(false);
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, HEATER_ACTIVE_HIGH ? LOW : HIGH);

  analogReadResolution(12);
  analogSetPinAttenuation(NTC_ADC_PIN, ADC_11db);

  heater = new Supla::Control::Relay(HEATER_PIN, HEATER_ACTIVE_HIGH);
  heater->setInitialCaption("Grzalka");

  autoMode = new Supla::Control::VirtualRelay();
  autoMode->setInitialCaption("Tryb AUTO (ON=auto)");

  ntcThermo = new Supla::Sensor::VirtualThermometer();
  ntcThermo->setInitialCaption("Temp Grzałki NTC");

  dhtThermo = new Supla::Sensor::VirtualThermometer();
  dhtThermo->setInitialCaption("Temp Komory DHT22");

  dhtHygro = new Supla::Sensor::VirtualHygroMeter();
  dhtHygro->setInitialCaption("Wilgotnosc Komory DHT22");

  dht.begin();

  SuplaDevice.begin(GUID, SUPLA_SERVER, SUPLA_EMAIL, AUTHKEY);

  autoMode->turnOn();
  setHeater(false);
}

void loop() {
  SuplaDevice.iterate();

  uint32_t now = millis();

  // NTC co 1000 ms (rzadziej niż wcześniej)
  static uint32_t lastNtc = 0;
  if (now - lastNtc >= 1000) {
    lastNtc = now;

    float t = readNtcTemperatureC();
    if (!isnan(t)) {
      t += NTC_OFFSET_C;
      t = roundf(t * 10.0f) / 10.0f;

      // nie spamuj Supli
      if (isnan(sentNtcC) || fabsf(t - sentNtcC) >= NTC_SEND_DELTA_C) {
        ntcThermo->setValue(t);
        sentNtcC = t;
      }

      lastTempC = t;
      lastTempMs = now;
    }
  }

  // DHT22 co 2500 ms (jak było), ale setValue tylko przy zmianie
  static uint32_t lastDht = 0;
  if (now - lastDht >= 2500) {
    lastDht = now;

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t)) {
      t += DHT_TEMP_OFFSET_C;
      t = roundf(t * 10.0f) / 10.0f;

      if (isnan(sentDhtC) || fabsf(t - sentDhtC) >= DHT_T_SEND_DELTA_C) {
        dhtThermo->setValue(t);
        sentDhtC = t;
      }
    }

    if (!isnan(h)) {
      h += DHT_HUM_OFFSET_PCT;
      if (h < 0) h = 0;
      if (h > 100) h = 100;
      h = roundf(h * 10.0f) / 10.0f;

      if (isnan(sentHumPct) || fabsf(h - sentHumPct) >= HUM_SEND_DELTA_PCT) {
        dhtHygro->setValue(h);
        sentHumPct = h;
      }

      lastHumPct = h;
      lastHumMs = now;
    }
  }

  // Kontrola co 250 ms
  static uint32_t lastCtl = 0;
  if (now - lastCtl >= 250) {
    lastCtl = now;
    applyHeaterControl();
  }

  // ODCIĄŻENIE CPU + stabilniejsze WiFi
  delay(2);
}
