#include <Arduino.h>
#include <SuplaDevice.h>
#include <supla/network/esp_wifi.h>
#include <supla/control/relay.h>
#include <supla/sensor/general_purpose_measurement.h>

// ==== USTAWIENIA ====
const char* WIFI_SSID   = "iPhone 15"; // wpisz swoja nazwę sieci
const char* WIFI_PASS   = "qweasdzxc"; // wpisz hasło sieci
const char* SUPLA_SERVER = "svr122.supla.org";   // wpisz swój serwer z SUPLA Cloud (supla - supla-dev - adres serwer)
const char* SUPLA_EMAIL  = "mateuszpiwowarskioppl@gmail.com"; // wpisz email konta supla cloud

// GUID i AUTHKEY generujesz dla urządzenia (SUPLA udostępnia generatory). (Są stałe i tylko do jednego urządzenia )
// W przykładach Supli jest dokładnie ten schemat użycia SuplaDevice.begin(...). :contentReference[oaicite:4]{index=4}
// GUID:    https://www.supla.org/arduino/get-guid
// AUTHKEY: https://www.supla.org/arduino/get-authkey
// W SUPLA Cloud upewnij się, że masz włączone dodawanie/rejestrację nowych urządzeń (czasem bywa wyłączone).

char GUID[SUPLA_GUID_SIZE] = {0xCB,0x03,0xCC,0x3F,0xB0,0x20,0x95,0x32,0x02,0x48,0xCB,0xB1,0xAF,0xAB,0x72,0xAD};
char AUTHKEY[SUPLA_AUTHKEY_SIZE] = {0x8C,0x0E,0xF0,0x7D,0x3A,0xD3,0xDC,0xAF,0xB6,0xFB,0x78,0x43,0xD2,0x53,0x9A,0x91};

Supla::ESPWifi wifi(WIFI_SSID, WIFI_PASS);

// ==== PINY (ESP32-C3) ====
static constexpr uint8_t ADC_PIN   = 3;  // GPIO3 (ADC)
static constexpr uint8_t RELAY_PIN = 6;  // GPIO6 (OUT)

// ==== DZIELNIK 100k/22k ====
static constexpr float R1 = 100000.0f;
static constexpr float R2 = 22000.0f;
static constexpr float SCALE = (R1 + R2) / R2;   // 

Supla::Sensor::GeneralPurposeMeasurement *vmeas = nullptr;

static float readVoltageVin() {
  // uśrednianie, żeby uspokoić odczyt
  const int N = 32;
  uint32_t sum_mv = 0;

  for (int i = 0; i < N; i++) {
    sum_mv += analogReadMilliVolts(ADC_PIN);  // wynik w mV (skalibrowany) :contentReference[oaicite:5]{index=5}
    delay(2);
  }

  float v_adc = (sum_mv / (float)N) / 1000.0f;  // V na pinie ADC
  return v_adc * SCALE;                          // V na wejściu (do 15V i więcej)
}

void setup() {
  // Przekaźnik aktywny LOW: na start wymuś OFF
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // OFF dla active-LOW

  // ADC: ustaw pod ESP32-C3 sensowny zakres (11 dB ~ do ok. 2.5V dla C3) :contentReference[oaicite:6]{index=6}
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  // Relay(int pin, bool highIsOn=true, ...) -> dla active-LOW daj false :contentReference[oaicite:7]{index=7}
  new Supla::Control::Relay(RELAY_PIN, false);

  // Kanał pomiarowy (GPM)
  vmeas = new Supla::Sensor::GeneralPurposeMeasurement();
  vmeas->setDefaultUnitAfterValue("V");
  vmeas->setDefaultValuePrecision(2);

  SuplaDevice.begin(GUID, SUPLA_SERVER, SUPLA_EMAIL, AUTHKEY);
}

void loop() {
  SuplaDevice.iterate();

  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();

    float vin = readVoltageVin();
    vin = roundf(vin * 100.0f) / 100.0f;

    vmeas->setValue(vin);
  }
}
