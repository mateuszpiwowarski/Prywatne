#include <Arduino.h>
#include <cmath>

#include <SuplaDevice.h>
#include <supla/control/relay.h>
#include <supla/network/esp_wifi.h>
#include <supla/sensor/general_purpose_measurement.h>

namespace {

constexpr char WIFI_SSID[] = "Security 367/3";
constexpr char WIFI_PASS[] = "Bae5p4ukeprj";
constexpr char SUPLA_SERVER[] = "svr122.supla.org";
constexpr char SUPLA_EMAIL[] = "mateuszpiwowarskioppl@gmail.com";

constexpr bool RELAY_ACTIVE_HIGH = false;

constexpr uint8_t ADC_PIN = 3;
constexpr uint8_t RELAY1_PIN = 6;
constexpr uint8_t RELAY2_PIN = 7;

constexpr float R1 = 100000.0f;
constexpr float R2 = 22000.0f;
constexpr float VOLTAGE_SCALE = (R1 + R2) / R2;

constexpr uint8_t ADC_SAMPLES = 32;
constexpr uint32_t MEASUREMENT_INTERVAL_MS = 1000;
constexpr uint8_t BOARD_LED_BLUE_BRIGHTNESS = 2;

constexpr uint8_t BOARD_RGB_LED_PINS[] = {48, 38};

// GUID i AUTHKEY generujesz dla urzadzenia. Sa stale i przypisane do jednego
// urzadzenia w SUPLA Cloud.
// GUID:    https://www.supla.org/arduino/get-guid
// AUTHKEY: https://www.supla.org/arduino/get-authkey
// Upewnij sie tez, ze w SUPLA Cloud masz wlaczone dodawanie nowych urzadzen.
char GUID[SUPLA_GUID_SIZE] = {
    static_cast<char>(0xCB), static_cast<char>(0x03),
    static_cast<char>(0xCC), static_cast<char>(0x3F),
    static_cast<char>(0xB0), static_cast<char>(0x20),
    static_cast<char>(0x95), static_cast<char>(0x32),
    static_cast<char>(0x02), static_cast<char>(0x48),
    static_cast<char>(0xCB), static_cast<char>(0xB1),
    static_cast<char>(0xAF), static_cast<char>(0xAB),
    static_cast<char>(0x72), static_cast<char>(0xAD)};

char AUTHKEY[SUPLA_AUTHKEY_SIZE] = {
    static_cast<char>(0x8C), static_cast<char>(0x0E),
    static_cast<char>(0xF0), static_cast<char>(0x7D),
    static_cast<char>(0x3A), static_cast<char>(0xD3),
    static_cast<char>(0xDC), static_cast<char>(0xAF),
    static_cast<char>(0xB6), static_cast<char>(0xFB),
    static_cast<char>(0x78), static_cast<char>(0x43),
    static_cast<char>(0xD2), static_cast<char>(0x53),
    static_cast<char>(0x9A), static_cast<char>(0x91)};

Supla::ESPWifi wifi(WIFI_SSID, WIFI_PASS);
Supla::Sensor::GeneralPurposeMeasurement *batteryVoltage = nullptr;

void configureRelayPin(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, RELAY_ACTIVE_HIGH ? LOW : HIGH);
}

void setBoardLedBlue(uint8_t brightness) {
  for (uint8_t pin : BOARD_RGB_LED_PINS) {
    neopixelWrite(pin, 0, 0, brightness);
  }
}

float readInputVoltage() {
  uint32_t sumMv = 0;

  for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
    sumMv += analogReadMilliVolts(ADC_PIN);
    delay(2);
  }

  const float adcVoltage = (sumMv / static_cast<float>(ADC_SAMPLES)) / 1000.0f;
  return adcVoltage * VOLTAGE_SCALE;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  setBoardLedBlue(BOARD_LED_BLUE_BRIGHTNESS);

  configureRelayPin(RELAY1_PIN);
  configureRelayPin(RELAY2_PIN);

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  auto *alarmRelay = new Supla::Control::Relay(RELAY1_PIN, RELAY_ACTIVE_HIGH);
  auto *lightRelay = new Supla::Control::Relay(RELAY2_PIN, RELAY_ACTIVE_HIGH);

  alarmRelay->setInitialCaption("Przekaznik alarm");
  lightRelay->setInitialCaption("Przekaznik swiatlo");

  batteryVoltage = new Supla::Sensor::GeneralPurposeMeasurement();
  batteryVoltage->setInitialCaption("Napiecie akumulatora");
  batteryVoltage->setDefaultUnitAfterValue("V");
  batteryVoltage->setDefaultValuePrecision(2);

  // If SSL turns out to be too heavy for this device, uncomment the next line.
  // wifi.enableSSL(false);

  SuplaDevice.begin(GUID, SUPLA_SERVER, SUPLA_EMAIL, AUTHKEY);
}

void loop() {
  SuplaDevice.iterate();

  static uint32_t lastMeasurementMs = 0;
  const uint32_t now = millis();

  if (now - lastMeasurementMs < MEASUREMENT_INTERVAL_MS) {
    return;
  }

  lastMeasurementMs = now;

  const float vin = std::round(readInputVoltage() * 100.0f) / 100.0f;

  if (batteryVoltage != nullptr) {
    batteryVoltage->setValue(vin);
  }
}
