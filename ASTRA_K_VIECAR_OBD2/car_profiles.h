#pragma once
#include <Arduino.h>

struct CarProfile {
  const char* key;      // krótki identyfikator
  const char* name;     // opis dla konsoli
  const char* nvsNS;    // namespace NVS (<=15 znaków)

  const char* filterLabel; // "DPF" albo "GPF"  <-- DODANE

  uint32_t header;

  // PIDs (Mode 01)
  uint8_t pid_vbat;
  uint8_t pid_ect;
  uint8_t pid_oil;

  // DIDs (Mode 22)
  uint16_t did_soot;
  uint16_t did_km;
  uint16_t did_burn;
  uint16_t did_state;
  uint16_t did_prog;
  uint16_t did_oil_uds;
};

static const CarProfile PROFILES[] = {
  {
    "ASTRAK_16",
    "OPEL ASTRA K 1.6 CDTI",
    "ak16",
    "DPF",
    0x7E0,
    0x42, 0x05, 0x5C,
    0x336A, 0x3039, 0x20FA, 0x20F2, 0x0000, 0x1154
  },
  {
    "GRANDLANDX_16",
    "OPEL GRANDLANDX 1.6 CDTI",
    "gx16",
    "DPF",
    0x7E0,
    0x42, 0x05, 0x5C,
    0x3275, 0x3277, 0x20FA, 0x20F2, 0x0000, 0x1154
  },
  {
    "ASTRAJ_17",
    "OPEL ASTRA J 1.7 CDTI",
    "aj17",
    "DPF",
    0x7E0,
    0x42, 0x05, 0x5C,
    0x3275, 0x3277, 0x20FA, 0x20F2, 0x0000, 0x1154
  },
  {
    "INSYGNIA_20",
    "OPEL INSYGNIA A 2.0 CDTI",
    "insa20",
    "DPF",
    0x7E0,
    0x42, 0x05, 0x5C,
    0x336A, 0x3039, 0x20FA, 0x20F2, 0x0000, 0x1154
  },

  // --- NOWY PROFIL: ASTRA K 1.4 BENZYNA (GPF) ---
  {
    "ASTRAK_14",
    "OPEL ASTRA K 1.4 TURBO (BENZYNA, GPF)",
    "ak14",
    "GPF",
    0x7E0,
    0x42, 0x05, 0x5C,
    // startowo daję te same DIDy co Astra K diesel (łatwo zmienisz z konsoli, jeśli NO DATA)
    0x336A, 0x3039, 0x20FA, 0x20F2, 0x0000, 0x1154
  }
};

static const int PROFILE_COUNT = (int)(sizeof(PROFILES) / sizeof(PROFILES[0]));
