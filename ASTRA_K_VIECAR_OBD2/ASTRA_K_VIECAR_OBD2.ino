/*
  ============================================================
   ESP32 + ELM327 (BLE) + TFT_eSPI
   Display: ST7789V3 1.47" 172x320  -> landscape 320x172
  ============================================================

  Co to robi:
  - Łączy się po BLE z adapterem ELM327 (sam znajduje parę RX(write) + TX(notify))
  - Inicjuje ELM (ATZ / E0 / L0 / H0 / SP0)
  - Odczytuje parametry DPF/temperatury:
      * UDS (service 22 / DID xxxx) -> idzie na nagłówek CAN: g_header (np. 0x7E0)
      * OBD-II (service 01 / PID xx) -> idzie na functional broadcast (0x7DF lub 0x18DB33F1)
  - Rysuje 4 ekrany:
      1) DASH    (normalny podgląd)
      2) BURN    (wypalanie / regeneracja)
      3) SUCCESS (regen zakończona)
      4) ABORT   (regen przerwana / nieudana)

  ------------------------------------------------------------
  Dane (Astra K 1.6 CDTI – typowe wartości):
  ------------------------------------------------------------
  UDS / DID (Service 22 xxxx) – idą na nagłówek CAN g_header (np. 0x7E0):
    - DPF soot %        : 22 336A  -> A = % (0..100)
    - Km since regen    : 22 3039  -> A*256+B (km)
    - Regen req         : 22 20FA  -> bit0 w bajcie A (REQ)
    - Regen state       : 22 20F2  -> (A & 7) mapowanie:
         0 OFF, 1..4 Stage1..4, 6 Warming, 7 Actively
    - Oil temp UDS      : 22 1154  -> A - 40 (°C) (fallback)

  OBD-II / PID (Service 01 xx) – zawsze na functional broadcast (nie używa g_header):
    - VBAT (napięcie)   : 01 42  -> (A*256+B)/1000 (V)
    - ECT               : 01 05  -> A - 40 (°C)
    - OIL               : 01 5C  -> A - 40 (°C) (jeśli brak -> fallback DID 22 1154)

  UWAGA: functional header wybierany automatycznie:
    - jeśli g_header <= 0x7FF -> OBD functional = 0x7DF (11-bit)
    - jeśli g_header > 0x7FF  -> OBD functional = 0x18DB33F1 (29-bit)

  ------------------------------------------------------------
  UI / Wyświetlanie:
  ------------------------------------------------------------
  DASH:
    - Góra: "DPF : <status>" + duże "XX%"
    - Pasek %: >=80% zmienia kolor na pomarańczowy (COL_WARM)
    - Siatka 4 kolumny: REG(km) / VBAT / OIL / ECT

  BURN (wypalanie):
    - Tytuł u góry jest DYNAMICZNY i pokazuje stan regeneracji (stateStr)
    - Specjalnie: stan "Actively" wyświetlany jako "BURNING"
    - Pasek u dołu jest stały czerwony

  SUCCESS:
    - "REGENERATION COMPLETED" (zielony)

  ABORT:
    - "REGENERATION ABORTED" (pomarańczowy)

  ------------------------------------------------------------
  Logika przełączania ekranów (ważne):
  ------------------------------------------------------------
  - Na ekran BURN przechodzi TYLKO, gdy stan = Actively (A&7 == 7)
  - Po wejściu na BURN działa "latch" (zatrzask) i ekran nie wraca od razu:
      * zostaje w BURN dopóki: Actively lub Warming lub burnReq==true
  - Wyjście z BURN następuje dopiero, gdy powyższe warunki nie występują
    przez REGEN_HOLD_MS (domyślnie 5000 ms)
  - Po wyjściu pokazuje:
      * SUCCESS jeśli kmSinceRegen <= REGEN_DONE_KM_MAX
      * ABORT w przeciwnym wypadku
    i trzyma ekran przez SUCCESS_SHOW_MS (domyślnie 4000 ms)

  ------------------------------------------------------------
  Konsola / Sterowanie (Serial 115200):
  ------------------------------------------------------------
  W Arduino Serial Monitor ustaw:
    - 115200 baud
    - Line ending: "Newline" (albo "Both NL & CR")

  Komendy są nieczułe na wielkość liter, ale zalecane są małe litery.

  [połączenie ble / elm]
    help / ?          - pomoc
    scan [s]          - skan BLE (domyślnie 5s), zapisuje listę
    list              - pokaż urządzenia z ostatniego scan
    use <n>           - ustaw MAC z listy (bez łączenia)
    conn <n> [save]   - ustaw MAC z listy i połącz (save = zapisz MAC + restart)
    connect           - połącz z zapisanym MAC
    disconnect        - rozłącz BLE
    status            - status + konfiguracja

  [profil auta]
    cars              - lista profili
    car?              - aktywny profil
    car <n>           - wybierz profil (ładuje ustawienia profilu)

  [konfiguracja obd/uds]
    mac / mac?        - pokaż MAC
    mac <AA:..>       - ustaw MAC
    mac save          - zapisz MAC (restart)
    header / header?  - pokaż nagłówek (CAN ID)
    header <hex>      - ustaw (np. 7e0 lub 18daf110)
    header save       - zapisz header
    pid / pid?        - pokaż PIDy (01)
    pid <vbat|ect|oil> <hex>  - ustaw PID
    pid save          - zapisz PIDy
    did / did?        - pokaż DIDy (22)
    did <soot|km|burn|state|prog|oil> <hex> - ustaw DID
    did save          - zapisz DIDy

  [ekran]
    bl <0-100>        - jasność (%)
    bl save           - zapisz jasność
    rot <0-3>         - obrót
    rot 180           - obrót o 180°
    rot save          - zapisz obrót

  [odczyt / diagnostyka]
    vbat | ect | oil  - szybki odczyt PID
    req | state       - szybki odczyt DID (regen)
    read22 <0xXXXX>   - odczyt dowolnego DID (np. read22 336a)
    vin               - odczyt VIN (OBD 09 02 / fallback UDS 22 F190)
    at <cmd>          - surowa komenda do ELM (np. at i, 01 0c, 22 33 6a)
    logio on/off      - log TX/RX
    test on/off       - tryb testowy UI (bez auta)

  [narzędzia]
    testall           - test PID/DID dla profilu
    dump              - pełny zrzut konfiguracji
    factory / factory yes - reset profilu do domyślnych

  ------------------------------------------------------------
  Zapis ustawień (NVS / Preferences):
  ------------------------------------------------------------
  - MAC trzymany w namespace "global"
  - PIDs/DIDs/HEADER/BL/ROT trzymane w namespace profilu (np. ak16, gx16, ...)
  - Po restarcie wczytuje zapisane wartości

  ------------------------------------------------------------
  Podświetlenie (BACKLIGHT):
  ------------------------------------------------------------
  - PWM na pinie TFT_BL = GPIO22 (LEDC)
  - BL_INVERT=false oznacza "większy PWM = jaśniej"
  - Jeśli na starcie widzisz "śmieci" zanim kod przejmie BL,
    to sprzętowo pomaga rezystor 47k..100k z BL do GND (dla BL_INVERT=false).

  ------------------------------------------------------------
  Uwagi praktyczne:
  ------------------------------------------------------------
  - Nie każdy ELM327 poprawnie obsługuje wszystkie PID/DID.
  - Jeśli coś zwraca NO DATA: zmień PID/DID z konsoli i zapisz.
  - PIDs (01 xx) lecą na functional broadcast, a DID (22 xxxx) lecą na g_header.
*/

// ===================== FIXED BYTE BUFFER =====================
struct ByteBuf {
  static const uint8_t CAP = 96;   // możesz dać 64 jeśli chcesz
  uint8_t n = 0;
  uint8_t b[CAP];
};



#include <TFT_eSPI.h>
#include <SPI.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>

#include <Preferences.h>
#include <math.h>

#include "opel_240.h"
#include "car_profiles.h"
#include "isotp_helpers.h"

// ===================== TFT =====================
TFT_eSPI tft = TFT_eSPI();PI();

// ===================== DEFAULT SETTINGS =====================
static const char* DEFAULT_MAC       = "00:00:00:00:00:00";
static uint32_t    DEFAULT_HEADER    = 0x7E0;
static const char* NVS_GLOBAL = "global";   // trzymamy MAC tutaj (nie miesza się z profilami)

// ===================== NVS VERSIONING =====================
// Zwiększ te numerki gdy zmieniasz klucze w NVS albo znaczenie wartości
static const uint16_t CFG_VER_GLOBAL  = 1;   // namespace "global"
static const uint16_t CFG_VER_PROFILE = 1;   // namespace profili (ak16, gx16, ...)


// ===================== SCAN CACHE (wybór po numerze) =====================
static const int SCAN_MAX = 20;     // ile pozycji max zapamiętujemy
static String g_scanMac[SCAN_MAX];
static String g_scanName[SCAN_MAX];
static int    g_scanCount = 0;

// ===================== CAR PROFILE =====================
static int g_profileIdx = 0;           // aktywny profil (index)
static String g_nvsNS = "ak16";        // aktywny namespace NVS (dla PIDs/DIDs/HEADER/BL/ROT)
static const char* g_filterLabel = "DPF";  // "DPF" albo "GPF" zależnie od profilu


// Standard OBD-II PID defaults (Service 01)
static uint8_t DEFAULT_PID_VBAT   = 0x42; // Control module voltage (A*256+B)/1000
static uint8_t DEFAULT_PID_ECT    = 0x05; // Engine coolant temp: A-40
static uint8_t DEFAULT_PID_OIL    = 0x5C; // Engine oil temp: A-40 (if supported)

// Astra K typowe DIDy:
static uint16_t    DEFAULT_DID_SOOT     = 0x336A;  // A = % (0..100)
static uint16_t    DEFAULT_DID_KM       = 0x3039;  // A*256+B (km)
static uint16_t    DEFAULT_DID_BURN     = 0x20FA;  // bit0 (REQ)
static uint16_t    DEFAULT_DID_STATE    = 0x20F2;  // Lookup(A&7)
static uint16_t    DEFAULT_DID_PROG     = 0x0000;  // opcjonalnie
static uint16_t    DEFAULT_DID_OIL_UDS  = 0x1154;  // GM Oil Temp: 22 11 54, A-40

// ===================== BLE HANDLES =====================
static BLEClient* g_client = nullptr;
static BLERemoteCharacteristic* g_charTX = nullptr; // notify
static BLERemoteCharacteristic* g_charRX = nullptr; // write

static bool g_writeWithResponse = true;


// ===================== ELM RX BUFFER (no-alloc) =====================
static portMUX_TYPE g_bufMux = portMUX_INITIALIZER_UNLOCKED;

static const size_t RX_BUF_SZ = 2048;     // możesz dać 1024..4096
static char g_rxBuf[RX_BUF_SZ];
static volatile size_t g_rxLen = 0;
static volatile bool g_prompt = false;
static volatile bool g_rxOverflow = false;

static inline void rxClearLocked() {
  g_rxLen = 0;
  g_prompt = false;
  g_rxOverflow = false;
  g_rxBuf[0] = 0;
}

// ===================== NVS =====================
Preferences g_prefs;
String   g_mac = "";
uint32_t g_header = 0;

// OBD functional request header (PID 01 xx)
static const uint32_t OBD_FUNC_11 = 0x7DF;
static const uint32_t OBD_FUNC_29 = 0x18DB33F1;

static uint32_t g_obdFuncHeader = OBD_FUNC_11;

static inline void updateObdFuncHeader() {
  g_obdFuncHeader = (g_header <= 0x7FF) ? OBD_FUNC_11 : OBD_FUNC_29;
}


// PIDs (01 xx) - konfigurowalne z konsoli
uint8_t g_pid_vbat = 0;
uint8_t g_pid_ect  = 0;
uint8_t g_pid_oil  = 0;

// DIDs (22 xxxx) - konfigurowalne z konsoli
uint16_t g_did_soot  = 0;
uint16_t g_did_km    = 0;
uint16_t g_did_burn  = 0;
uint16_t g_did_state = 0;
uint16_t g_did_prog  = 0;
uint16_t g_did_oil_uds = 0;

// ===================== OPTIONS =====================
static bool g_logIO = false;
static bool g_testMode = false;
static uint32_t g_testStartMs = 0;
static uint32_t g_pauseReconnectUntilMs = 0;

// ===================== LOG THROTTLE =====================
static uint32_t g_lastMacNotSeenLogMs = 0;
static uint32_t g_macNotSeenCount = 0;

// co ile wypisywać komunikat (ms)
static const uint32_t MAC_NOT_SEEN_LOG_EVERY_MS = 15000;  // 15s


// ===================== MUTEX =====================
static SemaphoreHandle_t g_elmMutex = nullptr;
static SemaphoreHandle_t g_tftMutex = nullptr;
static inline void tftLock()   { if (g_tftMutex) xSemaphoreTake(g_tftMutex, portMAX_DELAY); }
static inline void tftUnlock() { if (g_tftMutex) xSemaphoreGive(g_tftMutex); }

static SemaphoreHandle_t g_bleMutex = nullptr;
static inline void bleLock()   { if (g_bleMutex) xSemaphoreTake(g_bleMutex, portMAX_DELAY); }
static inline void bleUnlock() { if (g_bleMutex) xSemaphoreGive(g_bleMutex); }

// ===================== ELM AUTO-RECOVER =====================
static portMUX_TYPE g_elmStatMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t  g_elmFailStreak = 0;
static volatile bool     g_elmNeedsReinit = false;
static volatile bool     g_elmInReinit = false;
static volatile uint32_t g_elmCooldownUntilMs = 0;

static const int REGEN_DONE_SOOT_MAX = 35;

static void elmInit();
// ile kolejnych TIMEOUT (brak '>') zanim zrobimy elmInit()
static const uint8_t  ELM_FAIL_STREAK_MAX = 6;

// żeby nie robić resetów co chwilę
static const uint32_t ELM_REINIT_COOLDOWN_MS = 15000;

static inline bool elmTransportReady() {
  return (g_client && g_client->isConnected() && g_charRX && g_charTX);
}

static bool isMacStringValid(const String& s) {
  if (s.length() != 17) return false;
  for (int i=0;i<17;i++) {
    if ((i+1)%3==0) { if (s[i] != ':') return false; }
    else {
      char c = s[i];
      bool hex = (c>='0'&&c<='9')||(c>='a'&&c<='f')||(c>='A'&&c<='F');
      if (!hex) return false;
    }
  }
  return true;
}

static void ensureGlobalNvsVersion() {
  if (!g_prefs.begin(NVS_GLOBAL, false)) return;

    uint16_t v = g_prefs.getUShort("cfg_ver", 0);
    if (v != CFG_VER_GLOBAL) {
      // zachowaj MAC, bo global trzyma tylko to
      String macKeep = g_prefs.getString("mac", DEFAULT_MAC);
      uint8_t profKeep = g_prefs.getUChar("profile", 0);
      macKeep.trim(); macKeep.toUpperCase();

      g_prefs.clear();
      g_prefs.putUShort("cfg_ver", CFG_VER_GLOBAL);

    if (isMacStringValid(macKeep) && macKeep != "00:00:00:00:00:00") {
      g_prefs.putString("mac", macKeep);
    }

    if (profKeep < PROFILE_COUNT) {
      g_prefs.putUChar("profile", profKeep);
    }


      Serial.printf("[NVS] GLOBAL reset/upgrade (ver %u -> %u)\n", v, CFG_VER_GLOBAL);
    }

    // upewnij się, że cfg_ver istnieje nawet w czystym NVS
    if (g_prefs.getUShort("cfg_ver", 0) != CFG_VER_GLOBAL) {
      g_prefs.putUShort("cfg_ver", CFG_VER_GLOBAL);
    }

    g_prefs.end();
  }

static bool ensureProfileNvsVersion(const char* ns) {
  if (!g_prefs.begin(ns, false)) return false;

  uint16_t v = g_prefs.getUShort("cfg_ver", 0);
  if (v != CFG_VER_PROFILE) {
    g_prefs.clear();
    g_prefs.putUShort("cfg_ver", CFG_VER_PROFILE);
    Serial.printf("[NVS] PROFILE %s reset/upgrade (ver %u -> %u)\n", ns, v, CFG_VER_PROFILE);
  }

  // jw. – jak czysto, to wpisz wersję
  if (g_prefs.getUShort("cfg_ver", 0) != CFG_VER_PROFILE) {
    g_prefs.putUShort("cfg_ver", CFG_VER_PROFILE);
  }

  g_prefs.end();
  return true;
}


static inline void elmNotePrompt(bool gotPrompt) {
  portENTER_CRITICAL(&g_elmStatMux);

  if (g_elmInReinit) { // nie licz w trakcie reinit
    portEXIT_CRITICAL(&g_elmStatMux);
    return;
  }

  if (gotPrompt) {
    g_elmFailStreak = 0;
    g_elmNeedsReinit = false;
  } else {
    if (g_elmFailStreak < 250) g_elmFailStreak++;
    if (g_elmFailStreak >= ELM_FAIL_STREAK_MAX) g_elmNeedsReinit = true;
  }

  portEXIT_CRITICAL(&g_elmStatMux);
}

 static void elmAutoRecoverTick(uint32_t now) {
  if (!elmTransportReady()) return;

  bool doIt = false;

  portENTER_CRITICAL(&g_elmStatMux);
  if (!g_elmInReinit && g_elmNeedsReinit && now >= g_elmCooldownUntilMs) {
    doIt = true;
    g_elmNeedsReinit = false;
    g_elmFailStreak = 0;
    g_elmInReinit = true;
    g_elmCooldownUntilMs = now + ELM_REINIT_COOLDOWN_MS;
  }
  portEXIT_CRITICAL(&g_elmStatMux);

  if (!doIt) return;

  Serial.println("[ELM] Auto-reinit (timeout streak)");
  elmInit();          // reinit ELM

  portENTER_CRITICAL(&g_elmStatMux);
  g_elmInReinit = false;
  portEXIT_CRITICAL(&g_elmStatMux);
}




// ===================== COLORS =====================
#define COL_BG        0x0000
#define COL_TEXT      0xFFFF
#define COL_TITLE     0xFEC0
#define COL_BAR_BG    0x31A6
#define COL_BAR_FG    0x07E0
#define COL_INFO      0x04FF
#define COL_WARM      0xFD20
#define COL_GRID      0x18E3
#define COL_RED       0xF800
#define COL_GREY      0x8410

// ===================== TEMP VALIDATION =====================
static const float TEMP_NODATA = -9999.0f;
static inline bool tempValid(float t) { return (t > -50.0f && t < 260.0f); }

// ===================== HEADER CACHE (ważne dla 01 xx) =====================
static uint32_t g_lastHdr = 0xFFFFFFFF;

// ===================== CONSOLE =====================
static String sLine;

// ===================== UI MODES =====================
enum UiMode : uint8_t { UI_DASH = 0, UI_BURN = 1, UI_SUCCESS = 2, UI_ABORT = 3 };
static UiMode g_mode = UI_DASH;

static TaskHandle_t g_task = nullptr;

// ===================== ROTATION =====================
static uint8_t g_rotation = 1;

// ===================== DASH LAYOUT (320x172) =====================
static bool g_dashStaticDrawn = false;
static int   g_dash_lastPct  = -999;
static int   g_dash_lastKm   = -999;
static float g_dash_lastVbat = -999;
static float g_dash_lastOil  = -999;
static float g_dash_lastEct  = -999;
static String g_dash_lastStatus = "";

// Ogólny margines boczny dla elementów na DASH (lewy/prawy)
static const int D_PAD   = 10;

// Pozycja paska procentowego (DPF %) w pionie:
// - D_BAR_Y = górna krawędź paska (im mniejsze, tym wyżej)
static const int D_BAR_Y = 46;

// Wysokość paska procentowego (DPF %)
static const int D_BAR_H = 14;

// Gdzie zaczyna się (w pionie) sekcja "grid" na dole (REG/VBAT/OIL/ECT)
// - im mniejsze, tym wyżej podjedzie cała siatka
static const int D_GRID_TOP = 72;

// Wysokość siatki (obszaru 4 kolumn)
static const int D_GRID_H   = 96;

// Offsety Y wewnątrz każdej kolumny siatki (liczone od D_GRID_TOP):
// pozycja napisu etykiety (REG/VBAT/OIL/ECT)
static const int D_LABEL_Y_OFF = 6;

// pozycja wartości (np. 123, 14.2, 90, 80)
static const int D_VAL_Y_OFF   = 36;

// pozycja jednostki (km, V, C)
static const int D_UNIT_Y_OFF  = 64;

// Pozycja napisu statusu DPF u góry po lewej: "DPF : ---" / "DPF : REQ" / itd.
// - D_STAT_X: lewo/prawo (większe = bardziej w prawo)
// - D_STAT_Y: góra/dół   (mniejsze = wyżej)
static const int D_STAT_X = 10;
static const int D_STAT_Y = 11;

// Rezerwowany "pas" po prawej na duże "XX%"
// (W = szerokość obszaru na procent, H = wysokość górnej sekcji)
static const int D_PCT_W = 120;
static const int D_PCT_H = D_BAR_Y;   // wysokość górnej sekcji do wysokości paska

// Pozycja Y dla dużego tekstu procentów "XX%" po prawej
// - mniejsze = wyżej, większe = niżej
static const int D_PCT_TEXT_Y = 5;

// ===================== BURN LAYOUT =====================
static bool  g_burnStaticDrawn = false;
static int   g_burn_lastPct = -999;
static String g_burn_lastTitle = "";

// ===================== REGEN SESSION (LATCH) =====================
// ===================== REGEN SESSION (LATCH) =====================
static bool     g_regenLatched = false;
static uint32_t g_regenLastSeenMs = 0;

static uint32_t g_successUntilMs = 0;
static bool     g_successStaticDrawn = false;

static uint32_t g_abortUntilMs = 0;
static bool     g_abortStaticDrawn = false;

static const uint32_t REGEN_HOLD_MS   = 5000;
static const uint32_t SUCCESS_SHOW_MS = 4000;

// jeśli po zakończeniu regen kmSinceRegen <= ten próg -> uznajemy, że zakończone
static const int REGEN_DONE_KM_MAX = 3;   // ustaw np. 1..5 jak chcesz

static inline void clearPostScreens() {
  g_successUntilMs = 0;
  g_abortUntilMs = 0;
  g_successStaticDrawn = false;
  g_abortStaticDrawn = false;
}

static inline void startSuccess(uint32_t now) {
  g_successUntilMs = now + SUCCESS_SHOW_MS;
  g_abortUntilMs = 0;
  g_successStaticDrawn = false;
  g_abortStaticDrawn = false;
}

static inline void startAbort(uint32_t now) {
  g_abortUntilMs = now + SUCCESS_SHOW_MS;
  g_successUntilMs = 0;
  g_abortStaticDrawn = false;
  g_successStaticDrawn = false;
}


// ===================== BACKLIGHT (PWM) =====================
#define TFT_BL 22
static const bool BL_INVERT = false;

static const int  BL_PWM_CH   = 0;
static const int  BL_PWM_FREQ = 5000;
static const int  BL_PWM_RES  = 8;

static uint8_t g_blPct = 80;

static inline uint8_t clampU8(int v, int lo, int hi) {
  if (v < lo) return (uint8_t)lo;
  if (v > hi) return (uint8_t)hi;
  return (uint8_t)v;
}

static void setBacklightPct(int pct) {
  g_blPct = clampU8(pct, 0, 100);
  int duty = (g_blPct * 255) / 100;
  if (BL_INVERT) duty = 255 - duty;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(TFT_BL, duty);
#else
  ledcWrite(BL_PWM_CH, duty);
#endif
}

static void initBacklightPwm() {
  pinMode(TFT_BL, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(TFT_BL, BL_PWM_FREQ, BL_PWM_RES, BL_PWM_CH);
#else
  ledcSetup(BL_PWM_CH, BL_PWM_FREQ, BL_PWM_RES);
  ledcAttachPin(TFT_BL, BL_PWM_CH);
#endif

  setBacklightPct(g_blPct);
}

// ===================== SPLASH (non-blocking) =====================
static volatile bool     g_splashActive  = false;
static volatile uint32_t g_splashUntilMs = 0;

static inline bool splashOn(uint32_t now) {
  return g_splashActive && now < g_splashUntilMs;
}

static inline void startSplash(uint32_t ms) {
  g_splashActive  = true;
  g_splashUntilMs = millis() + ms;
}

static inline void stopSplashIfDone(uint32_t now) {
  if (g_splashActive && now >= g_splashUntilMs) {
    g_splashActive = false;

    // po zejściu z logo wymuś pełny redraw
    g_dashStaticDrawn    = false;
    g_burnStaticDrawn    = false;
    g_successStaticDrawn = false;
    g_abortStaticDrawn   = false;
  }
}


// ===================== HELPERS =====================
// Ramka (zostaw malutki inset, żeby nie wchodziła w tekst)
static const int FRAME_PAD = 4;
static const int FRAME_RAD = 10;

static inline bool isConnectedOBD() {
  return (g_client && g_client->isConnected() && g_charRX && g_charTX);
}


static inline void drawFrame(uint16_t col) {
  int w = tft.width()  - FRAME_PAD * 2 - 1;
  int h = tft.height() - FRAME_PAD * 2 - 1;
  if (w > 0 && h > 0) tft.drawRoundRect(FRAME_PAD, FRAME_PAD, w, h, FRAME_RAD, col);
}

// ====== TEXT FIT HELPERS ======
static void splitTwoLines(const String& s, String& a, String& b) {
  String x = s; x.trim();
  int best = -1;
  int mid = x.length() / 2;

  // szukaj spacji najbliżej środka
  for (int d = 0; d < mid; d++) {
    int i1 = mid - d;
    int i2 = mid + d;
    if (i1 > 0 && i1 < (int)x.length() && x[i1] == ' ') { best = i1; break; }
    if (i2 > 0 && i2 < (int)x.length() && x[i2] == ' ') { best = i2; break; }
  }

  if (best < 0) { // brak spacji -> tniemy na pół
    best = mid;
  }

  a = x.substring(0, best); a.trim();
  b = x.substring(best);    b.trim();
}

static int pickFontToFit(const String& s, int maxW) {
  // preferuj większą czcionkę, ale tylko jeśli się mieści
  if (tft.textWidth(s, 4) <= maxW) return 4;
  return 2; // font 2 prawie zawsze się mieści na 172px
}

static void drawCenteredLineFit(const String& s, int cx, int y, int maxW,
                                uint16_t fg, uint16_t bg) {
  int f = pickFontToFit(s, maxW);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(fg, bg);
  tft.setTextFont(f);

  // jeśli nawet font2 nie mieści -> łamiemy na 2 linie
  if (tft.textWidth(s, f) > maxW) {
    String a, b;
    splitTwoLines(s, a, b);

    tft.setTextFont(2);
    int dy = 12;
    tft.drawString(a, cx, y - dy);
    tft.drawString(b, cx, y + dy);
    return;
  }

  tft.drawString(s, cx, y);
}

static int pickFontToFit(const String& s, int maxW, int fontMax, int fontMin) {
  for (int f = fontMax; f >= fontMin; f--) {
    if (tft.textWidth(s, f) <= maxW) return f;
  }
  return fontMin;
}

static void splitToTwoLines(const String& s, String& a, String& b) {
  String x = s; x.trim();
  int best = -1;
  int mid = x.length() / 2;

  // znajdź spację najbliżej środka
  for (int d = 0; d < mid; d++) {
    int i1 = mid - d;
    int i2 = mid + d;
    if (i1 > 0 && x[i1] == ' ') { best = i1; break; }
    if (i2 < (int)x.length() && x[i2] == ' ') { best = i2; break; }
  }

  if (best < 0) { // brak spacji -> tniemy "na sztywno"
    a = x.substring(0, mid);
    b = x.substring(mid);
  } else {
    a = x.substring(0, best);
    b = x.substring(best + 1);
  }
  a.trim(); b.trim();
}

// rysuje 1 linię wyśrodkowaną, dobiera font żeby weszło
static void drawCenterFit1(const String& s, int cx, int y, int maxW, int fontMax, int fontMin, uint16_t fg, uint16_t bg) {
  int f = pickFontToFit(s, maxW, fontMax, fontMin);
  tft.setTextDatum(TC_DATUM);
  tft.setTextFont(f);
  tft.setTextColor(fg, bg);
  tft.drawString(s, cx, y);
}

// rysuje tekst, a jeśli musiałby spaść do małego fontu -> łamie na 2 linie i powiększa
static void drawCenterFit2(const String& s, int cx, int y1, int y2, int maxW,
                           int fontMax, int fontMin, uint16_t fg, uint16_t bg)
{
  int f = pickFontToFit(s, maxW, fontMax, fontMin);
  bool fitsOneLine = (tft.textWidth(s, f) <= maxW);

  // jeśli jedyna opcja to mały font (fontMin) i jest spacja -> lepiej złamać na 2 linie
  bool preferSplit = (f <= fontMin && s.indexOf(' ') >= 0);

  if (fitsOneLine && !preferSplit) {
    tft.setTextDatum(TC_DATUM);
    tft.setTextFont(f);
    tft.setTextColor(fg, bg);
    tft.drawString(s, cx, y1);
    return;
  }

  // 2 linie
  String a, b;
  splitToTwoLines(s, a, b);

  int fa = pickFontToFit(a, maxW, fontMax, fontMin);
  int fb = pickFontToFit(b, maxW, fontMax, fontMin);

  // jedna wielkość dla obu linii (czytelniej)
  int fu = (fa < fb) ? fa : fb;

  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(fg, bg);

  tft.setTextFont(fu);
  tft.drawString(a, cx, y1);

  tft.setTextFont(fu);
  tft.drawString(b, cx, y2);
}


static void showMsg(const char* line1, const char* line2 = nullptr, uint16_t c1 = COL_TITLE, uint16_t c2 = COL_TEXT) {
  if (splashOn(millis())) return;

  tftLock();
  tft.fillScreen(COL_BG);

  int pad = 10;
  int w = tft.width()  - pad*2;   // -1 jak w ramce
  int h = tft.height() - pad*2;

  tft.drawRoundRect(pad, pad, w, h, 12, c2);

  int cx = tft.width()/2;
  int maxW = w - 16; // trochę marginesu w środku

  // Y wyliczane tak, żeby działało i w LANDSCAPE i w PORTRAIT
  int y1 = tft.height()/2 - 18;
  int y2 = tft.height()/2 + 18;

  // LINE1: font 4->2
  drawCenterFit2(String(line1 ? line1 : ""), cx, y1, y1 + 18, maxW, 4, 2, c1, COL_BG);

  // LINE2: font 2->1
  if (line2 && *line2) {
    drawCenterFit2(String(line2), cx, y2, y2 + 16, maxW, 2, 1, c2, COL_BG);
  }

  tftUnlock();
}



static void drawOpelSplashZoom(uint32_t ms = 4000, int zoomPct = 160, int panX = 0, int panY = 0) {
  const int W = tft.width();
  const int H = tft.height();

  const int cxD = W / 2;
  const int cyD = H / 2;

  const int cxS = (OPEL_W / 2) + panX;
  const int cyS = (OPEL_H / 2) + panY;

  static int16_t mapX[320];
  static int16_t mapY[320];

  for (int x = 0; x < W; x++) mapX[x] = cxS + ((x - cxD) * 100) / zoomPct;
  for (int y = 0; y < H; y++) mapY[y] = cyS + ((y - cyD) * 100) / zoomPct;

  static uint16_t line[320];

  tftLock();
  tft.fillScreen(COL_BG);

  for (int y = 0; y < H; y++) {
    const int sy = mapY[y];
    for (int x = 0; x < W; x++) {
      const int sx = mapX[x];
      if ((unsigned)sx < OPEL_W && (unsigned)sy < OPEL_H) line[x] = pgm_read_word(&opel[sy * OPEL_W + sx]);
      else line[x] = COL_BG;
    }
    tft.pushImage(0, y, W, 1, line);
  }
  tftUnlock();
  if (ms) delay(ms);
}

static String fmtFloatPL_noUnit(float v, uint8_t prec=1) {
  if (v < 0) return String("--");
  char buf[24]; dtostrf(v, 0, prec, buf);
  String s(buf);
  s.replace('.', ',');
  return s;
}


static inline bool isHexChar(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

// Tokenizuje dowolne "hex tokeny" z linii (ignoruje resztę) do ByteBuf.
// Działa również gdy linia ma np. "7E8 10 14 62 33 6A 55" albo "0: 62 33 6A 55".
static bool tokenizeHexToBuf(const char* s, ByteBuf& out) {
  out.n = 0;
  if (!s) return false;

  while (*s) {
    // pomiń spacje i separatory
    while (*s && (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')) s++;

    // jeśli nie zaczyna się od hex, pomiń do następnej spacji
    if (!*s) break;
    if (!isHexChar(*s)) {
      while (*s && *s != ' ' && *s != '\t') s++;
      continue;
    }

    // zbierz token (max np. 8 znaków, ale i tak walidujemy zakres 0..255)
    char tok[10];
    int k = 0;
    while (*s && *s != ' ' && *s != '\t' && k < 9) {
      tok[k++] = *s++;
    }
    tok[k] = 0;

    // sprawdź, czy token jest czysto hex
    bool ok = (k > 0);
    for (int i = 0; i < k; i++) {
      if (!isHexChar(tok[i])) { ok = false; break; }
    }
    if (!ok) continue;

    char* e = nullptr;
    long v = strtol(tok, &e, 16);
    if (e && *e == 0 && v >= 0 && v <= 255) {
      if (out.n < ByteBuf::CAP) out.b[out.n++] = (uint8_t)v;
      else return (out.n > 0); // utnij, ale uznaj że coś mamy
    }
  }
  return out.n > 0;
}


// ===================== BLE notify =====================
static void notifyCB(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  portENTER_CRITICAL(&g_bufMux);

  for (size_t i = 0; i < len; ++i) {
    char c = (char)data[i];
    if (c == '\r') c = '\n';


    if (c == '>') {            // prompt = koniec odpowiedzi
      g_prompt = true;
      continue;                // nie zapisuj '>'
    }

    // dopisz znak jeśli jest miejsce
    if (g_rxLen < RX_BUF_SZ - 1) {
      g_rxBuf[g_rxLen++] = c;
      g_rxBuf[g_rxLen] = 0;    // utrzymuj NUL-terminator
    } else {
      g_rxOverflow = true;     // przepełnienie -> utnij, ale nie crashuj
    }
  }

  portEXIT_CRITICAL(&g_bufMux);
}


// ===================== ELM SEND/WAIT =====================
static void sendCmdLocked(const char* sNoCR) {
  if (!g_charRX) return;
  String s = String(sNoCR);
  if (!s.endsWith("\r")) s += "\r";

  bool ok = g_charRX->writeValue((uint8_t*)s.c_str(), s.length(), g_writeWithResponse);

  // fallback: jak nie poszło, spróbuj drugim trybem
  if (!ok) {
    ok = g_charRX->writeValue((uint8_t*)s.c_str(), s.length(), !g_writeWithResponse);
    if (ok) g_writeWithResponse = !g_writeWithResponse;
  }

  if (g_logIO) { Serial.print("[TX] "); Serial.print(s); }
}


static bool sendELMAndWait(const char* s, String& out, uint32_t timeout_ms = 900) {
  if (!g_elmMutex) return false;
  xSemaphoreTake(g_elmMutex, portMAX_DELAY);

  portENTER_CRITICAL(&g_bufMux);
  rxClearLocked();
  portEXIT_CRITICAL(&g_bufMux);

  sendCmdLocked(s);

  bool gotPrompt = false;

  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    bool ready;
    portENTER_CRITICAL(&g_bufMux);
    ready = g_prompt;
    portEXIT_CRITICAL(&g_bufMux);

    if (ready) { gotPrompt = true; break; }
    vTaskDelay(pdMS_TO_TICKS(1));
  }


  // skopiuj atomowo do lokalnego bufora (żeby nie trzymać locka długo)
  char tmp[RX_BUF_SZ];
  bool overflow = false;

  portENTER_CRITICAL(&g_bufMux);
  size_t n = g_rxLen;
  if (n >= RX_BUF_SZ) n = RX_BUF_SZ - 1;
  memcpy(tmp, g_rxBuf, n);
  tmp[n] = 0;
  overflow = g_rxOverflow;
  portEXIT_CRITICAL(&g_bufMux);

  out = String(tmp);
  if (overflow) {
    // przy overflow możesz uznać odpowiedź za niepewną
    // zostawiam informacyjnie w logu:
    if (g_logIO) Serial.println("[RX] WARNING: overflow (truncated)");
  }


  xSemaphoreGive(g_elmMutex);

  out.trim();
  // watchdog: liczymy tylko brak prompta (timeout / zwiecha ELM)
  if (elmTransportReady()) elmNotePrompt(gotPrompt);

  if (g_logIO) { Serial.print("[RX] "); Serial.println(out); }
  if (!out.length()) return false;

  String up = out;
  up.toUpperCase();

  if (up.indexOf("NO DATA") >= 0 || up.indexOf("NODATA") >= 0) return false;
  if (up.indexOf("ERROR") >= 0) return false;
  if (up.indexOf("STOPPED") >= 0) return false;
  if (up.indexOf("UNABLE TO CONNECT") >= 0) return false;
  if (up.indexOf("BUS INIT") >= 0);
  if (up.indexOf("CAN ERROR") >= 0) return false;
  // częsty sygnał nieznanej komendy (czasem samo "?")
  if (up == "?" || up.endsWith("\n?")) return false;

  return true;

}

// ===================== HEADER SET =====================
static bool elmSetHeader(uint32_t hdr) {
  if (g_lastHdr == hdr) return true;

  String out;

  // 11-bit
  if (hdr <= 0x7FF) {
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "ATSH%03lX", (unsigned long)hdr);
    if (sendELMAndWait(cmd, out, 600)) { g_lastHdr = hdr; return true; }

    snprintf(cmd, sizeof(cmd), "AT SH %03lX", (unsigned long)hdr);
    if (sendELMAndWait(cmd, out, 600)) { g_lastHdr = hdr; return true; }

    return false;
  }

  // 29-bit
  char cmd[20];
  snprintf(cmd, sizeof(cmd), "ATSH%08lX", (unsigned long)hdr);
  if (sendELMAndWait(cmd, out, 600)) { g_lastHdr = hdr; return true; }

  snprintf(cmd, sizeof(cmd), "AT SH %08lX", (unsigned long)hdr);
  if (sendELMAndWait(cmd, out, 600)) { g_lastHdr = hdr; return true; }

  return false;
}


// ===================== ISO-TP reassembly (UDS 22) =====================

static bool extract62DIDFromPayload(const uint8_t* p, int n, uint16_t did, ByteBuf& out) {
  for (int i = 0; i + 2 < n; i++) {
    if (p[i] == 0x62) {
      uint16_t got = (uint16_t)((p[i + 1] << 8) | p[i + 2]);
      if (got == did) {
        int cn = n - i;
        if (cn > ByteBuf::CAP) cn = ByteBuf::CAP;
        out.n = (uint8_t)cn;
        memcpy(out.b, p + i, out.n);
        return true;
      }
    }
  }
  return false;
}

// Próbuje złożyć ISO-TP (0x10.. + 0x21..), albo obsłużyć single-frame (0x0L ...).
static bool findIsoTp62DID(const String& resp, uint16_t did, ByteBuf& out) {
  int start = 0;
  ByteBuf tmp;
  IsoTpBuf pay;

  bool assembling = false;
  int remaining = 0;
  uint8_t nextSeq = 1;

  while (start < (int)resp.length()) {
    int end = resp.indexOf('\n', start);
    if (end < 0) end = resp.length();

    String line = resp.substring(start, end);
    line.trim();

    if (tokenizeHexToBuf(line.c_str(), tmp) && tmp.n >= 1) {
      for (int pass = 0; pass < 2; pass++) {
        uint8_t pci = tmp.b[0];

        if (!assembling) {
          // Single Frame: 0x0L
          if ((pci & 0xF0) == 0x00) {
            int len = pci & 0x0F;
            if (len > 0 && tmp.n >= (1 + len)) {
              isoClear(pay);
              isoAppend(pay, &tmp.b[1], len);
              if (extract62DIDFromPayload(pay.b, (int)pay.n, did, out)) return true;
            }
          }
          // First Frame: 0x10 LL
          else if ((pci & 0xF0) == 0x10 && tmp.n >= 2) {
            int totalLen = ((pci & 0x0F) << 8) | tmp.b[1];
            isoClear(pay);

            int take = (int)tmp.n - 2;
            if (take < 0) take = 0;
            if (take > totalLen) take = totalLen;

            isoAppend(pay, &tmp.b[2], take);
            remaining = totalLen - take;

            assembling = true;
            nextSeq = 1;

            if (remaining <= 0) {
              assembling = false;
              if (extract62DIDFromPayload(pay.b, (int)pay.n, did, out)) return true;
            }
          }
          break;
        } else {
          // Consecutive: 0x2N
          if ((pci & 0xF0) == 0x20) {
            uint8_t seq = pci & 0x0F;
            if (seq != (nextSeq & 0x0F)) { assembling = false; continue; }

            nextSeq++;

            int take = (int)tmp.n - 1;
            if (take < 0) take = 0;
            if (take > remaining) take = remaining;

            isoAppend(pay, &tmp.b[1], take);
            remaining -= take;

            if (remaining <= 0) {
              assembling = false;
              if (extract62DIDFromPayload(pay.b, (int)pay.n, did, out)) return true;
            }
            break;
          } else {
            assembling = false;
            continue; // pass=1 spróbuje potraktować to jako nowy start
          }
        }
      }
    }

    start = end + 1;
  }

  return false;
}



// ===================== UDS 0x22 helpers (no vector) =====================
// Szuka w odpowiedzi linii, która zawiera 0x62 + DID (w dowolnym miejscu).
// Jeśli znajdzie, przesuwa wynik tak, żeby out.b[0]==0x62, out.b[1..2]==DID.
static bool findBytesFor62DID_Simple(const String& resp, uint16_t did, ByteBuf& out) {
  int start = 0;
  ByteBuf tmp;

  out.n = 0;
  bool found = false;

  while (start < (int)resp.length()) {
    int end = resp.indexOf('\n', start);
    if (end < 0) end = resp.length();

    String line = resp.substring(start, end);
    line.trim();

    if (tokenizeHexToBuf(line.c_str(), tmp)) {

      if (!found) {
        // szukaj 62 + DID w tej linii
        for (uint8_t i = 0; i + 2 < tmp.n; i++) {
          if (tmp.b[i] == 0x62) {
            uint16_t got = (uint16_t)((tmp.b[i + 1] << 8) | tmp.b[i + 2]);
            if (got == did) {
              // skopiuj od 0x62 do końca tej linii
              uint8_t take = tmp.n - i;
              if (take > ByteBuf::CAP) take = ByteBuf::CAP;
              memcpy(out.b, &tmp.b[i], take);
              out.n = take;
              found = true;
              break;
            }
          }
        }
      } else {
        // już znaleźliśmy 62 DID -> doklejaj kolejne linie z samymi danymi
        uint8_t room = (out.n < ByteBuf::CAP) ? (ByteBuf::CAP - out.n) : 0;
        if (room == 0) return true;

        uint8_t take = tmp.n;
        if (take > room) take = room;

        memcpy(out.b + out.n, tmp.b, take);
        out.n += take;

        if (out.n >= ByteBuf::CAP) return true;
      }
    }

    start = end + 1;
  }

  return found;
}


static bool findBytesFor62DID(const String& resp, uint16_t did, ByteBuf& out) {
  if (findIsoTp62DID(resp, did, out)) return true;          // ISO-TP first
  return findBytesFor62DID_Simple(resp, did, out);          // fallback
}

static bool query22_bytes(uint16_t did, ByteBuf& outBytes, uint32_t timeout_ms = 900) {
  outBytes.n = 0;
  if (!elmSetHeader(g_header)) return false;

  char cmd[16];
  snprintf(cmd, sizeof(cmd), "22 %02X %02X", (did >> 8) & 0xFF, did & 0xFF);

  String resp;
  if (!sendELMAndWait(cmd, resp, timeout_ms)) return false;

  return findBytesFor62DID(resp, did, outBytes);
}

static bool query22_u16(uint16_t did, uint16_t& value, uint32_t timeout_ms = 900) {
  value = 0;
  ByteBuf b;
  if (!query22_bytes(did, b, timeout_ms)) return false;
  if (b.n < 5) return false; // 62 DID(2) + A + B
  value = (uint16_t)((b.b[3] << 8) | b.b[4]);
  return true;
}

static bool query22_u8(uint16_t did, uint8_t& A, uint32_t timeout_ms = 900) {
  A = 0;
  ByteBuf b;
  if (!query22_bytes(did, b, timeout_ms)) return false;
  if (b.n < 4) return false; // 62 DID(2) + A
  A = b.b[3];
  return true;
}


// ===================== Standard OBD helpers (no vector) =====================
// Szuka 0x41 + PID w dowolnym miejscu w linii i zwraca DATA (czyli bajty po PID).
static bool findDataFor41PID(const String& resp, uint8_t pid, ByteBuf& dataOut) {
  int start = 0;
  ByteBuf tmp;
  dataOut.n = 0;

  while (start < (int)resp.length()) {
    int end = resp.indexOf('\n', start);
    if (end < 0) end = resp.length();

    String line = resp.substring(start, end);
    line.trim();

    if (tokenizeHexToBuf(line.c_str(), tmp)) {
      for (uint8_t i = 0; i + 1 < tmp.n; i++) {
        if (tmp.b[i] == 0x41 && tmp.b[i + 1] == pid) {
          uint8_t dn = tmp.n - (i + 2);
          if (dn > ByteBuf::CAP) dn = ByteBuf::CAP;
          dataOut.n = dn;
          memcpy(dataOut.b, &tmp.b[i + 2], dn);
          return true;
        }
      }
    }

    start = end + 1;
  }
  return false;
}

static bool query01_u16(uint8_t pid, uint16_t& value, uint32_t timeout_ms = 900) {
  value = 0;

  if (!elmSetHeader(g_obdFuncHeader)) return false;


  char cmd[12];
  snprintf(cmd, sizeof(cmd), "01 %02X", pid);

  String resp;
  if (!sendELMAndWait(cmd, resp, timeout_ms)) return false;

  ByteBuf data;
  if (!findDataFor41PID(resp, pid, data)) return false;
  if (data.n < 2) return false;

  value = (uint16_t)((data.b[0] << 8) | data.b[1]);
  return true;
}

static bool query01_u8(uint8_t pid, uint8_t& A, uint32_t timeout_ms = 900) {
  A = 0;

  if (!elmSetHeader(g_obdFuncHeader)) return false;


  char cmd[12];
  snprintf(cmd, sizeof(cmd), "01 %02X", pid);

  String resp;
  if (!sendELMAndWait(cmd, resp, timeout_ms)) return false;

  ByteBuf data;
  if (!findDataFor41PID(resp, pid, data)) return false;
  if (data.n < 1) return false;

  A = data.b[0];
  return true;
}

// ===================== VIN helpers =====================

static bool parseVinFrom0902(const String& resp, String& vinOut) {
  vinOut = "";
  int start = 0;
  ByteBuf tmp;

  bool started = false; // już znaleźliśmy 49 02 (albo zaczęliśmy składać VIN)

  while (start < (int)resp.length() && vinOut.length() < 17) {
    int end = resp.indexOf('\n', start);
    if (end < 0) end = resp.length();

    String line = resp.substring(start, end);
    line.trim();

    if (tokenizeHexToBuf(line.c_str(), tmp) && tmp.n > 0) {

      bool hadHeaderThisLine = false;

      // 1) standard: linia zawiera 49 02 xx <ASCII...>
      for (uint8_t i = 0; i + 2 < tmp.n && vinOut.length() < 17; i++) {
        if (tmp.b[i] == 0x49 && tmp.b[i + 1] == 0x02) {
          hadHeaderThisLine = true;
          started = true;

          // pomijamy: 49 02 <counter>
          for (uint8_t j = i + 3; j < tmp.n && vinOut.length() < 17; j++) {
            char c = (char)tmp.b[j];
            if (c >= 32 && c <= 126 && c != ' ') vinOut += c;
          }
        }
      }

      // 2) jeśli VIN jest w ISO-TP ramkach (10.. / 21.. / 22..)
      // albo ELM daje kontynuacje bez 49 02,
      // to po "started" dokładamy ASCII z kolejnych linii.
      if (started && !hadHeaderThisLine && vinOut.length() < 17) {
        uint8_t from = 0;

        // jeśli wygląda na ISO-TP PCI:
        uint8_t b0 = tmp.b[0];
        if ((b0 & 0xF0) == 0x20) {
          // Consecutive Frame: 21/22/23... -> pomiń PCI
          from = 1;
        } else if ((b0 & 0xF0) == 0x10 && tmp.n >= 2) {
          // First Frame: 10 LL -> pomiń 2 bajty PCI
          from = 2;
        } else if ((b0 & 0xF0) == 0x00) {
          // Single Frame: 0L -> pomiń PCI
          from = 1;
        }

        for (uint8_t j = from; j < tmp.n && vinOut.length() < 17; j++) {
          char c = (char)tmp.b[j];
          if (c >= 32 && c <= 126 && c != ' ') vinOut += c;
        }
      }
    }

    start = end + 1;
  }

  if (vinOut.length() > 17) vinOut = vinOut.substring(0, 17);
  vinOut.trim();
  return (vinOut.length() == 17);
}


// próba VIN po OBD-II: 09 02 (standard)
static bool queryVin_0902(String& vinOut) {
  vinOut = "";
  String resp;

  // 1) 11-bit functional 7DF
  if (elmSetHeader(OBD_FUNC_11) && sendELMAndWait("09 02", resp, 2500)) {
    if (parseVinFrom0902(resp, vinOut)) return true;
  }

  // 2) 29-bit functional 18DB33F1 (część aut / adapterów)
  resp = "";
  if (elmSetHeader(OBD_FUNC_29) && sendELMAndWait("09 02", resp, 3000)) {
    if (parseVinFrom0902(resp, vinOut)) return true;
  }

  return false;
}

// fallback VIN po UDS: 22 F190 (bardzo częste w ECU)
static bool queryVin_F190(String& vinOut) {
  vinOut = "";
  ByteBuf b;
  if (!query22_bytes(0xF190, b, 2500)) return false;
  if (b.n < 4) return false; // 62 F1 90 ...

  // ASCII od b[3]
  for (uint8_t i = 3; i < b.n && vinOut.length() < 17; i++) {
    char c = (char)b.b[i];
    if (c >= 32 && c <= 126 && c != ' ') vinOut += c;
  }
  vinOut.trim();
  return (vinOut.length() == 17);
}

// jedna funkcja "publiczna" do komendy vin
static bool readVIN(String& vinOut) {
  vinOut = "";

  // najpierw standard OBD
  if (queryVin_0902(vinOut)) {
    // przywróć header do UDS (żeby po VIN nie zostało 7DF)
    elmSetHeader(g_header);
    return true;
  }

  // potem fallback UDS F190
  if (queryVin_F190(vinOut)) {
    return true;
  }

  return false;
}



// ===================== Standard OBD (PIDs) =====================
// VBAT (01 xx): (A*256+B)/1000
static bool queryVBAT(float& volts) {
  volts = -1.f;
  uint16_t raw;
  if (!query01_u16(g_pid_vbat, raw, 800)) return false;
  volts = raw / 1000.0f;
  return true;
}

// ECT (01 xx): A-40
static bool queryECT(float& tempC) {
  tempC = TEMP_NODATA;
  uint8_t A;
  if (!query01_u8(g_pid_ect, A, 800)) return false;
  tempC = (float)A - 40.0f;
  return true;
}

// OIL TEMP (01 xx): A - 40
static bool queryOIL_01(float& tempC) {
  tempC = TEMP_NODATA;
  uint8_t A;
  if (!query01_u8(g_pid_oil, A, 900)) return false;
  tempC = (float)A - 40.0f;
  return true;
}

// OIL: PID fallback UDS 22 1154
static bool queryOIL(float& tempC) {
  tempC = TEMP_NODATA;

  if (queryOIL_01(tempC)) return true;

  if (g_did_oil_uds != 0) {
    uint8_t A = 0;
    if (query22_u8(g_did_oil_uds, A, 900)) {
      tempC = (float)A - 40.0f;
      return true;
    }
  }
  return false;
}

// ===================== Astra K formulas =====================

// static int sootPercentFrom_22336A(const ByteBuf& b) {
//   if (b.n < 4) return -1;
//   int A = (int)b.b[3];
//   if (A < 0) A = 0;
//   if (A > 100) A = 100;
//   return A;
// }

static int sootPercentFrom_22336A(const ByteBuf& b) {
  if (b.n < 4) return -1;
  int A = (int)b.b[3];

  // AUTO: jeśli wygląda jak 0..255 -> przelicz na %
  if (A > 100) {
    A = (A * 100 + 127) / 255;   // zaokrąglenie
  }

  if (A < 0) A = 0;
  if (A > 100) A = 100;
  return A;
}


static bool burnReqFrom_2220FA(const ByteBuf& b) {
  if (b.n < 4) return false;
  return (b.b[3] & 0x01) != 0;
}


static const char* regenStateFrom_2220F2(uint8_t A) {
  uint8_t s = A & 0x07;
  switch (s) {
    case 0: return "OFF";
    case 1: return "Stage1";
    case 2: return "Stage2";
    case 3: return "Stage3";
    case 4: return "Stage4";
    case 6: return "Warming";
    case 7: return "Actively";
    default: return "Unknown";
  }
}

// ===================== DASH LAYOUT PORTRAIT (172x320) =====================
// Działa gdy tft.width() < tft.height()  (czyli ekran w pionie)

static const int P_PAD    = 8;    // marginesy

// Pasek statusu na górze
static const int P_STAT_H = 30;   // wysokość obszaru statusu
static const int P_STAT_Y = 8;    // y dla tekstu (TC_DATUM -> y = top)

// Duży procent
static const int P_PCT_Y  = 40;   // y (top) dla dużego "XX%"
static const int P_PCT_H  = 62;   // wysokość obszaru procentu (czyszczenie tła)

// Pasek procentowy
static const int P_BAR_Y  = 110;  // y paska
static const int P_BAR_H  = 14;   // wysokość paska

// Siatka 2x2
static const int P_GRID_TOP     = 136; // gdzie startuje grid
static const int P_LABEL_Y_OFF  = 8;   // offset label w komórce
static const int P_VAL_Y_OFF    = 34;  // offset wartości w komórce
static const int P_UNIT_Y_OFF   = 62;  // offset jednostki w komórce


// ===================== UI: DASH =====================
// prototypy (żeby wrapper mógł wołać funkcje niezależnie od kolejności)
static void uiDashDrawStaticLandscape();
static void uiDashUpdateDynamicLandscape(int sootPct, int kmSince, float vbatV, float oilC, float ectC, const String& statusStr);

static void uiDashDrawStaticPortrait();
static void uiDashUpdateDynamicPortrait(int sootPct, int kmSince, float vbatV, float oilC, float ectC, const String& statusStr);

// auto-wybór orientacji (działa, bo tft.width/height zależy od rotation)
static inline bool dashIsPortrait() { return tft.width() < tft.height(); }

// --- WRAPPERY (reszta kodu nic nie musi wiedzieć o layoutach) ---
static void uiDashDrawStatic() {
  if (dashIsPortrait()) uiDashDrawStaticPortrait();
  else                  uiDashDrawStaticLandscape();
}

static void uiDashUpdateDynamic(int sootPct, int kmSince, float vbatV, float oilC, float ectC, const String& statusStr) {
  if (dashIsPortrait()) uiDashUpdateDynamicPortrait(sootPct, kmSince, vbatV, oilC, ectC, statusStr);
  else                  uiDashUpdateDynamicLandscape(sootPct, kmSince, vbatV, oilC, ectC, statusStr);
}


// ===================== LANDSCAPE (320x172) =====================
static void uiDashDrawStaticLandscape() {
  const int W = tft.width();
  const int pad = D_PAD;

  tft.fillScreen(COL_BG);
  tft.fillRect(0, 0, W, D_PCT_H, COL_BG);

  // duży procent po prawej
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(COL_TITLE, COL_BG);
  tft.setTextFont(6);
  tft.drawString("--%", W - pad, D_PCT_TEXT_Y);

  // status po lewej
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_TITLE, COL_BG);
  tft.setTextFont(4);
  tft.drawString(String(g_filterLabel) + " : ---", D_STAT_X, D_STAT_Y);


  // pasek %
  int barX = pad;
  int barW = W - 2*pad;
  tft.fillRoundRect(barX, D_BAR_Y, barW, D_BAR_H, D_BAR_H/2, COL_BAR_BG);

  // siatka 4 kolumny
  int x0 = pad;
  int colW = (W - 2*pad) / 4;

  for (int i = 1; i < 4; i++) {
    int x = x0 + i*colW;
    tft.drawLine(x, D_GRID_TOP, x, D_GRID_TOP + D_GRID_H, COL_GRID);
  }

  auto colStatic = [&](int idx, const char* label, const char* unit, uint16_t labelCol) {
    int cx = x0 + idx*colW + colW/2;

    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(labelCol, COL_BG);
    tft.setTextFont(4);
    tft.drawString(label, cx, D_GRID_TOP + D_LABEL_Y_OFF);

    tft.setTextColor(COL_TEXT, COL_BG);
    tft.setTextFont(4);
    tft.drawString(unit, cx, D_GRID_TOP + D_UNIT_Y_OFF);
  };

  colStatic(0, "REG",  "km", COL_GREY);
  colStatic(1, "VBAT", "V",  COL_GREY);
  colStatic(2, "OIL",  "C",  COL_GREY);
  colStatic(3, "ECT",  "C",  COL_GREY);

  // wyczyść miejsca na wartości
  int yValTop = D_GRID_TOP + D_VAL_Y_OFF - 4;
  for (int i=0;i<4;i++){
    int x = x0 + i*colW + 2;
    tft.fillRect(x, yValTop, colW-2, 34, COL_BG);
  }

  // reset cache
  g_dash_lastPct  = -999;
  g_dash_lastKm   = -999;
  g_dash_lastVbat = -999;
  g_dash_lastOil  = -999;
  g_dash_lastEct  = -999;
  g_dash_lastStatus = "";

  g_dashStaticDrawn = true;
}

static void uiDashUpdateDynamicLandscape(int sootPct, int kmSince, float vbatV, float oilC, float ectC, const String& statusStr) {
  const int W = tft.width();
  const int pad = D_PAD;

  int barX = pad;
  int barW = W - 2*pad;

  bool pctValid = (sootPct >= 0 && sootPct <= 100);
  int pct = pctValid ? sootPct : 0;
  int pctKey = pctValid ? pct : -1;

  if (pctKey != g_dash_lastPct) {
    g_dash_lastPct = pctKey;

    // procent
    tft.fillRect(W - D_PCT_W, 0, D_PCT_W, D_PCT_H, COL_BG);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.setTextFont(6);
    tft.drawString(pctValid ? (String(pct) + "%") : String("--%"), W - pad, D_PCT_TEXT_Y);

    // bar
    tft.fillRoundRect(barX, D_BAR_Y, barW, D_BAR_H, D_BAR_H/2, COL_BAR_BG);
    int fillW = pctValid ? (barW * pct) / 100 : 0;
    uint16_t barCol = (pct >= 80) ? COL_WARM : COL_BAR_FG;
    if (pctValid && fillW > 0) tft.fillRoundRect(barX, D_BAR_Y, fillW, D_BAR_H, D_BAR_H/2, barCol);
  }

  if (statusStr != g_dash_lastStatus) {
    g_dash_lastStatus = statusStr;

    int wLeft = W - D_PCT_W - 6;
    tft.fillRect(0, 0, wLeft, D_BAR_Y, COL_BG);

    String s = String(g_filterLabel) + " : " + statusStr;


    int maxW = wLeft - D_STAT_X - 6;

    // ✅ OFF/REQ będą duże (font4), WARMING/StageX spadnie do font2 jeśli trzeba
    uint8_t font = (tft.textWidth(s, 4) <= maxW) ? 4 : 2;

    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(COL_TITLE, COL_BG);
    tft.setTextFont(font);
    tft.drawString(s, D_STAT_X, D_STAT_Y);
  }


  int x0 = pad;
  int colW = (W - 2*pad) / 4;
  int yValTop = D_GRID_TOP + D_VAL_Y_OFF - 4;
  int yValTxt = D_GRID_TOP + D_VAL_Y_OFF;

  auto drawValue = [&](int idx, const String& v, uint16_t col) {
    int cx = x0 + idx*colW + colW/2;
    tft.fillRect(x0 + idx*colW + 2, yValTop, colW - 2, 34, COL_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(col, COL_BG);
    tft.setTextFont(4);
    tft.drawString(v, cx, yValTxt);
  };

  if (kmSince != g_dash_lastKm) {
    g_dash_lastKm = kmSince;
    drawValue(0, (kmSince < 0) ? String("--") : String(kmSince), COL_TEXT);
  }

  if (fabsf(vbatV - g_dash_lastVbat) > 0.05f) {
    g_dash_lastVbat = vbatV;
    drawValue(1, (vbatV < 0) ? String("--") : fmtFloatPL_noUnit(vbatV, 1), COL_TEXT);
  }

  if (fabsf(oilC - g_dash_lastOil) > 1.0f) {
    g_dash_lastOil = oilC;
    drawValue(2, (!tempValid(oilC)) ? String("--") : String((int)oilC), COL_TEXT);
  }

  if (fabsf(ectC - g_dash_lastEct) > 1.0f) {
    g_dash_lastEct = ectC;
    drawValue(3, (!tempValid(ectC)) ? String("--") : String((int)ectC), COL_TEXT);
  }
}


// ===================== PORTRAIT (172x320) =====================
static void uiDashDrawStaticPortrait() {
  const int W = tft.width();
  const int H = tft.height();
  const int pad = P_PAD;

  tft.fillScreen(COL_BG);

  // STATUS (góra)
  tft.fillRect(0, 0, W, P_STAT_H, COL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_TITLE, COL_BG);
  tft.setTextFont(4);
  tft.drawString(String(g_filterLabel) + " ---", W/2, P_STAT_Y);


  // DUŻY PROCENT (środek u góry)
  tft.fillRect(0, P_PCT_Y, W, P_PCT_H, COL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextFont(6);
  tft.drawString("--%", W/2, P_PCT_Y);

  // BAR
  int barX = pad;
  int barW = W - 2*pad;
  tft.fillRoundRect(barX, P_BAR_Y, barW, P_BAR_H, P_BAR_H/2, COL_BAR_BG);

  // GRID 2x2
  int gridX = pad;
  int gridY = P_GRID_TOP;
  int gridW = W - 2*pad;
  int gridH = H - gridY - pad;

  int cellW = gridW / 2;
  int cellH = gridH / 2;

  // linie podziału
  tft.drawLine(gridX + cellW, gridY, gridX + cellW, gridY + gridH, COL_GRID);
  tft.drawLine(gridX, gridY + cellH, gridX + gridW, gridY + cellH, COL_GRID);

  auto cellStatic = [&](int idx, const char* label, const char* unit, uint16_t labelCol) {
    int r = idx / 2;
    int c = idx % 2;
    int cellX = gridX + c*cellW;
    int cellY = gridY + r*cellH;
    int cx    = cellX + cellW/2;

    tft.setTextDatum(TC_DATUM);

    tft.setTextColor(labelCol, COL_BG);
    tft.setTextFont(4);
    tft.drawString(label, cx, cellY + P_LABEL_Y_OFF);

    tft.setTextColor(COL_TEXT, COL_BG);
    tft.setTextFont(4);
    tft.drawString(unit, cx, cellY + P_UNIT_Y_OFF);

    // miejsce na wartość
    int yValTop = cellY + P_VAL_Y_OFF - 4;
    tft.fillRect(cellX + 2, yValTop, cellW - 4, 34, COL_BG);
  };

  // układ 2x2:
  // [ REG | VBAT ]
  // [ OIL | ECT  ]
  cellStatic(0, "REG",  "km", COL_GREY);
  cellStatic(1, "VBAT", "V",  COL_GREY);
  cellStatic(2, "OIL",  "C",  COL_GREY);
  cellStatic(3, "ECT",  "C",  COL_GREY);

  // reset cache
  g_dash_lastPct  = -999;
  g_dash_lastKm   = -999;
  g_dash_lastVbat = -999;
  g_dash_lastOil  = -999;
  g_dash_lastEct  = -999;
  g_dash_lastStatus = "";

  g_dashStaticDrawn = true;
}

static void uiDashUpdateDynamicPortrait(int sootPct, int kmSince, float vbatV, float oilC, float ectC, const String& statusStr) {
  const int W = tft.width();
  const int H = tft.height();
  const int pad = P_PAD;

  // PROCENT + BAR
  bool pctValid = (sootPct >= 0 && sootPct <= 100);
  int pct = pctValid ? sootPct : 0;
  int pctKey = pctValid ? pct : -1;

  int barX = pad;
  int barW = W - 2*pad;

  if (pctKey != g_dash_lastPct) {
    g_dash_lastPct = pctKey;

    // procent
    tft.fillRect(0, P_PCT_Y, W, P_PCT_H, COL_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.setTextFont((pct >= 100) ? 4 : 6); // 100% bywa szerokie na 172px
    tft.drawString(pctValid ? (String(pct) + "%") : String("--%"), W/2, P_PCT_Y);

    // bar
    tft.fillRoundRect(barX, P_BAR_Y, barW, P_BAR_H, P_BAR_H/2, COL_BAR_BG);
    int fillW = pctValid ? (barW * pct) / 100 : 0;
    uint16_t barCol = (pct >= 80) ? COL_WARM : COL_BAR_FG;
    if (pctValid && fillW > 0) tft.fillRoundRect(barX, P_BAR_Y, fillW, P_BAR_H, P_BAR_H/2, barCol);
  }

  // STATUS (góra)
if (statusStr != g_dash_lastStatus) {
  g_dash_lastStatus = statusStr;

  tft.fillRect(0, 0, W, P_STAT_H, COL_BG);

  String s = String(g_filterLabel) + " " + statusStr;
  int maxW = W - 12;

  uint8_t font = (tft.textWidth(s, 4) <= maxW) ? 4 : 2;

  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_TITLE, COL_BG);
  tft.setTextFont(font);
  tft.drawString(s, W/2, P_STAT_Y);
}


  // GRID 2x2 VALUES
  int gridX = pad;
  int gridY = P_GRID_TOP;
  int gridW = W - 2*pad;
  int gridH = H - gridY - pad;

  int cellW = gridW / 2;
  int cellH = gridH / 2;

  auto drawCellValue = [&](int idx, const String& v, uint16_t col) {
    int r = idx / 2;
    int c = idx % 2;

    int cellX = gridX + c*cellW;
    int cellY = gridY + r*cellH;
    int cx    = cellX + cellW/2;

    int yValTop = cellY + P_VAL_Y_OFF - 4;
    int yValTxt = cellY + P_VAL_Y_OFF;

    tft.fillRect(cellX + 2, yValTop, cellW - 4, 34, COL_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(col, COL_BG);
    tft.setTextFont(4);
    tft.drawString(v, cx, yValTxt);
  };

  if (kmSince != g_dash_lastKm) {
    g_dash_lastKm = kmSince;
    drawCellValue(0, (kmSince < 0) ? String("--") : String(kmSince), COL_TEXT);
  }

  if (fabsf(vbatV - g_dash_lastVbat) > 0.05f) {
    g_dash_lastVbat = vbatV;
    drawCellValue(1, (vbatV < 0) ? String("--") : fmtFloatPL_noUnit(vbatV, 1), COL_TEXT);
  }

  if (fabsf(oilC - g_dash_lastOil) > 1.0f) {
    g_dash_lastOil = oilC;
    drawCellValue(2, (!tempValid(oilC)) ? String("--") : String((int)oilC), COL_TEXT);
  }

  if (fabsf(ectC - g_dash_lastEct) > 1.0f) {
    g_dash_lastEct = ectC;
    drawCellValue(3, (!tempValid(ectC)) ? String("--") : String((int)ectC), COL_TEXT);
  }
}


// ===================== UI: BURN =====================
static void uiBurnDrawStatic() {
  const int W = tft.width();
  tft.fillScreen(COL_BG);

  // miejsce na tytuł (dynamiczny)
  tft.fillRect(0, 0, W, 32, COL_BG);

  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextFont(6);
  tft.drawString("--%", W/2, 55);

  int bx = 16, by = 118, bw = W - 32, bh = 24;
  tft.fillRoundRect(bx, by, bw, bh, bh/2, COL_BAR_BG);

  tft.fillRect(W/2 - 90, 34, 180, 40, COL_BG);

  g_burn_lastPct = -999;
  g_burn_lastTitle = "";
  g_burnStaticDrawn = true;

  drawFrame(COL_RED);
}

static void uiBurnUpdateDynamic(int pctCenter, float, const String& stateStr, uint32_t) {
  const int W = tft.width();

  // --- TITLE (STATE) ---
  String title = stateStr;
  if (title.equalsIgnoreCase("Actively")) title = "BURNING";  // Actively -> BURNING
  title.toUpperCase();

  if (title != g_burn_lastTitle) {
    g_burn_lastTitle = title;

    tft.fillRect(0, 0, W, 32, COL_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COL_TITLE, COL_BG);
    tft.setTextFont(4);
    tft.drawString(title, W/2, 13);
  }

  // --- PERCENT ---
  int pct = pctCenter;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  if (pct != g_burn_lastPct) {
    g_burn_lastPct = pct;
    tft.fillRect(W/2 - 90, 34, 180, 40, COL_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.setTextFont(6);
    tft.drawString(String(pct) + "%", W/2, 55);
  }

  // --- BAR ---
  int bx = 16, by = 118, bw = W - 32, bh = 24;
  tft.fillRoundRect(bx, by, bw, bh, bh/2, COL_BAR_BG);
  int fillW = (bw * pct) / 100;
  if (fillW > 0) tft.fillRoundRect(bx, by, fillW, bh, bh/2, COL_RED);

  // ramka na wierzchu
  drawFrame(COL_RED);
}

// ===================== UI: SUCCESS =====================
static void uiSuccessDrawStatic() {
  const int W = tft.width();
  const int H = tft.height();
  const int cx = W/2;

  tft.fillScreen(COL_BG);
  tft.setTextDatum(TC_DATUM);

  int maxW = W - 30;

  // pełny tekst, auto-fit / ewentualnie łamanie
  drawCenterFit2("REGENERATION COMPLETED", cx,
                 H/2 - 18, H/2 + 10,
                 maxW, 4, 2, COL_BAR_FG, COL_BG);

  g_successStaticDrawn = true;
  drawFrame(COL_BAR_FG);
}


static void uiAbortDrawStatic() {
  const int W = tft.width();
  const int H = tft.height();
  const int cx = W/2;

  tft.fillScreen(COL_BG);
  tft.setTextDatum(TC_DATUM);

  int maxW = W - 30;

  drawCenterFit2("REGENERATION ABORTED", cx,
                 H/2 - 18, H/2 + 10,
                 maxW, 4, 2, COL_WARM, COL_BG);

  g_abortStaticDrawn = true;
  drawFrame(COL_WARM);
}





// ===================== TEST MODE =====================
static void setTestMode(bool on) {
  g_testMode = on;
  g_testStartMs = millis();
  showMsg(on ? "TEST MODE" : "TEST MODE", on ? "Data simulation" : "Off");
}

static void genTestData(int& soot, int& km, bool& burnReq, float& vbat, float& oil, float& ect, uint8_t& stateA) {
  uint32_t t = millis() - g_testStartMs;
  soot = (int)((t / 250) % 100);
  km   = (t / 1000) % 300;
  vbat = 14.3f + 0.2f * sinf(2.0f * 3.14159f * (t % 7000) / 7000.0f);
  oil  = 92.0f  + 8.0f  * sinf(2.0f * 3.14159f * (t % 6000) / 6000.0f);
  ect  = 81.0f  + 11.0f * sinf(2.0f * 3.14159f * (t % 9000) / 9000.0f);

  bool on = ((t / 12000) % 2) == 1;
  burnReq = on;
  stateA  = on ? 0x07 : 0x00;
}

// ===================== BLE helpers (FIXED) =====================
// Sprawdza czy target MAC w ogóle jest widoczny w reklamach BLE (zanim będziesz próbował connect)
static bool bleTargetPresent(const char* mac, int seconds = 1) {
  String target = String(mac); target.toLowerCase();

  bleLock();

  BLEScan* scan = BLEDevice::getScan();
  scan->stop();
  scan->clearResults();
  scan->setActiveScan(true);
  scan->setInterval(120);
  scan->setWindow(80);

  BLEScanResults* res = scan->start(seconds, false);

  bool found = false;
  if (res) {
    int n = res->getCount();
    for (int i = 0; i < n; i++) {
      BLEAdvertisedDevice d = res->getDevice(i);
      String a = d.getAddress().toString().c_str();
      a.toLowerCase();
      if (a == target) { found = true; break; }
    }
  }

  bleUnlock();
  return found;
}

// ===================== BLE connect/discover =====================

static bool probeELMChannel(uint32_t timeoutMs = 900) {
  if (!g_charRX || !g_charTX) return false;

  // jeśli mutex jeszcze nie istnieje (teoretycznie), nie blokuj connect
  if (!g_elmMutex) return true;

  String out;
  if (sendELMAndWait("ATI", out, timeoutMs)) return true;
  if (sendELMAndWait("AT I", out, timeoutMs)) return true;
  return false;
}


static bool connectAndDiscover(const char* mac) {
  bleLock();

  Serial.print("[BLE] Connecting to "); Serial.println(mac);
  BLEAddress addr(mac);

  if (!g_client) {
    g_client = BLEDevice::createClient();
    if (!g_client) { bleUnlock(); return false; }
  } else {
    if (g_client->isConnected()) g_client->disconnect();
  }

  g_charTX = nullptr;
  g_charRX = nullptr;

  if (!g_client->connect(addr)) {
    Serial.println("[BLE] Connect failed");
    bleUnlock();
    return false;
  }

  Serial.println("[BLE] Connected");
  g_client->setMTU(185);

  auto* svcs = g_client->getServices();
  if (!svcs || svcs->empty()) {
    Serial.println("[BLE] No services");
    g_client->disconnect();
    bleUnlock();
    return false;
  }

  for (auto& kv : *svcs) {
    BLERemoteService* svc = kv.second;
    if (!svc) continue;

    auto* chars = svc->getCharacteristics();
    if (!chars) continue;

    BLERemoteCharacteristic* tx = nullptr;
    BLERemoteCharacteristic* rx = nullptr;

    for (auto& ck : *chars) {
      BLERemoteCharacteristic* ch = ck.second;
      if (!ch) continue;
      if (!tx && (ch->canNotify() || ch->canIndicate())) tx = ch;
      if (!rx && (ch->canWrite() || ch->canWriteNoResponse())) rx = ch;
      if (tx && rx) break;
    }

    if (tx && rx) {
      g_charTX = tx;
      g_charRX = rx;

      g_writeWithResponse = g_charRX->canWrite();

      g_charTX->registerForNotify(notifyCB);

      if (auto* cccd = g_charTX->getDescriptor(BLEUUID((uint16_t)0x2902))) {
        uint8_t v[2] = { g_charTX->canIndicate() ? 0x02 : 0x01, 0x00 };
        cccd->writeValue(v, 2, true);
      }


      // PROBE: sprawdź, czy to faktycznie kanał OBD
      if (probeELMChannel(900)) {
        Serial.print("[BLE] TX: "); Serial.println(g_charTX->getUUID().toString().c_str());
        Serial.print("[BLE] RX: "); Serial.println(g_charRX->getUUID().toString().c_str());
        bleUnlock();
        return true;
      }

      Serial.println("[BLE] TX/RX pair found but no ELM response, trying next...");

      // opcjonalnie: wyłącz notify na tej parze
      if (auto* cccd = g_charTX->getDescriptor(BLEUUID((uint16_t)0x2902))) {
        uint8_t v0[2] = {0x00, 0x00};
        cccd->writeValue(v0, 2, true);
      }

      g_charTX = nullptr;
      g_charRX = nullptr;
    }
  }

  Serial.println("[BLE] TX/RX not found");
  g_client->disconnect();
  bleUnlock();
  return false;
}


static void elmInit() {
  String out;
  sendELMAndWait("ATZ",   out, 1500); vTaskDelay(pdMS_TO_TICKS(150));
  sendELMAndWait("ATE0",  out, 600);  vTaskDelay(pdMS_TO_TICKS(50));
  sendELMAndWait("ATL0",  out, 600);  vTaskDelay(pdMS_TO_TICKS(50));
  sendELMAndWait("ATH0",  out, 600);  vTaskDelay(pdMS_TO_TICKS(50));
  sendELMAndWait("ATAL",  out, 600);  vTaskDelay(pdMS_TO_TICKS(50));
  sendELMAndWait("ATSP0", out, 900);  vTaskDelay(pdMS_TO_TICKS(80));
  sendELMAndWait("ATCAF1", out, 600);  vTaskDelay(pdMS_TO_TICKS(50)); // CAN auto formatting ON
  sendELMAndWait("ATCFC1", out, 600);  vTaskDelay(pdMS_TO_TICKS(50)); // flow control ON (dla multi-frame)

  g_lastHdr = 0xFFFFFFFF;
}

  // ===================== NVS =====================
static void loadSettings() {

  // --- NVS version checks ---
  ensureGlobalNvsVersion();
  ensureProfileNvsVersion(g_nvsNS.c_str());

  // ===== MAC (GLOBAL) =====
  g_mac = DEFAULT_MAC;
  if (g_prefs.begin(NVS_GLOBAL, true)) {
    g_mac = g_prefs.getString("mac", DEFAULT_MAC);
    g_prefs.end();
  }

  g_mac.trim();
  g_mac.toUpperCase();
  if (!isMacStringValid(g_mac) || g_mac == "00:00:00:00:00:00") {
    g_mac = DEFAULT_MAC;
  }

  // ===== PROFILE DEFAULTS =====
  const CarProfile& P = PROFILES[g_profileIdx];
  g_filterLabel = (P.filterLabel && *P.filterLabel) ? P.filterLabel : "DPF";


  // jeśli nie da się otworzyć profilu -> bierz default profilu
  if (!g_prefs.begin(g_nvsNS.c_str(), true)) {
    g_header       = P.header;

    g_did_soot     = P.did_soot;
    g_did_km       = P.did_km;
    g_did_burn     = P.did_burn;
    g_did_state    = P.did_state;
    g_did_prog     = P.did_prog;
    g_did_oil_uds  = P.did_oil_uds;

    g_pid_vbat     = P.pid_vbat;
    g_pid_ect      = P.pid_ect;
    g_pid_oil      = P.pid_oil;

    g_blPct = 80;
    g_rotation = 1;

    updateObdFuncHeader();
    return;
  }

  // ===== READ PROFILE (NVS) =====
  g_header = g_prefs.getUInt("header", (uint32_t)P.header);

  g_did_soot     = (uint16_t)g_prefs.getUShort("did_soot",  P.did_soot);
  g_did_km       = (uint16_t)g_prefs.getUShort("did_km",    P.did_km);
  g_did_burn     = (uint16_t)g_prefs.getUShort("did_burn",  P.did_burn);
  g_did_state    = (uint16_t)g_prefs.getUShort("did_state", P.did_state);
  g_did_prog     = (uint16_t)g_prefs.getUShort("did_prog",  P.did_prog);
  g_did_oil_uds  = (uint16_t)g_prefs.getUShort("did_oil",   P.did_oil_uds);

  g_pid_vbat     = (uint8_t)g_prefs.getUChar("pid_vbat", P.pid_vbat);
  g_pid_ect      = (uint8_t)g_prefs.getUChar("pid_ect",  P.pid_ect);
  g_pid_oil      = (uint8_t)g_prefs.getUChar("pid_oil",  P.pid_oil);

  g_blPct        = (uint8_t)g_prefs.getUChar("bl", 80);
  g_rotation     = (uint8_t)g_prefs.getUChar("rot", 1);
  if (g_rotation > 3) g_rotation = 1;

  g_prefs.end();

  updateObdFuncHeader();

  Serial.printf("[NVS] MAC=%s HEADER=0x%lX FUNC=0x%lX NS=%s\n",
                g_mac.c_str(),
                (unsigned long)g_header,
                (unsigned long)g_obdFuncHeader,
                g_nvsNS.c_str());
}


static void saveDIDs() {
  if (g_prefs.begin(g_nvsNS.c_str(), false)) {
    g_prefs.putUShort("cfg_ver", CFG_VER_PROFILE);
    g_prefs.putUShort("did_soot",  g_did_soot);
    g_prefs.putUShort("did_km",    g_did_km);
    g_prefs.putUShort("did_burn",  g_did_burn);
    g_prefs.putUShort("did_state", g_did_state);
    g_prefs.putUShort("did_prog",  g_did_prog);
    g_prefs.putUShort("did_oil",   g_did_oil_uds);
    g_prefs.end();
    Serial.println("[NVS] Saved DIDs");
  }
}

static void savePIDs() {
  if (g_prefs.begin(g_nvsNS.c_str(), false)) {
    g_prefs.putUShort("cfg_ver", CFG_VER_PROFILE);
    g_prefs.putUChar("pid_vbat", g_pid_vbat);
    g_prefs.putUChar("pid_ect",  g_pid_ect);
    g_prefs.putUChar("pid_oil",  g_pid_oil);
    g_prefs.end();
    Serial.println("[NVS] Saved PIDs");
  }
}

static void saveHeader() {
  if (g_prefs.begin(g_nvsNS.c_str(), false)) {
    g_prefs.putUShort("cfg_ver", CFG_VER_PROFILE);
    g_prefs.putUInt("header", g_header);     // <= TYLKO UInt
    g_prefs.end();
    Serial.println("[NVS] Saved HEADER");
  }
}


static void saveBacklight() {
  if (g_prefs.begin(g_nvsNS.c_str(), false)) {
    g_prefs.putUShort("cfg_ver", CFG_VER_PROFILE);
    g_prefs.putUChar("bl", g_blPct);
    g_prefs.end();
    Serial.println("[NVS] Saved BL");
  }
}

static void saveRotation() {
  if (g_prefs.begin(g_nvsNS.c_str(), false)) {
    g_prefs.putUShort("cfg_ver", CFG_VER_PROFILE);
    g_prefs.putUChar("rot", g_rotation);
    g_prefs.end();
    Serial.println("[NVS] Saved ROT");
  }
}

static void saveMac() {
  if (g_prefs.begin(NVS_GLOBAL, false)) {
    g_prefs.putUShort("cfg_ver", CFG_VER_GLOBAL);
    g_prefs.putString("mac", g_mac);
    g_prefs.end();
    Serial.println("[NVS] Saved MAC");
  }
}

static void saveMacAndRestart() {
  saveMac();
  Serial.println("[SYS] Restarting...");
  delay(250);
  ESP.restart();
}

static void saveActiveProfile() {
  if (g_prefs.begin(NVS_GLOBAL, false)) {
    g_prefs.putUShort("cfg_ver", CFG_VER_GLOBAL);
    g_prefs.putUChar("profile", (uint8_t)g_profileIdx);
    g_prefs.end();
    Serial.printf("[NVS] Saved ACTIVE PROFILE=%d (%s)\n", g_profileIdx + 1, PROFILES[g_profileIdx].key);
  }
}

static int loadActiveProfile() {
  int idx = 0; // default
  if (g_prefs.begin(NVS_GLOBAL, true)) {
    idx = (int)g_prefs.getUChar("profile", 0);
    g_prefs.end();
  }

  bool fixed = false;
  if (idx < 0 || idx >= PROFILE_COUNT) {
    idx = 0;
    fixed = true;
  }

  g_profileIdx = idx;

  // zapis tylko jeśli musieliśmy naprawić wartość
  if (fixed) saveActiveProfile();

  return idx;
}



// ===================== Console helpers =====================
static void printHelp() {
  Serial.println();
  Serial.println(F("=== help / komendy ==="));
  Serial.println(F("Wpisuj małymi literami (duże też działają)."));
  Serial.println();

  Serial.println(F("[połączenie ble / elm]"));
  Serial.println(F("  scan [s]        - skan BLE (domyślnie 5s), zapisuje listę"));
  Serial.println(F("  list            - pokaż urządzenia z ostatniego scan"));
  Serial.println(F("  use <n>         - ustaw MAC z listy (bez łączenia)"));
  Serial.println(F("  conn <n> [save] - ustaw MAC z listy i połącz (save=zapamiętaj MAC + restart)"));
  Serial.println(F("  connect         - połącz z zapisanym MAC"));
  Serial.println(F("  disconnect      - rozłącz BLE"));
  Serial.println(F("  status          - status połączenia + konfiguracja"));
  Serial.println();

  Serial.println(F("[profil auta]"));
  Serial.println(F("  cars            - lista profili"));
  Serial.println(F("  car?            - aktywny profil"));
  Serial.println(F("  car <n>         - wybierz profil"));
  Serial.println();

  Serial.println(F("[konfiguracja obd/uds]"));
  Serial.println(F("  mac             - pokaż MAC"));
  Serial.println(F("  mac <AA:..>     - ustaw MAC"));
  Serial.println(F("  mac save        - zapisz MAC (restart)"));
  Serial.println(F("  header          - pokaż nagłówek (CAN ID)"));
  Serial.println(F("  header <hex>    - ustaw (np. 7e0 lub 18daf110)"));
  Serial.println(F("  header save     - zapisz header"));
  Serial.println(F("  pid             - pokaż PIDy (01)"));
  Serial.println(F("  pid <vbat|ect|oil> <hex> - ustaw PID (np. pid oil 5c)"));
  Serial.println(F("  pid save        - zapisz PIDy"));
  Serial.println(F("  did             - pokaż DIDy (22)"));
  Serial.println(F("  did <soot|km|burn|state|prog|oil> <hex> - ustaw DID"));
  Serial.println(F("  did save        - zapisz DIDy"));
  Serial.println();

  Serial.println(F("[ekran]"));
  Serial.println(F("  bl <0-100>      - jasność (%)"));
  Serial.println(F("  bl save         - zapisz jasność"));
  Serial.println(F("  rot <0-3>       - obrót"));
  Serial.println(F("  rot 180         - obrót o 180°"));
  Serial.println(F("  rot save        - zapisz obrót"));
  Serial.println();

  Serial.println(F("[odczyt / diagnostyka]"));
  Serial.println(F("  vin             - odczyt VIN (OBD 09 02 / UDS 22 F190)"));
  Serial.println(F("  vbat | ect | oil - szybki odczyt PID"));
  Serial.println(F("  req | state      - szybki odczyt DID (regen)"));
  Serial.println(F("  read22 <0xXXXX>  - odczyt dowolnego DID (np. read22 336a)"));
  Serial.println(F("  at <cmd>         - surowa komenda do ELM (np. at i, 01 0c, 22 33 6a)"));
  Serial.println(F("  logio on/off     - log TX/RX"));
  Serial.println(F("  test on/off      - tryb testowy UI (bez auta)"));
  Serial.println();

  Serial.println(F("[narzędzia]"));
  Serial.println(F("  testall          - test PID/DID dla profilu"));
  Serial.println(F("  dump             - pełny zrzut konfiguracji"));
  Serial.println(F("  factory / factory yes - reset profilu do domyślnych"));
  Serial.println();
}


static bool parseHexU16(const String& s, uint16_t& out) {
  String x = s; x.trim(); x.toUpperCase();
  const char* c = x.c_str(); if (x.startsWith("0X")) c += 2;
  char* e = nullptr; long v = strtol(c, &e, 16);
  if (!e || *e != 0) return false;
  if (v<0 || v>0xFFFF) return false;
  out = (uint16_t)v;
  return true;
}

static bool parseHexU32(const String& s, uint32_t& out) {
  String x = s; x.trim(); x.toUpperCase();
  const char* c = x.c_str(); if (x.startsWith("0X")) c += 2;
  char* e = nullptr; unsigned long v = strtoul(c, &e, 16);
  if (!e || *e != 0) return false;
  if (v > 0x1FFFFFFFul) return false;   // max 29-bit CAN ID
  out = (uint32_t)v;
  return true;
}


static bool parseHexU8(const String& s, uint8_t& out) {
  String x = s; x.trim(); x.toUpperCase();
  const char* c = x.c_str(); if (x.startsWith("0X")) c += 2;
  char* e = nullptr; long v = strtol(c, &e, 16);
  if (!e || *e != 0) return false;
  if (v < 0 || v > 0xFF) return false;
  out = (uint8_t)v;
  return true;
}

static void showDidStatus() {
  Serial.printf("[DID] SOOT=0x%04X KM=0x%04X BURN=0x%04X STATE=0x%04X PROG=0x%04X OIL_UDS=0x%04X\n",
                g_did_soot, g_did_km, g_did_burn, g_did_state, g_did_prog, g_did_oil_uds);
}

static void showPidStatus() {
  Serial.printf("[PID] VBAT=0x%02X ECT=0x%02X OIL=0x%02X\n",
                g_pid_vbat, g_pid_ect, g_pid_oil);
}

static void dumpConfig() {
  Serial.println("===== CONFIG DUMP =====");
  Serial.printf("[FW] build: %s %s\n", __DATE__, __TIME__);

  Serial.printf("[CAR] %d/%d key=%s name=%s ns=%s\n",
                g_profileIdx + 1, PROFILE_COUNT,
                PROFILES[g_profileIdx].key,
                PROFILES[g_profileIdx].name,
                g_nvsNS.c_str());

  Serial.printf("[NVS] ver_global=%u ver_profile=%u\n", CFG_VER_GLOBAL, CFG_VER_PROFILE);

  Serial.printf("[MAC] %s\n", g_mac.c_str());
  if (g_header <= 0x7FF) Serial.printf("[HEADER] 0x%03lX\n", (unsigned long)g_header);
  else                   Serial.printf("[HEADER] 0x%08lX\n", (unsigned long)g_header);


  showPidStatus();
  showDidStatus();

  Serial.printf("[BL] %u %%\n", (unsigned)g_blPct);
  Serial.printf("[ROT] %u\n", (unsigned)g_rotation);

  Serial.printf("[BLE] connected=%s\n", isConnectedOBD() ? "YES" : "NO");
  if (g_charTX) {
    String u = g_charTX->getUUID().toString();
    Serial.printf("[BLE] TX uuid=%s\n", u.c_str());
  }
  if (g_charRX) {
    String u = g_charRX->getUUID().toString();
    Serial.printf("[BLE] RX uuid=%s\n", u.c_str());
  }


  // ELM auto-recover stats (bezpiecznie)
  portENTER_CRITICAL(&g_elmStatMux);
  uint8_t streak = g_elmFailStreak;
  bool need = g_elmNeedsReinit;
  bool inr = g_elmInReinit;
  uint32_t cd = g_elmCooldownUntilMs;
  portEXIT_CRITICAL(&g_elmStatMux);

  Serial.printf("[ELM] failStreak=%u needsReinit=%s inReinit=%s cooldownUntilMs=%lu\n",
                (unsigned)streak,
                need ? "YES" : "NO",
                inr ? "YES" : "NO",
                (unsigned long)cd);

  Serial.println("=======================");
}




static void doScan(int seconds) {
  g_pauseReconnectUntilMs = millis() + (uint32_t)(seconds + 1) * 1000;

  // wyczyść cache
  g_scanCount = 0;
  for (int i = 0; i < SCAN_MAX; i++) {
    g_scanMac[i] = "";
    g_scanName[i] = "";
  }

  // rozłącz, żeby nie przeszkadzało w skanowaniu
  if (g_client && g_client->isConnected()) g_client->disconnect();
  g_charTX = nullptr;
  g_charRX = nullptr;

  bleLock();

  BLEScan* scan = BLEDevice::getScan();
  scan->stop();
  scan->clearResults();
  scan->setActiveScan(true);
  scan->setInterval(120);
  scan->setWindow(80);

  Serial.printf("[BLE] Scanning %d s...\n", seconds);
  BLEScanResults* res = scan->start(seconds, false);
  if (!res) {
    bleUnlock();
    Serial.println("[BLE] Scan error");
    return;
  }

  int n = res->getCount();
  Serial.printf("[BLE] Found %d devices\n", n);

  // zapamiętujemy max SCAN_MAX urządzeń
  g_scanCount = (n > SCAN_MAX) ? SCAN_MAX : n;

  for (int i = 0; i < n; i++) {
    BLEAdvertisedDevice d = res->getDevice(i);

    String mac = d.getAddress().toString().c_str();
    int rssi = d.getRSSI();
    String name = d.haveName() ? String(d.getName().c_str()) : String("(no name)");

    Serial.printf("  %2d) %s  RSSI=%d  Name=%s\n", i + 1, mac.c_str(), rssi, name.c_str());

    // cache tylko pierwszych SCAN_MAX
    if (i < SCAN_MAX) {
      g_scanMac[i]  = mac;
      g_scanName[i] = name;
    }
  }

  bleUnlock();
}

// ===================== PROFILES / TESTALL / FACTORY =====================

static void listProfiles() {
  Serial.println("[CARS] Available profiles:");
  for (int i = 0; i < PROFILE_COUNT; i++) {
    Serial.printf("  %d) %s  (%s)  NS=%s\n", i+1, PROFILES[i].key, PROFILES[i].name, PROFILES[i].nvsNS);
  }
  Serial.println("Use: CAR <n>   (e.g. CAR 2)");
}

static void testAllOnce() {
  if (!isConnectedOBD()) {
    Serial.println("[TESTALL] not connected. Use CONNECT first.");
    return;
  }

  int ok = 0, fail = 0;

  auto mark = [&](const char* name, bool r){
    Serial.printf("  %-10s : %s\n", name, r ? "OK" : "NO DATA");
    if (r) ok++; else fail++;
  };

  // PIDy
  float f;
  mark("VBAT", queryVBAT(f));
  mark("ECT",  (queryECT(f) && tempValid(f)));
  mark("OIL",  (queryOIL(f) && tempValid(f)));

  // DIDy
  ByteBuf b;
  mark("SOOT",  query22_bytes(g_did_soot,  b, 900));
  mark("STATE", query22_bytes(g_did_state, b, 900));
  mark("BURN",  query22_bytes(g_did_burn,  b, 900));

  // KM jako u16 (na razie jak w Astrze)
  uint16_t v16;
  mark("KM", query22_u16(g_did_km, v16, 900));

  Serial.printf("[TESTALL] OK=%d FAIL=%d\n", ok, fail);
}

static void autoTestHintOrRun() {
  Serial.println("[CAR] Next steps:");

  if (isConnectedOBD()) {
    // jak już jest połączone (np. po CONN#), nie ma sensu pisać CONNECT
    Serial.println("  1) TESTALL");
    Serial.println("  2) If FAIL -> SETPID/SETDID then SAVEPID/SAVEDID");

    Serial.println("[CAR] OBD connected -> running TESTALL now...");
    testAllOnce();
  } else {
    // jak nie ma połączenia, to CONNECT/CONN# ma sens
    Serial.println("  1) CONNECT (or CONN# <n>)");
    Serial.println("  2) TESTALL");
    Serial.println("  3) If FAIL -> SETPID/SETDID then SAVEPID/SAVEDID");
    Serial.println("[CAR] OBD not connected yet.");
  }
}


static void setActiveProfile(int idx, bool disconnectNow = true) {
  if (idx < 0 || idx >= PROFILE_COUNT) {
    Serial.println("[CAR] bad index");
    return;
  }

  g_profileIdx = idx;
  g_nvsNS = PROFILES[idx].nvsNS;

  Serial.printf("[CAR] Active: %s (%s) NS=%s\n",
                PROFILES[idx].key, PROFILES[idx].name, g_nvsNS.c_str());

  if (disconnectNow) {
    if (g_client && g_client->isConnected()) g_client->disconnect();
    g_charTX = nullptr;
    g_charRX = nullptr;
  }

  loadSettings();
  saveActiveProfile();
  g_lastHdr = 0xFFFFFFFF;

  // zastosuj ustawienia ekranu od razu
  setBacklightPct(g_blPct);
  tftLock();
  tft.setRotation(g_rotation);
  tft.fillScreen(COL_BG);
  tftUnlock();

  g_dashStaticDrawn = false;
  g_burnStaticDrawn = false;
  g_successStaticDrawn = false;
  g_abortStaticDrawn = false;

  Serial.println("[CAR] Settings loaded for this profile.");

  // soft test / hint
  autoTestHintOrRun();
}

static void factoryResetProfile(bool keepMac = true) {
  Serial.printf("[FACTORY] Reset profile NS=%s (%s)\n",
                g_nvsNS.c_str(), PROFILES[g_profileIdx].key);

  String macKeep = g_mac;

  if (g_prefs.begin(g_nvsNS.c_str(), false)) {
    g_prefs.clear();
    g_prefs.putUShort("cfg_ver", CFG_VER_PROFILE);
    g_prefs.end();
  }

  loadSettings();

  if (keepMac && isMacStringValid(macKeep) && macKeep != "00:00:00:00:00:00") {
    g_mac = macKeep;
    saveMac();
    Serial.println("[FACTORY] MAC preserved.");
  }

  g_lastHdr = 0xFFFFFFFF;

  setBacklightPct(g_blPct);
  tftLock();
  tft.setRotation(g_rotation);
  tft.fillScreen(COL_BG);
  tftUnlock();

  g_dashStaticDrawn = false;
  g_burnStaticDrawn = false;
  g_successStaticDrawn = false;
  g_abortStaticDrawn = false;

  Serial.println("[FACTORY] Done. Use CONNECT / TESTALL.");
}

static bool readVIN(String& vinOut);

static void handleConsoleLine(const String& lineIn) {
  String line = lineIn; line.trim();
  if (!line.length()) return;

  // Uproszczony parser: rozpoznawanie komend bez wielkości liter
  String lc = line;
  lc.toLowerCase();

  auto connected = [](){ return isConnectedOBD(); };

  if (lc == "help" || lc == "?") { printHelp(); return; }

  // ====== PROFILES ======
  if (lc == "cars") { listProfiles(); return; }

  if (lc == "car?" || lc == "car") {
    Serial.printf("[CAR] Active: %d) %s (%s) NS=%s\n",
                  g_profileIdx+1,
                  PROFILES[g_profileIdx].key,
                  PROFILES[g_profileIdx].name,
                  g_nvsNS.c_str());
    return;
  }

  if (lc == "dump" || lc == "export") {
    dumpConfig();
    return;
  }


  if (lc.startsWith("car ")) {
    int idx = line.substring(4).toInt();  // 1..N
    if (idx < 1 || idx > PROFILE_COUNT) { Serial.println("[CAR] bad index. Use CARS."); return; }
    bool wasConn = isConnectedOBD();
    setActiveProfile(idx - 1, !wasConn);   // jak było połączone -> nie rozłączaj
    return;
  }

  // ====== TESTALL ======
  if (lc == "testall") { testAllOnce(); return; }

  // ====== FACTORY ======
  if (lc == "factory") {
    Serial.println("[FACTORY] Reset current profile settings to defaults.");
    Serial.println("[FACTORY] Confirm with: FACTORY YES");
    return;
  }
  if (lc == "factory yes") {
    if (g_client && g_client->isConnected()) g_client->disconnect();
    g_charTX = nullptr;
    g_charRX = nullptr;
    factoryResetProfile(true);
    return;
  }


if (lc == "status") {
  bool conn = (g_client && g_client->isConnected());

  Serial.printf("[STATUS] MAC=%s HEADER=0x%03X BLE=%s\n",
                g_mac.c_str(), g_header, conn ? "ON" : "OFF");

  // --- DODANE: info o profilu auta ---
  Serial.printf("[CAR] %d/%d key=%s name=%s ns=%s\n",
                g_profileIdx + 1, PROFILE_COUNT,
                PROFILES[g_profileIdx].key,
                PROFILES[g_profileIdx].name,
                g_nvsNS.c_str());

  showPidStatus();
  showDidStatus();

  Serial.printf("[LOGIO] %s\n", g_logIO ? "ON" : "OFF");
  Serial.printf("[TEST] %s\n", g_testMode ? "ON" : "OFF");
  Serial.printf("[BL] %u %%\n", (unsigned)g_blPct);
  Serial.printf("[ROT] %u\n", (unsigned)g_rotation);
  return;
}

  // ====== VIN ======
  if (lc == "vin" || lc == "vin?") {
    if (!connected()) { Serial.println("[VIN] not connected"); return; }

    String vin;
    if (readVIN(vin)) {
      Serial.print("VIN: ");
      Serial.println(vin);
    } else {
      Serial.println("[VIN] NO DATA");
    }
    return;
  }


  // scan [seconds]
  if (lc == "scan" || lc.startsWith("scan ")) {
    int sec = 5;
    if (lc.startsWith("scan ")) {
      int t = line.substring(5).toInt();
      if (t > 0 && t <= 30) sec = t;
    }
    doScan(sec);
    return;
  }

  // ====== SCAN CACHE: LIST / MAC# / CONN# ======
  if (lc == "list") {
    if (g_scanCount <= 0) {
      Serial.println("[SCAN] empty. Run SCAN first.");
      return;
    }
    Serial.printf("[SCAN] cached %d devices:\n", g_scanCount);
    for (int i = 0; i < g_scanCount; i++) {
      Serial.printf("  %2d) %s  Name=%s\n", i + 1, g_scanMac[i].c_str(), g_scanName[i].c_str());
    }
    return;
  }

  // use <n>  (ustawia MAC z ostatniego scan)
  if (lc.startsWith("use ")) {
    if (g_scanCount <= 0) { Serial.println("[SCAN] empty. Run SCAN first."); return; }
    int idx = line.substring(4).toInt();
    if (idx < 1 || idx > g_scanCount) { Serial.println("[USE] bad index. Use LIST."); return; }
    String m = g_scanMac[idx - 1];
    m.toUpperCase();
    if (!isMacStringValid(m) || m == "00:00:00:00:00:00") { Serial.println("[USE] invalid cached MAC"); return; }
    g_mac = m;
    Serial.printf("[MAC] set from SCAN #%d -> %s\n", idx, g_mac.c_str());
    return;
  }

  // conn <n> [save]  (ustawia MAC z listy i od razu CONNECT)
  if (lc.startsWith("conn ")) {
    if (g_scanCount <= 0) { Serial.println("[SCAN] empty. Run SCAN first."); return; }
    String rest = line.substring(5);
    rest.trim();

    bool doSave = false;
    int sp = rest.indexOf(' ');
    String idxStr = rest;
    if (sp > 0) {
      idxStr = rest.substring(0, sp);
      String tail = rest.substring(sp + 1);
      tail.trim();
      String tailLc = tail; tailLc.toLowerCase();
      if (tailLc == "save") doSave = true;
    }

    int idx = idxStr.toInt();
    if (idx < 1 || idx > g_scanCount) { Serial.println("[CONN] bad index. Use LIST."); return; }

    String m = g_scanMac[idx - 1];
    m.toUpperCase();
    if (!isMacStringValid(m) || m == "00:00:00:00:00:00") { Serial.println("[CONN] invalid cached MAC"); return; }

    if (isConnectedOBD()) {
      String current = g_mac; current.toUpperCase();
      if (m != current) {
        Serial.printf("[CONN] already connected (current MAC=%s). DISCONNECT first to switch.\n", current.c_str());
        return;
      }
      Serial.println("[BLE] already connected");
      if (doSave) {
        showMsg("SAVING MAC", "Restart...", COL_TITLE, COL_TEXT);
        delay(250);
        saveMacAndRestart();
      }
      return;
    }

    String oldMac = g_mac;
    g_mac = m;
    Serial.printf("[MAC] set from SCAN #%d -> %s\n", idx, g_mac.c_str());

    showMsg("Connecting BLE", "Wait...");
    if (!connectAndDiscover(g_mac.c_str())) {
      showMsg("BLE FAIL", "Check ELM", COL_RED);
      Serial.println("[BLE] connect failed");
      g_mac = oldMac;
      Serial.printf("[MAC] restore -> %s\n", g_mac.c_str());
      return;
    }

    showMsg("BLE OK", "Init ELM...");
    elmInit();
    showMsg("ELM READY", "Start");
    Serial.println("[BLE] connected + ELM ready");

    g_dashStaticDrawn = false;
    g_burnStaticDrawn = false;
    g_successStaticDrawn = false;
    g_abortStaticDrawn = false;

    if (doSave) {
      showMsg("SAVING MAC", "Restart...", COL_TITLE, COL_TEXT);
      delay(250);
      saveMacAndRestart();
    }
    return;
  }

  if (lc.startsWith("mac#")) {
    // akceptuje: "MAC# 3" albo "MAC#3"
    String rest = line.substring(4);
    rest.trim();
    int idx = rest.toInt();

    if (idx < 1 || idx > g_scanCount) {
      Serial.println("[MAC#] bad index. Use LIST.");
      return;
    }

    String m = g_scanMac[idx - 1];
    m.toUpperCase();

    if (!isMacStringValid(m)) {
      Serial.println("[MAC#] invalid cached MAC");
      return;
    }

    g_mac = m;
    Serial.printf("[MAC] set from SCAN #%d -> %s\n", idx, g_mac.c_str());
    return;
  }

  if (lc.startsWith("conn#")) {
    // akceptuje: "CONN# 3" albo "CONN#3"
    // "CONN# 3 SAVE" -> po udanym polaczeniu zapisuje MAC i robi restart

    String rest = line.substring(5);
    rest.trim();

    bool doSave = false;
    int sp = rest.indexOf(' ');
    String idxStr = rest;

    if (sp > 0) {
      idxStr = rest.substring(0, sp);
      String tail = rest.substring(sp + 1);
      tail.trim();
      if (tail.equalsIgnoreCase("SAVE")) doSave = true;
    }

    int idx = idxStr.toInt();
    if (idx < 1 || idx > g_scanCount) {
      Serial.println("[CONN#] bad index. Use LIST.");
      return;
    }

    String m = g_scanMac[idx - 1];
    m.toUpperCase();

    // odetnij 00:00... oraz ewidentnie zly MAC
    if (!isMacStringValid(m) || m == "00:00:00:00:00:00") {
      Serial.println("[CONN#] invalid cached MAC");
      return;
    }

    // === ROLLBACK: zapamietaj stary MAC na wypadek fail ===
    if (isConnectedOBD()) {
      String current = g_mac;
      current.toUpperCase();

      if (m != current) {
        Serial.printf("[CONN#] already connected (current MAC=%s). DISCONNECT first to switch.\n", current.c_str());
        return;
      }

      Serial.println("[BLE] already connected");
      if (doSave) {
        showMsg("SAVING MAC", "Restart...", COL_TITLE, COL_TEXT);
        delay(250);
        saveMacAndRestart();
      }
      return;
    }

    // dopiero tu ustaw g_mac i próbuj łączyć
    String oldMac = g_mac;
    g_mac = m;
    Serial.printf("[MAC] set from SCAN #%d -> %s\n", idx, g_mac.c_str());


    showMsg("Connecting BLE", "Wait...");

    if (!connectAndDiscover(g_mac.c_str())) {
      showMsg("BLE FAIL", "Check ELM", COL_RED);
      Serial.println("[BLE] connect failed");

      // === rollback MAC, zeby auto-reconnect nie wlazl na zly adres ===
      g_mac = oldMac;
      Serial.printf("[MAC] restore -> %s\n", g_mac.c_str());
      return;
    }

    // sukces
    showMsg("BLE OK", "Init ELM...");
    elmInit();
    showMsg("ELM READY", "Start");
    Serial.println("[BLE] connected + ELM ready");

    g_dashStaticDrawn = false;
    g_burnStaticDrawn = false;
    g_successStaticDrawn = false;
    g_abortStaticDrawn = false;

    // jeśli było SAVE -> zapis + restart (TYLKO po udanym polaczeniu)
    if (doSave) {
      showMsg("SAVING MAC", "Restart...", COL_TITLE, COL_TEXT);
      delay(250);
      saveMacAndRestart();  // zapis + reset
    }

    return;
  }




  if (lc == "connect") {
    if (g_client && g_client->isConnected()) { Serial.println("[BLE] already connected"); return; }

    showMsg("Connecting BLE", "Wait...");

    if (!bleTargetPresent(g_mac.c_str(), 2)) {
      showMsg("BLE FAIL", "MAC not seen", COL_RED);
      Serial.println("[BLE] Target MAC not seen -> skip connect");
      return;
    }

    if (!connectAndDiscover(g_mac.c_str())) {
      showMsg("BLE FAIL", "Check ELM", COL_RED);
      Serial.println("[BLE] connect failed");
    } else {
      showMsg("BLE OK", "Init ELM...");
      elmInit();
      showMsg("ELM READY", "Start");
      Serial.println("[BLE] connected + ELM ready");
      g_dashStaticDrawn = false;
      g_burnStaticDrawn = false;
      g_successStaticDrawn = false;
    }
    return;
  }

  if (lc == "disconnect") {
    if (g_client && g_client->isConnected()) g_client->disconnect();
    Serial.println("[BLE] Disconnected");
    return;
  }

  // bl [0-100] / bl save
  if (lc == "bl" || lc == "bl?") { Serial.printf("[BL] %u %%\n", (unsigned)g_blPct); return; }
  if (lc.startsWith("bl ")) {
    String rest = line.substring(3); rest.trim();
    String restLc = rest; restLc.toLowerCase();
    if (restLc == "save") { saveBacklight(); return; }
    int pct = rest.toInt();
    if (pct < 0 || pct > 100) { Serial.println("[BL] range 0..100"); return; }
    setBacklightPct(pct);
    Serial.printf("[BL] set %d %%\n", pct);
    return;
  }
  if (lc.startsWith("bl=")) {
    String v = line.substring(3); v.trim();
    int pct = v.toInt();
    if (pct < 0 || pct > 100) { Serial.println("[BL] range 0..100"); return; }
    setBacklightPct(pct);
    Serial.printf("[BL] set %d %%\n", pct);
    return;
  }
  if (lc == "save_bl") { saveBacklight(); return; }

  // rot [0-3|180] / rot save
  if (lc == "rot" || lc == "rot?") { Serial.printf("[ROT] %u\n", (unsigned)g_rotation); return; }
  if (lc.startsWith("rot ")) {
    String rest = line.substring(4); rest.trim();
    String restLc = rest; restLc.toLowerCase();
    if (restLc == "save") { saveRotation(); return; }
    if (restLc == "180") {
      // to samo co ROT180
      g_rotation = (g_rotation + 2) & 3;
      tftLock();
      tft.setRotation(g_rotation);
      tft.fillScreen(COL_BG);
      g_dashStaticDrawn = false; g_burnStaticDrawn = false; g_successStaticDrawn = false;
      tftUnlock();
      Serial.printf("[ROT] now %u (180deg)\n", (unsigned)g_rotation);
      return;
    }
    int r = rest.toInt();
    if (r < 0 || r > 3) { Serial.println("[ROT] range 0..3"); return; }
    g_rotation = (uint8_t)r;
    tftLock();
    tft.setRotation(g_rotation);
    tft.fillScreen(COL_BG);
    g_dashStaticDrawn = false; g_burnStaticDrawn = false; g_successStaticDrawn = false;
    tftUnlock();
    Serial.printf("[ROT] set to %u\n", (unsigned)g_rotation);
    return;
  }
  if (lc == "rot180") {
    g_rotation = (g_rotation + 2) & 3;
    tftLock();
    tft.setRotation(g_rotation);
    tft.fillScreen(COL_BG);
    g_dashStaticDrawn = false; g_burnStaticDrawn = false; g_successStaticDrawn = false;
    tftUnlock();
    Serial.printf("[ROT] now %u (180deg)\n", (unsigned)g_rotation);
    return;
  }
  if (lc.startsWith("rot=")) {
    int r = line.substring(4).toInt();
    if (r < 0 || r > 3) { Serial.println("[ROT] range 0..3"); return; }
    g_rotation = (uint8_t)r;
    tftLock();
    tft.setRotation(g_rotation);
    tft.fillScreen(COL_BG);
    g_dashStaticDrawn = false; g_burnStaticDrawn = false; g_successStaticDrawn = false;
    tftUnlock();
    Serial.printf("[ROT] set to %u\n", (unsigned)g_rotation);
    return;
  }
  if (lc == "save_rot") { saveRotation(); return; }

  // log / logio
  if (lc == "log" || lc == "log?" || lc == "logio" || lc == "logio?") {
    Serial.printf("[LOGIO] %s\n", g_logIO ? "ON" : "OFF");
    return;
  }
  if (lc == "log on" || lc == "logio on")  { g_logIO=true;  Serial.println("[LOGIO] ON");  return; }
  if (lc == "log off" || lc == "logio off") { g_logIO=false; Serial.println("[LOGIO] OFF"); return; }

  if (lc == "test" || lc == "test?") { Serial.printf("[TEST] %s\n", g_testMode ? "ON" : "OFF"); return; }
  if (lc == "test on")  { setTestMode(true);  g_dashStaticDrawn=false; g_burnStaticDrawn=false; g_successStaticDrawn=false; return; }
  if (lc == "test off") { setTestMode(false); g_dashStaticDrawn=false; g_burnStaticDrawn=false; g_successStaticDrawn=false; return; }

  // mac / mac save / mac <addr>
  if (lc == "mac" || lc == "mac?") { Serial.print("[MAC] "); Serial.println(g_mac); return; }
  if (lc.startsWith("mac ")) {
    String rest = line.substring(4); rest.trim();
    String restLc = rest; restLc.toLowerCase();
    if (restLc == "save") { saveMacAndRestart(); return; }

    String m = rest; m.trim(); m.toUpperCase();
    if (!isMacStringValid(m) || m == "00:00:00:00:00:00") {
      Serial.println("[MAC] invalid (AA:BB:CC:DD:EE:FF), not 00:00:00:00:00:00");
      return;
    }
    g_mac = m;
    Serial.print("[MAC] set to ");
    Serial.println(g_mac);
    return;
  }

  if (line.equalsIgnoreCase("MAC?")) { Serial.print("[MAC] "); Serial.println(g_mac); return; }
  if (lc.startsWith("mac=")) {
    String m = line.substring(4); m.trim(); m.toUpperCase();

    if (!isMacStringValid(m) || m == "00:00:00:00:00:00") {
      Serial.println("[MAC] invalid (AA:BB:CC:DD:EE:FF), not 00:00:00:00:00:00");
      return;
    }

    g_mac = m;
    Serial.print("[MAC] set to ");
    Serial.println(g_mac);
    return;
  }

  if (lc == "save_mac") { saveMacAndRestart(); return; }

  // header / header save / header <hex>
  if (lc == "header" || lc == "header?") { Serial.printf("[HEADER] 0x%03X\n", g_header); return; }
  if (lc.startsWith("header ")) {
    String rest = line.substring(7); rest.trim();
    String restLc = rest; restLc.toLowerCase();
    if (restLc == "save") { saveHeader(); return; }

    uint32_t h;
    if (!parseHexU32(rest, h)) { Serial.println("[HEADER] invalid hex"); return; }

    g_header = h;
    g_lastHdr = 0xFFFFFFFF;
    updateObdFuncHeader();

    if (g_header <= 0x7FF)
      Serial.printf("[HEADER] set to 0x%03lX  FUNC=0x%03lX\n",
                    (unsigned long)g_header, (unsigned long)g_obdFuncHeader);
    else
      Serial.printf("[HEADER] set to 0x%08lX  FUNC=0x%08lX\n",
                    (unsigned long)g_header, (unsigned long)g_obdFuncHeader);
    return;
  }

  if (line.equalsIgnoreCase("HEADER?")) { Serial.printf("[HEADER] 0x%03X\n", g_header); return; }
  if (lc.startsWith("setheader")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println("[HEADER] usage: SETHEADER 7E0  (or 18DAF110)"); return; }

    String hex = line.substring(sp + 1); hex.trim();

    uint32_t h;
    if (!parseHexU32(hex, h)) { Serial.println("[HEADER] invalid hex"); return; }

    g_header = h;
    g_lastHdr = 0xFFFFFFFF;
    updateObdFuncHeader();

    if (g_header <= 0x7FF)
      Serial.printf("[HEADER] set to 0x%03lX  FUNC=0x%03lX\n",
                    (unsigned long)g_header, (unsigned long)g_obdFuncHeader);
    else
      Serial.printf("[HEADER] set to 0x%08lX  FUNC=0x%08lX\n",
                    (unsigned long)g_header, (unsigned long)g_obdFuncHeader);

    return;
  }

  if (lc == "saveheader") { saveHeader(); return; }

  // did / did save / did <name> <hex>
  if (lc == "did" || lc == "did?") { showDidStatus(); return; }
  if (lc.startsWith("did ")) {
    String rest = line.substring(4); rest.trim();
    String restLc = rest; restLc.toLowerCase();
    if (restLc == "save") { saveDIDs(); return; }

    int s1 = rest.indexOf(' ');
    if (s1 < 0) { Serial.println("[DID] usage: did <soot|km|burn|state|prog|oil> 0xXXXX"); return; }
    String name = rest.substring(0, s1); name.trim();
    String nameLc = name; nameLc.toLowerCase();
    String hex = rest.substring(s1 + 1); hex.trim();
    uint16_t did; if (!parseHexU16(hex, did)) { Serial.println("[DID] bad hex"); return; }

    if      (nameLc=="soot")  g_did_soot  = did;
    else if (nameLc=="km")    g_did_km    = did;
    else if (nameLc=="burn")  g_did_burn  = did;
    else if (nameLc=="state") g_did_state = did;
    else if (nameLc=="prog")  g_did_prog  = did;
    else if (nameLc=="oil")   g_did_oil_uds = did;
    else { Serial.println("[DID] name must be soot|km|burn|state|prog|oil"); return; }

    Serial.printf("[DID] %s = 0x%04X\n", nameLc.c_str(), did);
    return;
  }

  if (line.equalsIgnoreCase("DID?")) { showDidStatus(); return; }

  if (lc.startsWith("setdid")) {
    int s1=line.indexOf(' '), s2=(s1<0?-1:line.indexOf(' ', s1+1));
    if (s1<0||s2<0){ Serial.println("[DID] usage: SETDID <SOOT|KM|BURN|STATE|PROG|OIL> 0xXXXX"); return; }
    String name=line.substring(s1+1,s2); name.trim(); name.toUpperCase();
    String hex=line.substring(s2+1); hex.trim();
    uint16_t did; if(!parseHexU16(hex,did)){ Serial.println("[DID] bad hex"); return; }

    if      (name=="SOOT")  g_did_soot  = did;
    else if (name=="KM")    g_did_km    = did;
    else if (name=="BURN")  g_did_burn  = did;
    else if (name=="STATE") g_did_state = did;
    else if (name=="PROG")  g_did_prog  = did;
    else if (name=="OIL")   g_did_oil_uds = did;
    else { Serial.println("[DID] name must be SOOT|KM|BURN|STATE|PROG|OIL"); return; }

    Serial.printf("[DID] %s = 0x%04X\n", name.c_str(), did);
    return;
  }

  if (lc == "savedid") { saveDIDs(); return; }

  // PID commands
  // pid / pid save / pid <name> <hex>
  if (lc == "pid" || lc == "pid?") { showPidStatus(); return; }
  if (lc.startsWith("pid ")) {
    String rest = line.substring(4); rest.trim();
    String restLc = rest; restLc.toLowerCase();
    if (restLc == "save") { savePIDs(); return; }

    int s1 = rest.indexOf(' ');
    if (s1 < 0) { Serial.println("[PID] usage: pid <vbat|ect|oil> 0xXX"); return; }
    String name = rest.substring(0, s1); name.trim();
    String nameLc = name; nameLc.toLowerCase();
    String hex = rest.substring(s1 + 1); hex.trim();
    uint8_t pid; if (!parseHexU8(hex, pid)) { Serial.println("[PID] bad hex"); return; }

    if      (nameLc=="vbat")  g_pid_vbat = pid;
    else if (nameLc=="ect")   g_pid_ect  = pid;
    else if (nameLc=="oil")   g_pid_oil  = pid;
    else { Serial.println("[PID] name must be vbat|ect|oil"); return; }

    Serial.printf("[PID] %s = 0x%02X\n", nameLc.c_str(), pid);
    return;
  }

  if (line.equalsIgnoreCase("PID?")) { showPidStatus(); return; }

  if (lc.startsWith("setpid")) {
    int s1=line.indexOf(' '), s2=(s1<0?-1:line.indexOf(' ', s1+1));
    if (s1<0||s2<0){ Serial.println("[PID] usage: SETPID <VBAT|ECT|OIL> 0xXX"); return; }
    String name=line.substring(s1+1,s2); name.trim(); name.toUpperCase();
    String hex=line.substring(s2+1); hex.trim();
    uint8_t pid; if(!parseHexU8(hex,pid)){ Serial.println("[PID] bad hex"); return; }

    if      (name=="VBAT")  g_pid_vbat = pid;
    else if (name=="ECT")   g_pid_ect  = pid;
    else if (name=="OIL")   g_pid_oil  = pid;
    else { Serial.println("[PID] name must be VBAT|ECT|OIL"); return; }

    Serial.printf("[PID] %s = 0x%02X\n", name.c_str(), pid);
    return;
  }

  if (lc == "savepid") { savePIDs(); return; }

  // quick read commands
  if (lc == "vbat") {
    if (!connected()) { Serial.println("[VBAT] not connected"); return; }
    float v; if(queryVBAT(v)) Serial.printf("[VBAT] (01 %02X) %.3f V\n", g_pid_vbat, v); else Serial.println("[VBAT] NO DATA");
    return;
  }

  if (lc == "oil") {
    if (!connected()) { Serial.println("[OIL] not connected"); return; }
    float t;
    if (queryOIL(t) && tempValid(t)) Serial.printf("[OIL] (01 %02X / 22 %04X) %.1f C\n", g_pid_oil, g_did_oil_uds, t);
    else Serial.println("[OIL] NO DATA");
    return;
  }

  if (lc == "ect") {
    if (!connected()) { Serial.println("[ECT] not connected"); return; }
    float t;
    if (queryECT(t) && tempValid(t)) Serial.printf("[ECT] (01 %02X) %.1f C\n", g_pid_ect, t);
    else Serial.println("[ECT] NO DATA");
    return;
  }


  if (lc == "req") {
    if (!connected()) { Serial.println("[REQ] not connected"); return; }
    ByteBuf b;
    if (query22_bytes(g_did_burn, b, 1000) && b.n >= 4) {
      bool req = burnReqFrom_2220FA(b);
      Serial.printf("[REQ] %s (A=0x%02X)\n", req ? "REQ" : "NOTREQ", (unsigned)b.b[3]);
    } else Serial.println("[REQ] NO DATA");
    return;
  }


  if (lc == "state") {
    if (!connected()) { Serial.println("[STATE] not connected"); return; }
    ByteBuf b;
    if (query22_bytes(g_did_state, b, 1000) && b.n >= 4) {
      Serial.printf("[STATE] %s (A=0x%02X)\n", regenStateFrom_2220F2(b.b[3]), (unsigned)b.b[3]);
    } else Serial.println("[STATE] NO DATA");
    return;
  }


  if (lc == "read22" || lc.startsWith("read22 ")) {
    if (!connected()) { Serial.println("[READ22] not connected"); return; }
    String rest = (lc == "read22") ? String("") : line.substring(6);
    rest.trim();
    if (!rest.length()) { Serial.println("[READ22] usage: read22 0x336a"); return; }
    if (!(rest.startsWith("0x") || rest.startsWith("0X"))) rest = String("0x") + rest;
    uint16_t did; if (!parseHexU16(rest, did)) { Serial.println("[READ22] bad DID"); return; }

    ByteBuf b;
    if (query22_bytes(did, b, 2500)) {
      Serial.print("[READ22] ");
      for (uint8_t i = 0; i < b.n; i++) Serial.printf("%02X ", b.b[i]);
      Serial.println();
    } else Serial.println("[READ22] NO DATA");
    return;
  }



  // raw commands
  if (lc == "at" || lc.startsWith("at ") || isxdigit((unsigned char)line[0])) {
    if(!connected()){ Serial.println("[CMD] not connected"); return; }
    String cmdLine = line;
    cmdLine.trim();
    cmdLine.toUpperCase();
    String out;
    if (sendELMAndWait(cmdLine.c_str(), out, 2000)) {
      Serial.println("[CMD OK]");
      // pokaż treść odpowiedzi NORMALNIE:
      Serial.println(out);
    } else {
      Serial.println("[CMD NO DATA]");
    }
    return;
  }


  Serial.println("[ERR] Unknown command. Type help.");
}

// ===================== MAIN TASK (read + draw) =====================
static void taskLoop(void*) {
  uint32_t lastFast = 0;
  uint32_t lastSlow = 0;
  uint32_t lastDraw = 0;

  int soot = -1, km = -1;
  float vbat = -1, oil = TEMP_NODATA, ect = TEMP_NODATA;

  bool burnReq = false;
  uint8_t stateA = 0;
  String stateStr = "--";

  for (;;) {
    uint32_t now = millis();
    stopSplashIfDone(now);

    // ===================== TEST MODE (bez BLE/ELM) =====================
    if (g_testMode) {
      genTestData(soot, km, burnReq, vbat, oil, ect, stateA);
      stateStr = regenStateFrom_2220F2(stateA);

      uint8_t s = (stateA & 0x07);
      bool actively = (s == 7);
      bool warming  = (s == 6);

      // Wejście na BURN tylko przy Actively
      if (!g_regenLatched && actively) {
        g_regenLatched = true;
        g_regenLastSeenMs = now;
        clearPostScreens();
      }

      // Jak już jesteśmy na BURN, zostajemy dopóki: Actively/Warming/burnReq
      bool regenNow = g_regenLatched ? (actively || warming || burnReq) : actively;

      if (g_regenLatched && regenNow) {
        g_regenLastSeenMs = now;
        clearPostScreens();
      }

      // Wyjście dopiero gdy nie ma warunków przez REGEN_HOLD_MS
      if (g_regenLatched && !regenNow) {
        if (now - g_regenLastSeenMs > REGEN_HOLD_MS) {
          g_regenLatched = false;

          // TEST: decyzja na podstawie symulowanego km
          int kmNow = km;
          bool completed = ((kmNow >= 0 && kmNow <= REGEN_DONE_KM_MAX) || (soot >= 0 && soot <= REGEN_DONE_SOOT_MAX));
          if (completed) startSuccess(now);
          else           startAbort(now);
        }
      }

      UiMode newMode;
      if (g_successUntilMs != 0 && now < g_successUntilMs)      newMode = UI_SUCCESS;
      else if (g_abortUntilMs != 0 && now < g_abortUntilMs)     newMode = UI_ABORT;
      else if (g_regenLatched)                                   newMode = UI_BURN;
      else                                                       newMode = UI_DASH;

      tftLock();

      if (newMode != g_mode) {
        g_mode = newMode;
        g_dashStaticDrawn = false;
        g_burnStaticDrawn = false;
        g_successStaticDrawn = false;
        g_abortStaticDrawn = false;
      }

      if (g_mode == UI_SUCCESS) {
        if (!g_successStaticDrawn) uiSuccessDrawStatic();
      } else if (g_mode == UI_ABORT) {
        if (!g_abortStaticDrawn) uiAbortDrawStatic();
      } else if (g_mode == UI_BURN) {
        if (!g_burnStaticDrawn) uiBurnDrawStatic();
        uiBurnUpdateDynamic(soot, ect, stateStr, now);
      } else {
        if (!g_dashStaticDrawn) uiDashDrawStatic();
        String st = burnReq ? "REQ" : stateStr;
        uiDashUpdateDynamic(soot, km, vbat, oil, ect, st);
      }

      tftUnlock();
      vTaskDelay(pdMS_TO_TICKS(80));
      continue;
    }

    // ===================== NORMAL MODE (BLE/ELM) =====================
    if (!(g_client && g_client->isConnected() && g_charRX && g_charTX)) {
      if (now < g_pauseReconnectUntilMs) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }

      static uint32_t lastTry = 0;
      if (now - lastTry > 2000) {
        lastTry = now;
        static uint32_t lastNoObdMsg = 0;

      if (!bleTargetPresent(g_mac.c_str(), 1)) {
        // licznik prób "nie widzę MAC"
        if (g_macNotSeenCount < 0xFFFFFFFFUL) g_macNotSeenCount++;

        // wypisz maks raz na MAC_NOT_SEEN_LOG_EVERY_MS
        if (g_lastMacNotSeenLogMs == 0 || (now - g_lastMacNotSeenLogMs >= MAC_NOT_SEEN_LOG_EVERY_MS)) {
          g_lastMacNotSeenLogMs = now;
          Serial.printf("[BLE] Target MAC not seen (%lu). Use SCAN/LIST and CONN#.\n",
                        (unsigned long)g_macNotSeenCount);
        }

        // ekran możesz nadal pokazywać co kilka sekund
        if (now - lastNoObdMsg > 3000) {
          showMsg("OBD / ELM OFF", "Waiting for BLE...", COL_TITLE, COL_TEXT);
          lastNoObdMsg = now;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      } else {
        // jak tylko zacznie być widoczny -> zeruj liczniki (żeby nie wisiało "not seen (999)")
        g_macNotSeenCount = 0;
        g_lastMacNotSeenLogMs = 0;
      }

        lastNoObdMsg = 0;

        Serial.println("[BLE] Reconnect attempt...");
        showMsg("Reconnecting", "BLE...");

        if (connectAndDiscover(g_mac.c_str())) {
          showMsg("BLE OK", "Init ELM...");
          elmInit();
          showMsg("ELM READY", "Start");
          Serial.println("[BLE] Reconnected");
          g_macNotSeenCount = 0;
          g_lastMacNotSeenLogMs = 0;
          g_dashStaticDrawn = false;
          g_burnStaticDrawn = false;
          g_successStaticDrawn = false;
          g_abortStaticDrawn = false;
          clearPostScreens();
        } else {
          showMsg("BLE FAIL", "Check ELM", COL_RED);
        }
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    elmAutoRecoverTick(now);

    // FAST reads ~300ms
    if (now - lastFast >= 300) {
      lastFast = now;

      ByteBuf b;
      if (query22_bytes(g_did_soot, b, 900)) soot = sootPercentFrom_22336A(b);
      if (query22_bytes(g_did_burn, b, 900)) burnReq = burnReqFrom_2220FA(b);

      if (query22_bytes(g_did_state, b, 900) && b.n >= 4) {
        stateA = b.b[3];
        stateStr = regenStateFrom_2220F2(stateA);
      } else {
        stateA = 0;
        stateStr = "--";
      }
    }

    // SLOW reads ~1200ms
    if (now - lastSlow >= 1200) {
      lastSlow = now;

      uint16_t v16;
      if (query22_u16(g_did_km, v16, 900)) km = (int)v16;

      queryVBAT(vbat);
      queryOIL(oil);
      queryECT(ect);
    }

    // DRAW ~90ms
    if (now - lastDraw >= 90) {
      lastDraw = now;

      uint8_t s = (stateA & 0x07);
      bool actively = (s == 7);
      bool warming  = (s == 6);

      // Wejście na BURN tylko przy Actively
      if (!g_regenLatched && actively) {
        g_regenLatched = true;
        g_regenLastSeenMs = now;
        clearPostScreens();
      }

      bool regenNow = g_regenLatched ? (actively || warming || burnReq) : actively;

      if (g_regenLatched && regenNow) {
        g_regenLastSeenMs = now;
        clearPostScreens();
      }

      // Wyjście dopiero gdy brak warunków przez REGEN_HOLD_MS
      if (g_regenLatched && !regenNow) {
        if (now - g_regenLastSeenMs > REGEN_HOLD_MS) {
          g_regenLatched = false;

          // NORMAL: odśwież KM tuż przed decyzją (pewniej niż cache)
          int kmNow = km;
          uint16_t v16;
          if (query22_u16(g_did_km, v16, 900)) kmNow = (int)v16;

          bool completed = ((kmNow >= 0 && kmNow <= REGEN_DONE_KM_MAX) ||
                            (soot >= 0 && soot <= REGEN_DONE_SOOT_MAX));

          if (completed) startSuccess(now);
          else           startAbort(now);
        }
      }

      UiMode newMode;
      if (g_successUntilMs != 0 && now < g_successUntilMs)      newMode = UI_SUCCESS;
      else if (g_abortUntilMs != 0 && now < g_abortUntilMs)     newMode = UI_ABORT;
      else if (g_regenLatched)                                   newMode = UI_BURN;
      else                                                       newMode = UI_DASH;

if (!splashOn(now)) {
        tftLock();

        if (newMode != g_mode) {
          g_mode = newMode;
          g_dashStaticDrawn = false;
          g_burnStaticDrawn = false;
          g_successStaticDrawn = false;
          g_abortStaticDrawn = false;
        }

        if (g_mode == UI_SUCCESS) {
          if (!g_successStaticDrawn) uiSuccessDrawStatic();
        } else if (g_mode == UI_ABORT) {
          if (!g_abortStaticDrawn) uiAbortDrawStatic();
        } else if (g_mode == UI_BURN) {
          if (!g_burnStaticDrawn) uiBurnDrawStatic();
          uiBurnUpdateDynamic(soot, ect, stateStr, now);
        } else {
          if (!g_dashStaticDrawn) uiDashDrawStatic();
          String st = burnReq ? "REQ" : stateStr;
          uiDashUpdateDynamic(soot, km, vbat, oil, ect, st);
        }

        tftUnlock();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, BL_INVERT ? HIGH : LOW);

  Serial.begin(115200);
  delay(50);
  Serial.println("\n== ESP32 ELM327 BLE + ST7789 1.47 172x320 (landscape 320x172) ==");

  g_elmMutex = xSemaphoreCreateMutex();
  g_tftMutex = xSemaphoreCreateMutex();
  g_bleMutex = xSemaphoreCreateMutex();

  // ===== profil startowy (MUSI być przed loadSettings) =====
    ensureGlobalNvsVersion();
  g_profileIdx = loadActiveProfile();                           // 0 = np. Astra (jak w PROFILES)
  g_nvsNS = PROFILES[g_profileIdx].nvsNS;     // namespace dla tego profilu

  loadSettings();

  tft.init();
  tft.setRotation(g_rotation);
  tft.setSwapBytes(true);
  tft.fillScreen(COL_BG);

  initBacklightPwm();
  setBacklightPct(g_blPct);

  drawOpelSplashZoom(0, 90, 0, 0);  // tylko narysuj
  startSplash(5000);               // trzymaj logo 5s (w tle działa BLE)

  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  xTaskCreatePinnedToCore(taskLoop, "main", 8192, nullptr, 1, &g_task, 1);

  printHelp();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (sLine.length()) { handleConsoleLine(sLine); sLine = ""; }
    } else {
      sLine += c;
      if (sLine.length() > 200) sLine.remove(0, 50);
    }
  }
  delay(1);
}
