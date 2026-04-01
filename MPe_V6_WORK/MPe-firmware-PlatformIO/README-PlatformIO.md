# MPe-firmware - przewodnik do wersji PlatformIO

Ten katalog jest portem oryginalnego projektu Arduino z archiwum `MPe_V6_open_source-master.zip`.

## Dla uzytkownika
- Projekt jest przygotowany pod `PlatformIO` i domyslnie buduje sie dla `Arduino Uno / ATmega328P`.
- Kod jest juz uporzadkowany do pracy w PlatformIO i kompiluje sie poprawnie.
- Firmware jest bardzo blisko limitu pamieci Flash mikrokontrolera, wiec kazda nowa funkcja moze juz nie wejsc.

## Najwazniejsze pliki
- `platformio.ini` - glowna konfiguracja PlatformIO: plytka, framework, predkosc monitora i flagi buildu.
- `README-PlatformIO.md` - ten opis projektu.
- `README-original.md` - krotka informacja z oryginalnego projektu.

## Mapa projektu
- `src/` - aktywne pliki firmware. To jest glowny kod programu.
- `include/` - naglowki projektu, czyli stale, konfiguracja i pomocnicze klasy uzywane przez `src/`.
- `lib/` - lokalne biblioteki dolaczone do projektu, zeby nic nie trzeba bylo osobno instalowac.
- `.pio/` - katalog roboczy PlatformIO z plikami buildu. Tego zwykle sie nie edytuje recznie.
- `.vscode/` - pomocnicza konfiguracja Visual Studio Code.

## Co jest w `src/`
- `MPe-firmware.ino` - glowny plik szkicu: inicjalizacja, zmienne globalne, `setup()` i `loop()`.
- `MPe-power.ino` - pomiary napiecia, pradu, mocy i obsluga ADC.
- `MPe-speed.ino` - pomiar predkosci, dystansu i czasu jazdy.
- `MPe-throttle-pas.ino` - logika manetki, PAS, cruise control i sterowania moca.
- `MPe-temperature.ino` - obsluga czujnikow temperatury i ograniczen termicznych.
- `MPe-brake.ino` - obsluga hamulca elektrycznego.
- `MPe-serial.ino` - komunikacja po UART i informacje wysylane na port szeregowy.
- `MPe-oled-display.ino` - rysowanie informacji na OLED.
- `MPe-oled-buttons.ino` - obsluga przyciskow i menu na wyswietlaczu.
- `MPe-eeprom.ino` - zapis i odczyt ustawien oraz danych z EEPROM.
- `MPe-utilities.ino` - funkcje pomocnicze, przeliczenia i rozne narzedzia.
- `MPe-watchdog.ino` - obsluga watchdog.

## Co jest w `include/`
- `MPe-configuration.h` - najwazniejszy naglowek projektu: wlaczanie funkcji, stale, adresy EEPROM, mapowanie pinow.
- `MPe-PrintWithSC.h` - pomocnicza klasa do wypisywania danych przez `Serial`.
- `Filter.h` - filtr wykladniczy uzywany do wygladzania odczytow.
- `DejaVu_LGC_Sans_Mono_Bold_20.h` - font uzywany przez OLED.

## Co jest w `lib/`
- `PID_v1` - regulator PID.
- `EEPROMex` - rozszerzona obsluga EEPROM.
- `I2C` - lokalna obsluga magistrali I2C.
- `Adafruit_ADS1015` - obsluga przetwornika ADS1115/ADS1015.
- `Adafruit_GFX` - grafika dla wyswietlacza.
- `Adafruit_SSD1306` - obsluga OLED SSD1306.
- `Bounce2` - pomocnicza biblioteka do przyciskow.

## Gdzie co zmieniac
- Chcesz zmienic konfiguracje sprzetu, piny albo stale:
  edytuj `include/MPe-configuration.h`.
- Chcesz zmienic logike jazdy, PAS, manetki albo ograniczen:
  edytuj `src/MPe-throttle-pas.ino`.
- Chcesz zmienic sposob liczenia pomiarow:
  edytuj `src/MPe-power.ino`, `src/MPe-speed.ino`, `src/MPe-temperature.ino`.
- Chcesz zmienic ekran lub menu:
  edytuj `src/MPe-oled-display.ino` i `src/MPe-oled-buttons.ino`.
- Chcesz zmienic ustawienia PlatformIO:
  edytuj `platformio.ini`.

## Build w PlatformIO
Projekt kompiluje sie poleceniem:

```powershell
C:\Users\mpiwowarski\.platformio\penv\Scripts\pio.exe run
```

Monitor portu szeregowego:

```powershell
C:\Users\mpiwowarski\.platformio\penv\Scripts\pio.exe device monitor
```

Jesli trzeba ustawic konkretny port, odkomentuj w `platformio.ini`:

```ini
upload_port = COM5
monitor_port = COM5
```

## Wazne uwagi
- Projekt jest ustawiony na `board = uno`, zgodnie z uwaga w `MPe-configuration.h` o bootloaderze Arduino Uno.
- Dodane jest `-I include`, bo w PlatformIO biblioteki kompiluja sie osobno i musza widziec naglowki projektu.
- Ostatnio sprawdzone w tym repo:
  build zakonczony sukcesem dla srodowiska `uno`.
- Aktualne zuzycie Flash jest bardzo wysokie:
  okolo `98.7%`.
