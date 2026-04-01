Projekt został przygotowany pod PlatformIO / VS Code.

Struktura:
- platformio.ini
- src/main.cpp
- include/car_profiles.h
- include/isotp_helpers.h
- include/opel_240.h
- include/tft_setup.h

Uwagi:
1. Rdzeń logiki nadal siedzi w src/main.cpp, ale projekt jest już gotowy do otwarcia w PlatformIO.
2. Dodałem prototypy funkcji, bo .ino po zamianie na .cpp tego wymaga.
3. Ustawienia TFT_eSPI są ładowane przez build_flags: -include include/tft_setup.h
4. Jeśli Twoja płytka nie jest "esp32dev", zmień board w platformio.ini.
5. Jeśli używasz innego wyświetlacza albo innych pinów, popraw include/tft_setup.h.
