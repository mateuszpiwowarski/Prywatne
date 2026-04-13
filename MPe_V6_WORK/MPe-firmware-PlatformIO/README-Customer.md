# MPe-firmware - instrukcja dla klienta

Ten firmware ma 3 gotowe profile:
- `3000W`
- `6000W`
- `10000W`

Kazdy profil ma:
- zwykla wersje produkcyjna
- wersje `init`, ktora sluzy do awaryjnej reinicjalizacji EEPROM

## Wymagania
- zainstalowane `PlatformIO`
- sterownik podlaczony do odpowiedniego portu COM

## Ktora wersje wybrac
- `3000W`:
  uzyj srodowiska `uno`
- `6000W`:
  uzyj srodowiska `uno_6000w`
- `10000W`:
  uzyj srodowiska `uno_10000w`

## Zwykle wgranie firmware

Przyklad dla `COM8`.

### Profil 3000W
```powershell
C:\Users\mateu\.platformio\penv\Scripts\platformio.exe run -e uno -t upload --upload-port COM8
```

### Profil 6000W
```powershell
C:\Users\mateu\.platformio\penv\Scripts\platformio.exe run -e uno_6000w -t upload --upload-port COM8
```

### Profil 10000W
```powershell
C:\Users\mateu\.platformio\penv\Scripts\platformio.exe run -e uno_10000w -t upload --upload-port COM8
```

## Co zrobic, jesli ustawienia EEPROM sie uszkodza

Objawy moga byc takie:
- sterownik nie startuje poprawnie
- ekran nic nie pokazuje
- ustawienia wygladaja na uszkodzone

Wtedy trzeba zrobic 2 kroki:

### Recovery 3000W
```powershell
C:\Users\mateu\.platformio\penv\Scripts\platformio.exe run -e uno_init_3000w -t upload --upload-port COM8
C:\Users\mateu\.platformio\penv\Scripts\platformio.exe run -e uno -t upload --upload-port COM8
```

### Recovery 6000W
```powershell
C:\Users\mateu\.platformio\penv\Scripts\platformio.exe run -e uno_init_6000w -t upload --upload-port COM8
C:\Users\mateu\.platformio\penv\Scripts\platformio.exe run -e uno_6000w -t upload --upload-port COM8
```

### Recovery 10000W
```powershell
C:\Users\mateu\.platformio\penv\Scripts\platformio.exe run -e uno_init_10000w -t upload --upload-port COM8
C:\Users\mateu\.platformio\penv\Scripts\platformio.exe run -e uno_10000w -t upload --upload-port COM8
```

## Uwagi
- firmware jest przygotowany pod bootloader `Uno / Optiboot`
- jesli upload nie dziala, najpierw sprawdz poprawny port COM
- jesli problem nadal wystepuje, wykonaj procedure recovery dla odpowiedniego profilu
