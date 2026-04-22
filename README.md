# Minitank Firmware

Firmware pro ESP32, který z malého pásového robota udělá vlastní Wi‑Fi ovládaný tank s webovým rozhraním.

## Nahrání firmware a webu (SPIFFS) - nejdůležitější krok

Aby robot fungoval správně, musíš nahrát 2 věci: firmware a souborový systém (SPIFFS/LittleFS) s webem.

Postup v bodech:

1. Připoj ESP32 přes USB.
2. V PlatformIO spusť Build (ověření překladu).
3. Nahraj firmware: `Upload` (nebo `pio run -t upload`).
4. **POVINNĚ nahraj i webové soubory:** **`Upload Filesystem Image`** (nebo `pio run -t uploadfs`).
5. Otevři Serial Monitor a zkontroluj, že ESP32 naběhlo bez chyb.
6. Připoj se na Wi‑Fi ESP32 a otevři webové rozhraní.

Bez kroku 4 (`Upload Filesystem Image`) nebude dostupné webové ovládání z `data/index.html`.

Po zapnutí se ESP32 přepne do režimu access point, vytvoří vlastní síť, nahraje jednoduchou webovou stránku s joystickem a přes ni ovládá dva DC motory. Současně umí nastavovat serva, pípá při couvání a ukládá nastavení do interní paměti, aby zůstalo zachované i po restartu.

## Co projekt umí

- vytvoří vlastní Wi‑Fi síť přímo z ESP32
- zobrazí ovládací web bez nutnosti mobilní aplikace
- ovládá jízdu vpřed, vzad, zatáčení i otočení na místě
- má jemnější rozjezd motorů, deadzone joysticku a ochranné zastavení při ztrátě řízení
- umí ovládat servo výstupy z webu
- při couvání spíná akustickou signalizaci
- ukládá nastavení Wi‑Fi a řízení do `Preferences`
- zpřístupňuje zařízení i přes `http://pickbot.local`, pokud mDNS v síti funguje

## Jak to funguje jednoduše

1. ESP32 po startu vytvoří Wi‑Fi síť.
2. Po připojení otevřeš webové rozhraní v prohlížeči.
3. Pohyb joysticku se posílá přes WebSocket do firmware.
4. Firmware z joysticku spočítá výkon levého a pravého motoru.
5. Hodnoty přepočítá na PWM pro H‑bridge a podle potřeby ovládá i serva.
6. Když ovládání přestane chodit, tank se během krátké chvíle automaticky zastaví.

## Výchozí připojení

- Wi‑Fi SSID: `muj-pickbot`
- Wi‑Fi heslo: `12345678`
- Web: `http://192.168.4.1`
- mDNS adresa: `http://pickbot.local`

Název i heslo AP lze změnit přímo ve webovém rozhraní. Po změně se ESP32 automaticky restartuje.

## Ovládání z webu

- hlavní joystick řídí jízdu tanku
- pohyb nahoru znamená jízdu dopředu
- pohyb dolů znamená couvání
- pohyb do stran znamená zatáčení nebo rotaci na místě
- pod joystickem mohou být zobrazené posuvníky pro serva
- v sekci nastavení lze měnit citlivost rozjezdu, deadzone, minimální rozjezd motoru a inverzi os nebo motorů

## Hardware mapování

### Motory

- levý nebo motor A: `GPIO16`, `GPIO17`
- pravý nebo motor B: `GPIO14`, `GPIO27`

### Další výstupy

- bzučák: `GPIO22`
- LED pásek (NeoPixel): `GPIO4` (4 LED)
- servo výstupy v aktuální konfiguraci: `GPIO21`, `GPIO19`, `GPIO18`

Poznámka: v kódu je připravený i čtvrtý servo kanál, ale v aktuálním nastavení je označený jako nepoužitý (`SERVO_1_PIN = -1`).

## Struktura projektu

- `src/main.cpp` obsahuje firmware pro ESP32
- `data/index.html` je webové rozhraní nahrávané do LittleFS
- `platformio.ini` obsahuje konfiguraci PlatformIO projektu

## Použité technologie

- ESP32 + Arduino framework
- PlatformIO
- LittleFS
- ESPAsyncWebServer
- AsyncTCP
- Preferences
- WebSocket komunikace mezi webem a firmware

## Sestavení a nahrání

Ve Windows může být potřeba spouštět PlatformIO přímo přes:

```powershell
%USERPROFILE%/.platformio/penv/Scripts/platformio.exe
```

Typické příkazy:

```powershell
%USERPROFILE%/.platformio/penv/Scripts/platformio.exe run
%USERPROFILE%/.platformio/penv/Scripts/platformio.exe run -t upload
%USERPROFILE%/.platformio/penv/Scripts/platformio.exe run -t uploadfs
%USERPROFILE%/.platformio/penv/Scripts/platformio.exe device monitor
```

`uploadfs` (Upload Filesystem Image) nahraje soubory z adresáře `data`, tedy i webové rozhraní.

Doporučené pořadí při flashování:

1. `run` (build)
2. `run -t upload` (firmware)
3. **`run -t uploadfs` (POVINNÉ pro webové ovládání)**
4. `device monitor`

## Poznámky k chování firmware

- řízení používá deadzone kolem středu joysticku, takže tank nereaguje na malé nechtěné odchylky
- pro zatáčení je použité expo, aby bylo řízení ve středu jemnější
- při rozjezdu se krátce přidává boost, který pomáhá motorům překonat mrtvou zónu
- při couvání se periodicky ozývá bzučák
- při ztrátě ovládání delší než přibližně `300 ms` se motory zastaví

## Pro koho je projekt vhodný

Projekt je vhodný jako jednoduchý základ pro:

- RC mini tank nebo pásový robot
- školní nebo hobby robotický projekt
- vlastní experimenty s ESP32, webovým ovládáním a servy

## Shrnutí

Minitank Firmware je lehký firmware pro ESP32, který bez další aplikace vytvoří vlastní ovládací bod pro malý robotický tank. Hlavní výhoda je jednoduchost: zapnout, připojit se na Wi‑Fi, otevřít web a řídit.