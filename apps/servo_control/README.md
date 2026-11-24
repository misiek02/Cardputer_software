# Servo Control

Sterowanie dwoma serwami SG90 z Cardputer ADV.

## Podłączenie (port Grove)

| Grove Cardputera | Serwo 1 | Serwo 2 |
|---|---|---|
| GND (czarny) | brązowy | brązowy |
| 5V (czerwony) | czerwony | czerwony |
| G1 (biały) | pomarańczowy (sygnał) | – |
| G2 (żółty) | – | pomarańczowy (sygnał) |

Kolejność przewodów w kablu Grove sprawdź z opisem na obudowie.
Jeśli serwa resetują Cardputera albo drgają, zasil je z zewnętrznego 5V
(powerbank lub 4×AA) i **połącz GND zasilacza z GND Cardputera**.

## Inne piny (klawisz `p`)

Domyślnie S1 = G1, S2 = G2. Na ekranie konfiguracji wybierasz pin dla każdego
serwa; wybór zapisuje się i zostaje po restarcie. Dostępne:

| Pin | Gdzie | Uwagi |
|---|---|---|
| G1, G2 | Grove (biały / żółty) | |
| G3, G4, G6, G5 | EXT pin 1, 3, 5, 13 | |
| G13, G15 | EXT pin 12, 14 | |
| G40, G14, G39 | EXT pin 7, 9, 11 | wspólne z kartą SD – działa, bo aplikacja nie używa SD |

G8/G9 są pominięte – to wewnętrzna magistrala I²C klawiatury.
Zasilanie serw ze złącza EXT: GND = pin 4, 5V = pin 6 (5VOUT).

Sterowanie na ekranie pinów: `;` `.` / `TAB` – wybór serwa, `,` `/` – zmiana pinu,
`ENTER` – zapisz, `ESC` lub `p` – anuluj.

## Klawisze

| Klawisz | Akcja |
|---|---|
| `1` `2` / `TAB` | wybór serwa |
| `,` `/` | ruch w lewo / w prawo (można przytrzymać) |
| `;` `.` | krok większy / mniejszy (1/5/15/45°) |
| `c` | oba na 90° |
| `0` / `9` | 0° / 180° |
| `l` | link: oba serwa ruszają się razem |
| `d` | demo: ruch tam i z powrotem |
| `x` | włącz/wyłącz PWM (serwa przestają trzymać pozycję) |
| `t` | typ wybranego serwa: 180° (pozycyjne) / 360° (ciągłe obroty) |
| `[` `]` | trim punktu STOP serwa 360° (−/+ 5 µs) |
| `p` | konfiguracja pinów |
| `w` | Wi-Fi / sterowanie przez przeglądarkę |
| `h` | pomoc na ekranie |

Ekran pokazuje symulację obu serw: orczyk obraca się z prędkością zbliżoną do
prawdziwego SG90 (~0,1 s / 60°), biała kropka to kąt docelowy.

## Serwo 360° (ciągłe obroty)

Jeśli serwo nie zatrzymuje się na wybranym kącie, tylko kręci się bez końca,
a zmiana kąta zmienia jedynie **prędkość** (i stoi tylko w okolicy 90°), to jest
to wersja **360°** (np. „SG90 360”, FS90R). Nie ma ona czujnika położenia, więc
nie da się jej ustawić na kąt – sygnał oznacza kierunek i prędkość:

| sygnał | ruch |
|---|---|
| ~1,5 ms („90°”) | STOP |
| < 1,5 ms („0–89°”) | obroty w jedną stronę, im dalej od 90°, tym szybciej |
| > 1,5 ms („91–180°”) | obroty w drugą stronę |

Naciśnij `t` (albo Ustawienia → Typ serwa na stronie) – tarcza pokaże wtedy
prędkość w % i STOP, `c` zatrzymuje oba serwa, `0` / `9` to pełna prędkość.
Gdy na STOP serwo nadal powoli się kręci, popraw punkt zerowy klawiszami `[` `]`
(trim, zapisywany w pamięci). Typ jest zapamiętywany osobno dla każdego serwa.

## Sterowanie przez sieć (telefon, tablet, komputer)

Dwa tryby (klawisz `m` na ekranie Wi-Fi, albo Ustawienia na stronie):

- **własna sieć** – Cardputer tworzy swoją sieć (opis niżej); działa wszędzie,
- **istniejąca sieć** – Cardputer łączy się z Twoją siecią (dom, hotspot w
  telefonie) i nie tworzy własnej. Sterujesz z każdego urządzenia w tej sieci.

### Tryb: istniejąca sieć

1. `w` → `n` – Cardputer skanuje sieci. `;` `.` wybór, `ENTER` dalej, `r` ponowny skan.
2. Wpisz hasło z klawiatury Cardputera (`DEL` usuwa, `ENTER` łączy, `ESC` wraca).
   Sieci otwarte łączą się od razu.
3. Ekran Wi-Fi pokaże adres IP i kod QR ze stroną – otwórz go na telefonie/komputerze
   w tej samej sieci (albo http://servo.local/).

Sieć i tryb są zapamiętywane. Gdy sieci nie ma, Cardputer ponawia próbę co ~20 s;
`m` wraca do własnej sieci.

### Tryb: własna sieć

1. Na Cardputerze naciśnij `w`, potem `ENTER` – włącza Wi-Fi (stan zapamiętywany,
   po restarcie Wi-Fi włączy się samo).
2. Cardputer tworzy własną sieć **ServoCtl-XXXX**; hasło (losowe, stałe dla
   urządzenia) widać na ekranie. Telefonem zeskanuj kod QR – połączy się z siecią.
3. Telefon zwykle sam otworzy stronę sterowania (captive portal). Jeśli nie –
   wejdź na **http://192.168.4.1/** (klawisz `q` przełącza QR na adres strony).

Na stronie:

- **dotyk** – przeciągnij palcem po tarczy, każdy palec steruje swoim serwem
  (dwa kciuki = dwa serwa naraz); przyciski 0° / − / 90° / + / 180°,
- **mysz** – przeciąganie po tarczy, kółko myszy ±1° (z `Shift` ±krok),
- **klawiatura** – te same klawisze co na Cardputerze (`1` `2` `Tab`, strzałki
  lub `,` `/`, `↑` `↓` lub `;` `.` krok, `c`, `0`, `9`, `l`, `d`, `x`),
- link, demo, PWM on/off, krok oraz zmiana pinów (Ustawienia).

Stan jest wspólny: zmiany z klawiatury Cardputera i z kilku przeglądarek naraz
widać wszędzie na bieżąco (WebSocket, port 81).

**Własna + istniejąca naraz (opcjonalnie):** w trybie „własna sieć” wpisz w
Ustawieniach na stronie SSID i hasło. Cardputer dalej trzyma własną sieć, a dodatkowo łączy się z podaną – jego adres
pokazuje ekran `w` (lub http://servo.local/). Gdy połączenie się nie uda w 15 s,
przestaje próbować (`r` na ekranie Wi-Fi ponawia).

Wi-Fi zwiększa pobór prądu – jeśli Cardputer resetuje się przy ruchu serw,
zasil serwa z zewnętrznego 5V.

## Build

Potrzebny jest PlatformIO Core (CLI). Windows, jednorazowo:

```
winget install astral-sh.uv      # jeśli nie masz uv
uv tool install platformio       # pio trafia do %USERPROFILE%\.local\bin
```

(alternatywnie: rozszerzenie PlatformIO IDE w VS Code). Platforma ESP32 i biblioteki
pobiorą się same przy pierwszym buildzie.

```
pio run        # -> .pio/build/cardputer/firmware.bin
```

Wszystkie aplikacje naraz: `bash scripts/build_all.sh` (w Git Bash) → `dist/*.bin`.
Plik `firmware.bin` wgraj przez Launcher (SD lub WebUI). Nie używaj `pio run -t upload`, bo nadpisze Launcher.
