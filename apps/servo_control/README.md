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
| `p` | konfiguracja pinów |
| `h` | pomoc na ekranie |

Ekran pokazuje symulację obu serw: orczyk obraca się z prędkością zbliżoną do
prawdziwego SG90 (~0,1 s / 60°), biała kropka to kąt docelowy.

## Build

```
pio run        # -> .pio/build/cardputer/firmware.bin
```
Plik `firmware.bin` wgraj przez Launcher (SD lub WebUI). Nie używaj `pio run -t upload`, bo nadpisze Launcher.
