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
| `w` | Wi-Fi / sterowanie przez przeglądarkę |
| `h` | pomoc na ekranie |

Ekran pokazuje symulację obu serw: orczyk obraca się z prędkością zbliżoną do
prawdziwego SG90 (~0,1 s / 60°), biała kropka to kąt docelowy.

## Sterowanie przez sieć (telefon, tablet, komputer)

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

**Sieć domowa (opcjonalnie):** w Ustawieniach na stronie wpisz SSID i hasło.
Cardputer dalej trzyma własną sieć, a dodatkowo łączy się z domową – jego adres
pokazuje ekran `w` (lub http://servo.local/). Gdy połączenie się nie uda w 15 s,
przestaje próbować (`r` na ekranie Wi-Fi ponawia).

Wi-Fi zwiększa pobór prądu – jeśli Cardputer resetuje się przy ruchu serw,
zasil serwa z zewnętrznego 5V.

## Build

```
pio run        # -> .pio/build/cardputer/firmware.bin
```
Plik `firmware.bin` wgraj przez Launcher (SD lub WebUI). Nie używaj `pio run -t upload`, bo nadpisze Launcher.
