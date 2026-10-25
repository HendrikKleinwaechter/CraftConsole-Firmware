# Fremdkomponenten (CraftConsole-Firmware)

Der eigene Code der CraftConsole-Firmware steht unter der **MIT-Lizenz** (`LICENSE`). Die fertige `.uf2`-Datei (unter Releases)
ist mit den folgenden Komponenten gebaut und enthält Teile davon. Für diese Teile gelten ihre eigenen Lizenzen; die
vollständigen Texte liegen im Ordner `LICENSES/`.

Gebaut mit **arduino-pico 5.6.0** (Earle F. Philhower III), Board „Raspberry Pi Pico“, „USB Stack: Adafruit TinyUSB“,
„Flash Size: 2MB (no FS)“.

| Komponente | Version | Lizenz | Text |
|---|---|---|---|
| arduino-pico (Arduino-Core, Kern-Bibliotheken `Wire`, `EEPROM`, `SPI`) | 5.6.0 | LGPL 2.1 | `LICENSES/LGPL-2.1_arduino-pico.txt` |
| Raspberry Pi Pico SDK | 2.2.1 (in arduino-pico 5.6.0) | BSD-3-Clause, Copyright 2020 Raspberry Pi (Trading) Ltd. | `LICENSES/BSD-3-Clause_pico-sdk.txt` |
| TinyUSB | 0.18.0 (im Pico SDK) | MIT, Copyright (c) 2018 hathach (tinyusb.org) | `LICENSES/MIT_tinyusb.txt` |
| Adafruit TinyUSB Library | 3.7.7 (Bibliothek), 3.7.4 (im Core) | MIT, Copyright (c) 2019 Ha Thach for Adafruit Industries | `LICENSES/MIT_Adafruit-TinyUSB.txt` |
| Adafruit ADS1X15 | 2.6.2 | BSD, Copyright (c) 2012 Adafruit Industries | `LICENSES/BSD_Adafruit-ADS1X15.txt` |
| Adafruit BusIO | 1.17.4 | MIT, Copyright (c) 2017 Adafruit Industries | `LICENSES/MIT_Adafruit-BusIO.txt` |

## LGPL-Hinweis (arduino-pico)

Die fertige `.uf2`-Datei enthält LGPL-lizenzierte Teile des Arduino-Cores. Die LGPL erlaubt das, wenn du diese Teile durch
eigene Fassungen ersetzen und das Ganze neu bauen kannst. Das geht: Der vollständige Quellcode der Firmware liegt hier
(MIT, Änderungen und Fehlersuche für jeden Zweck erlaubt), gebaut wird mit der Arduino IDE bzw. `arduino-cli`
(Anleitung in `README.md`).

Den Quellcode von arduino-pico in der verwendeten Version 5.6.0 bekommst du unter
<https://github.com/earlephilhower/arduino-pico/releases/tag/5.6.0>. Zusätzlich gilt: **Auf Anfrage schicke ich dir den
Quellcode der LGPL-Bestandteile in genau der verwendeten Version** – drei Jahre lang ab Veröffentlichung der jeweiligen
`.uf2`, gegen höchstens die Kosten für den Versand (Kontakt: kontakt@hendriks-werkstatt.de).

> Stand 25.10.2026 (Firmware 2.16). Vor jeder neuen `.uf2` mit den tatsächlich verwendeten Versionen abgleichen.
