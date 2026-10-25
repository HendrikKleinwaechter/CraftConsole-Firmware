# CraftConsole-Firmware

**Firmware der CraftConsole-Armlehne** (Raspberry Pi Pico, RP2040).

Die Armlehne meldet sich am PC als Gamepad mit 128 Tasten und 8 Achsen. Mit [ControlHub](https://github.com/HendrikKleinwaechter/ControlHub)
kommen eine Tastatur und alle Funktionen dazu: Belegung, Profile, Makros, Kachelseiten. Ohne ControlHub ist sie ein
normales Gamepad, das du im Spiel direkt belegen kannst.

Die aktuelle Version liegt unter **[Releases](https://github.com/HendrikKleinwaechter/CraftConsole-Firmware/releases/latest)**.

## Firmware aufspielen (ohne Programmieren)

1. Die Datei `CraftConsole-Firmware_<Version>.uf2` unter Releases herunterladen.
2. ControlHub beenden und das USB-Kabel der Armlehne vom PC abziehen.
3. Die **BOOTSEL**-Taste am Pico in der Armlehne gedrückt halten und das USB-Kabel wieder einstecken. Am PC erscheint ein Laufwerk **RPI-RP2**. Jetzt kannst du die Taste loslassen.
4. Die `.uf2`-Datei auf dieses Laufwerk ziehen. Der Pico startet danach von selbst neu, das Laufwerk verschwindet.
5. ControlHub starten. Unter Einstellungen → Allgemein → Diagnose steht die neue Version.

Deine Einstellungen und die Kalibrierung bleiben erhalten. Nur wenn das Changelog einer Version es ausdrücklich sagt,
müssen die Achsen neu kalibriert werden (in ControlHub unter Einstellungen → Controller → Achsen-Kalibrierung).

## Selbst bauen (Arduino IDE)

1. Unter Einstellungen → „Zusätzliche Boardverwalter-URLs“ eintragen:
   `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
2. Im Boardverwalter **„Raspberry Pi Pico/RP2040“** (Earle F. Philhower) installieren. Gebaut ist die Version 5.6.0.
3. Im Bibliotheksverwalter **Adafruit TinyUSB Library** und **Adafruit ADS1X15** installieren (die IDE fragt nach Adafruit BusIO, mit installieren).
4. `CraftConsolePro_Pico/CraftConsolePro_Pico.ino` öffnen und unter Werkzeuge einstellen:
   - Board: **Raspberry Pi Pico**
   - USB Stack: **Adafruit TinyUSB** (ohne das gibt es kein Gamepad)
   - Flash Size: **2MB (no FS)** (eine andere Größe verschiebt den Speicher für die Einstellungen)
5. Hochladen. Beim ersten Mal BOOTSEL halten und einstecken wie oben, danach geht es ohne.

Mit `arduino-cli`:

```bash
arduino-cli compile --fqbn rp2040:rp2040:rpipico:usbstack=tinyusb --output-dir build CraftConsolePro_Pico
```

## Lizenz

Die Firmware ist **Open Source** unter der [MIT-Lizenz](LICENSE). Die fertige `.uf2` enthält Fremdkomponenten mit eigenen
Lizenzen, siehe [THIRD-PARTY-LICENSES.md](THIRD-PARTY-LICENSES.md). Zu den Namen siehe [NOTICE](NOTICE).

## Passend dazu

- [ControlHub](https://github.com/HendrikKleinwaechter/ControlHub): das Programm zur Armlehne, mit der Mod für die Spieldaten
- [CraftLink-Firmware](https://github.com/HendrikKleinwaechter/CraftLink-Firmware): ein Raspberry Pi Pico als kleiner Controller für ControlHub
