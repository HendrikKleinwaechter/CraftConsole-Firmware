/*
 * CraftConsole Pro – Firmware · Raspberry Pi Pico (RP2040)
 * ========================================================
 * USB-Verbundgerät: Gamepad „CraftConsole Pro“ (128 Tasten + 8 Achsen), Tastatur (nur mit ControlHub) und
 * serielle Schnittstelle (FRAME-Strom an ControlHub, Einstellungen, Kalibrierung). Keine Displays, die Anzeige
 * läuft über ControlHub. Ohne ControlHub arbeitet die Armlehne als normales Gamepad (Fallback, s. runController()).
 *
 * Tastennummern (verbindlich sind die BTN_*-Konstanten unten):
 *   1..29 CraftSlot 1 (Griff/Hebel/Thumbstick/Aux) · 30..45 CraftPad 1 · 46..53 Fingertip · 54..69 CraftPad 2 ·
 *   70..85 CraftDeck (70..72 Zündschloss/Schalter, 73..82 Tasten, 83..85 frei) · 86..114 CraftSlot 2 (wie CraftSlot 1) ·
 *   127 Ebenenschalter (CraftDeck-Schalter im Modus „ebene“, nur ohne ControlHub) · 115..126 und 128 frei.
 * Achsen (nur im jeweiligen Achsen-Modus belegt): X/Y = Thumbstick CraftSlot 1, Z/Rx = Hebel CraftSlot 1,
 *   Ry/Rz = Thumbstick CraftSlot 2, Slider/Dial = Hebel CraftSlot 2. Poti und Fingertip-Achsen kommen nur über den
 *   FRAME-Strom bei ControlHub an (das zweite Gamepad ist abgeschaltet, s. USB_SEC_ENABLED).
 *
 * !!! Werkzeuge (Tools) > USB Stack = "Adafruit TinyUSB" !!!
 * Bibliotheken: Adafruit TinyUSB Library, Adafruit ADS1X15.
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_TinyUSB.h>
#include <EEPROM.h>
#include <math.h>
#include <hardware/watchdog.h>   // watchdog_caused_reboot()
#ifdef USE_TINYUSB
#include <pico/mutex.h>         // __usb_mutex (USB-Sperre, s. usbSperren())
#endif
#if PICO_RP2040
#include <hardware/structs/vreg_and_chip_reset.h>   // Grund des Neustarts: Einschalten oder Reset-Taste (Diagnose)
#endif

// Zweites Gamepad ("CraftConsole Pro II" -- Poti + Fingertip-Joysticks als Achsen) TEMPORAER
// deaktiviert: das Senden seines HID-Reports direkt nach dem Haupt-Gamepad loest zuverlaessig
// einen USB-Stack-Haenger aus (TinyUSB-interne Race-Condition-Absicherung "Attempt to transfer
// on a busy endpoint"), der ueber den Watchdog zum Reset fuehrt. Bis das geklaert ist: Fingertips
// laufen weiter als TASTEN (Geraete-Einstellungen -> Fingertip 1-4 auf "Taste" stellen), nur die
// Achsen-Variante braucht dieses zweite Geraet. Mit ControlHub kommen Poti und Fingertip-Achsen trotzdem an
// (FRAME-Strom, Achsen 9..13). Einfach auf 1 setzen, um es wieder zu aktivieren.
#define USB_SEC_ENABLED 0

#define I2C_SDA 4
#define I2C_SCL 5
#define MUX_ADDR 0x70
#define AS5600_ADDR 0x36            // Hendriks eigene AS5600-Module (immer 0x36, getestet 01.10.2026)
#define AS5600L_ADDR 0x40           // gekaufte AS5600/AS5600L-Chips/Breakouts (0x40) -- pin-/registerkompatibel, nur andere Adresse; ein Unterbau kann beides enthalten
#define GRIP_MCP_ADDR 0x20          // Grip-MCP hinter dem Mux (beide Slots gleich, durch Kanäle isoliert)
#define ADS_THUMB_ADDR 0x49         // Thumbstick-ADS (beide Slot-Thumbsticks)
#define ADS_FINGER_ADDR 0x48        // Fingertip-ADS
#define MB_MCP_ADDR 0x27            // Mainboard-MCP (Aux-Tasten D1..D5 beider CraftSlots)
#define POTI_PIN 26
#define PIEZO_PIN 15                 // Piezo passiv (Mainboard R18/U$2, CraftCore R5/U$8): GP15 -> 100 Ohm -> Piezo -> GND, kein Transistor
#define PIEZO_FREQ_HZ 4000           // Resonanzfrequenz EKULIT EPM 121
#define MCP_GPIOA 0x12
#define MCP_IODIRA 0x00
#define MCP_IODIRB 0x01
#define MCP_GPPUA 0x0C
#define MCP_GPPUB 0x0D

#define MCP_ES1 0x21                // CraftPad 1
#define MCP_ES2 0x22                // CraftPad 2 (war 0x20 -> wegen Grip-MCP-Konflikt auf 0x22)
#define MCP_BEDIEN 0x23             // CraftDeck

// ===== Globale Tastennummerierung — Gerät 1 (reservierte Blöcke, kein Verrutschen) =====
// Finale Nummerierung (24.08.2026): jedes Modul bekommt volle MCP23017-Kapazität (16) reserviert, damit
// künftige Erweiterungen die Nummerierung nicht verschieben. Beide CraftSlots sind gleich aufgebaut: Grip(16) +
// Hebel-als-Taste(4) + Thumbstick-als-Taste(4) + Aux(5) = 29 Tasten je Slot.
#define BTN_S1_GRIP   1     // 1..16   CraftSlot1 Grip-MCP
#define BTN_S1_LEVER  17    // 17..20  CraftSlot1 Hebel-als-Taste (EncA vor/zur, EncB li/re)
#define BTN_S1_THUMB  21    // 21..24  CraftSlot1 Thumbstick-als-Taste (links/rechts/hoch/runter)
#define BTN_S1_AUX    25    // 25..29  CraftSlot1 Aux D1..D5
#define BTN_ES1       30    // 30..45  CraftPad 1 (16, davon 15 verkabelt)
#define BTN_FINGER    46    // 46..53  Fingertip 1..4 je vor/zurück
#define BTN_ES2       54    // 54..69  CraftPad 2 (zweite Position, gleiches Board mit Adresse 0x22; 16, davon 15 verkabelt)
#define BTN_BEDIEN    70    // 70..85  CraftDeck (16, davon 13 verkabelt):
                             // 70=Zuendschloss Nullstellung, 71=Eingang1(D1/GPB2), 72=Eingang2(D2/GPB3)
                             // (was dort ankommt, haengt von cfg.bedienSwitchMode ab, s. runController()),
                             // 73..82=10 Gateron-Pikto-Tasten, 83..85 frei.
#define BTN_S2_GRIP   86    // 86..101 CraftSlot2 Grip-MCP
#define BTN_S2_LEVER  102   // 102..105 CraftSlot2 Hebel-als-Taste
#define BTN_S2_THUMB  106   // 106..109 CraftSlot2 Thumbstick-als-Taste
#define BTN_S2_AUX    110   // 110..114 CraftSlot2 Aux D1..D5
// 115..126 bewusst frei (Reserve) -- die Ebenenumschaltung je CraftSlot braucht KEINE eigene Nummer
// hier: die laeuft schon vollstaendig in ControlHub (craftSlotLayers.json, eine ganz normale
// bestehende physische Taste wird dafuer zugewiesen), keine Firmware-Aenderung noetig. 128 bleibt als
// Sicherheitsmarge frei (DirectInput-Grenze).
#define BTN_LAYER_MOD 127   // Ebenenschalter: gedrueckt, solange der CraftDeck-Schalter D1 im Modus „ebene“
                             // aktiv ist, nur ohne ControlHub (s. runController()) -- keine physische Taste
                             // hat diese Nummer, im Spiel wird sie als zweite Ebene belegt.
#define MAIN_BTNS     128   // Gerät 1: 128 Tasten (DirectInput-Grenze, genutzt bis Nr. 127, 128 bleibt frei).
                             // HID-Descriptor (desc_main[]), MainReport.btn[], outBtn[]/nBtn[] und die
                             // Entprell-Schleife MUESSEN dazu passen (16 Byte).

#define USB_VID 0x2E8A
#define USB_PID 0xCC01
#define NAME_MANUF "ControlCraft"
#define NAME_PROD "CraftConsole Pro"
#define NAME_MAIN "CraftConsole Pro"
#define NAME_SEC "CraftConsole Pro II"
#define NAME_KBD "CraftConsole Pro Keyboard"

const uint8_t mapDirectus[16]={0,1,2,3,4,5,6,7,15,9,8,10,11,12,13,14};
// Tasten-Test-Protokoll 25.08.2026 (Hendrik): tatsächliche Verkabelung ist GPB0..7 (Bit 8..15) ZUERST,
// dann GPA0..6 (Bit 0..6) -- das unbenutzte 16. Pin ist GPA7 (Bit 7), NICHT GPB0 (Bit 8) wie zuvor
// angenommen. Alte Reihenfolge {0..7,9..15} las Bit 8 nie -- dadurch war "Taste 1" (oben links, an
// Bit 8) komplett unsichtbar (mit separatem MCP23017-Testprogramm als physisch funktionsfähig
// bestätigt), und alle anderen 14 Tasten lagen der Reihe nach eine Position verschoben.
const uint8_t mapSlot1[15]={8,9,10,11,12,13,14,15,0,1,2,3,4,5,6};
const uint8_t mapSlot2[15]={8,9,10,11,12,13,14,15,0,1,2,3,4,5,6}; // CraftPad 2 = dasselbe Board wie CraftPad 1
// Tasten-Test-Protokoll 25.08.2026 (Hendrik, zweiter sauberer Durchlauf, in sich vollstaendig
// widerspruchsfrei -- kein Kontaktprellen mehr gemeldet): auf der Platine sind 4 Paare benachbarter
// Gateron-Tasten (laut Netzliste/Schaltplan U$1<->U$3, U$4<->U$6, U$7<->U$8, U$9<->U$10) vertauscht
// verloetet/verdrahtet -- vermutlich ein Verwechseln beim Bestuecken benachbarter Positionen. Hier in
// der Zuordnungstabelle kompensiert (gleiches Prinzip wie mapDirectus/mapSlot1/mapSlot2): Taste
// „Gateron N" liest jetzt wieder den korrekten physischen Schalter, unabhaengig von der vertauschten
// Verkabelung. Bei kuenftigen Boards dieses Layouts beim Bestuecken auf genau diese 4 Paare achten.
const uint8_t mapBedien[10]={2,1,0,5,4,3,7,6,9,8}; // GPA0-7 + GPB0/1 (10 Gateron-Tasten des CraftDeck)
// Fingertip-Kanalreihenfolge ist am Board vertauscht (Hendriks Fund 25.08.2026): logischer Index c
// (0=Fingertip1..3=Fingertip4, s. BTN_FINGER/cfg.finger[]) muss von ADS-Kanal (3-c) statt Kanal c
// gelesen werden -- Fingertip1↔4 und 2↔3 vertauscht. NUR die Kanalauswahl beim Lesen ist betroffen,
// cfg.finger[c]/fingerMode[c]/Tastennummern bleiben unveraendert nach c indiziert.
#define FINGER_CH(c) (3-(c))

// HID: Gerät 1 "CraftConsole Pro" — 128 Tasten + 8 Achsen (128=0x80 Bits = 16 Byte, seit Schritt 2
// 24.08.2026 hochgesetzt von vorher 112=0x70/14 Byte, s. MAIN_BTNS-Kommentar oben). Die beiden 0x80
// unten sind Usage Maximum (Tasten-Anzahl) bzw. Report Count -- MUESSEN beide zu MAIN_BTNS passen.
uint8_t const desc_main[]={0x05,0x01,0x09,0x05,0xA1,0x01,0x05,0x09,0x19,0x01,0x29,0x80,0x15,0x00,0x25,0x01,
 0x75,0x01,0x95,0x80,0x81,0x02,0x05,0x01,0x16,0x00,0x80,0x26,0xFF,0x7F,0x75,0x10,0x95,0x08,
 0x09,0x30,0x09,0x31,0x09,0x32,0x09,0x33,0x09,0x34,0x09,0x35,0x09,0x36,0x09,0x37,0x81,0x02,0xC0};
// HID: Gerät 2 "CraftConsole Pro II" — nur 8 Achsen (keine Tasten)
uint8_t const desc_sec[]={0x05,0x01,0x09,0x04,0xA1,0x01,
 0x05,0x01,0x16,0x00,0x80,0x26,0xFF,0x7F,0x75,0x10,0x95,0x08,
 0x09,0x30,0x09,0x31,0x09,0x32,0x09,0x33,0x09,0x34,0x09,0x35,0x09,0x36,0x09,0x37,0x81,0x02,0xC0};
struct __attribute__((packed)) MainReport { uint8_t btn[16]; int16_t x,y,z,rx,ry,rz,slider,dial; };  // 16 Byte = 128 Tasten
struct __attribute__((packed)) SecReport  { int16_t x,y,z,rx,ry,rz,slider,dial; };
// HID: Gerät 3 -- Standard-Boot-Tastatur (nur aktiv, wenn ControlHub Tasten-Ausgabe sendet, s. Abschnitt "Fallback")
uint8_t const desc_kbd[]={ TUD_HID_REPORT_DESC_KEYBOARD() };
Adafruit_USBD_HID usb_main, usb_sec, usb_kbd;
MainReport rMain; SecReport rSec;
hid_keyboard_report_t rKbd={0,0,{0,0,0,0,0,0}};
Adafruit_ADS1115 ads48, ads49;

#define CFG_MAGIC 0xB5
#define CFG_VERSION 15   // Aufbau von Settings im EEPROM. Jede Aenderung am Struct -> hochzaehlen: dann gelten die
                          // gespeicherten Werte nicht mehr (Werkseinstellung, alle Achsen neu kalibrieren).
                          // v15 wurde vor der ersten Auslieferung bis 25.08.2026 mehrfach erweitert (Unterbau/
                          // Griff, Thumbstick-als-Taste, Zuendschloss-Modi, Werte je einzelner Achse).
#define EE_SIZE 256
#define FW_VERSION "2.16"   // 2.16 (25.10.2026): USB-Zugriffe der loop() unter der USB-Sperre (Watchdog-Neustarts „Phase 8“).
                            // 2.15 (25.10.2026): Diagnose (Grund des letzten Neustarts, Absturz-Erkennung, STAT), Einstellungen
                            // beim Start geprueft. 2.14 (24.10.2026): Schubhebel-Mitte nur per eigener Kalibrierung (CAL CENTER <hebel>), Tastenmodus nur Eingang 1,
                            // Prototyp-Board entfernt. 2.13: restliche Tastaturtasten, Druck/Rollen kommen an, tote Teile entfernt
// Unterbau je CraftSlot (Bewegungsart) -- bisher wirkt nur der Schubhebel anders (Mitte, s. istSchubhebel()), der Rest folgt spaeter
#define UNTERBAU_HEBEL_JOYSTICK 0   // Fahrhebel/Joystick (vom Prinzip gleich) -- aktuell einzige implementierte Verhaltenslogik
#define UNTERBAU_SCHUBHEBEL 1       // Mitte nur per eigener Kalibrierung (s. istSchubhebel()), sonst wie Fahrhebel
#define UNTERBAU_LENKJOYSTICK 2     // reserviert: eigene Anpassungen folgen erst spaeter
// Griff je CraftSlot (welcher konkrete Griff sitzt drauf) -- V2: weitere Werte folgen, dann hier ergaenzen
#define GRIFF_DIRECTUS 0
#define GRIFF_MORTIS 1
#define GRIFF_MINOR 2
#define GRIFF_VECTOR_R 3
#define GRIFF_VECTOR_L 4
#define GRIFF_POLARIS 5
#define GRIFF_MAX 5   // hoechster aktuell gueltiger Griff-Wert -- bei neuen Griffen (V2) mit hochzaehlen
struct Axis { int16_t cen,lo,hi; };
struct Settings {
  uint8_t magic,version;
  Axis thumb[2]; Axis thumb2[2]; Axis finger[4];   // thumb=CraftSlot1-Thumbstick, thumb2=CraftSlot2
  int16_t potiLo,potiHi,as0cen,as1cen,as0min,as0max,as1min,as1max;
  int16_t as2cen,as3cen,as2min,as2max,as3min,as3max;   // CraftSlot 2 Hebel-Encoder (final)
  uint8_t layerBtn;                   // ungenutzt (frueher Ebenen-Taste, Ebenen macht ControlHub) -- bleibt, damit die
                                      // gespeicherten Einstellungen weiter passen
  uint8_t leverModeA, leverModeB;
  uint8_t leverModeA2, leverModeB2;   // CraftSlot 2 Hebel: Taster- oder Achsen-Modus (analog zu leverModeA/B)
  uint8_t fingerMode[4];
  uint8_t thumbModeX, thumbModeY, thumbModeX2, thumbModeY2;   // Thumbstick-als-Taste (24.08.2026)
  // Totbereich/Empfindlichkeit/Ausloeseschwelle je einzelner Achse (25.08.2026, Hendriks Wunsch: "jede
  // einzelne Achse einstellen koennen").
  uint8_t thumbDzX, thumbDzY, thumbSensX, thumbSensY, thumbBtnThreshX, thumbBtnThreshY;       // CraftSlot 1 Thumbstick
  uint8_t thumbDzX2, thumbDzY2, thumbSensX2, thumbSensY2, thumbBtnThreshX2, thumbBtnThreshY2; // CraftSlot 2 Thumbstick
  uint8_t leverThreshA, leverThreshB;     // CraftSlot 1 Hebel
  uint8_t leverThreshA2, leverThreshB2;   // CraftSlot 2 Hebel
  uint8_t fingerThresh[4];                // Fingertip 1..4
  // Totbereich/Empfindlichkeit auch fuer Hebel + Fingertip (25.08.2026, Hendriks Wunsch: "fuer alle Achsen").
  uint8_t leverDzA, leverDzB, leverSensA, leverSensB;       // CraftSlot 1 Hebel
  uint8_t leverDzA2, leverDzB2, leverSensA2, leverSensB2;   // CraftSlot 2 Hebel
  uint8_t fingerDz[4], fingerSens[4];                       // Fingertip 1..4
  uint8_t model;                      // Armlehnen-Modell (0=Pro …, 9=Individuell)
  uint8_t potiInv;                    // Poti-Rad spiegeln (0=normal, 1=umgekehrt)
  uint16_t calDone;                   // Bitmaske: welche Achse ist kalibriert (Bit = calGroup-1)
  uint16_t expectMask;                // Modell „Individuell": welche Module SOLLEN vorhanden sein (Bits = MODULES-Index)
  uint8_t slot1Unterbau, slot2Unterbau;   // Bewegungsart je CraftSlot: s. UNTERBAU_* oben
  uint8_t slot1Griff, slot2Griff;         // konkreter Griff je CraftSlot: s. GRIFF_* oben
  // CraftDeck-Schalter D1/D2 (25.08.2026) -- frei waehlbar, was dort angeschlossen ist, s. runController():
  // 0=Ebenenschalter (D1 haelt Taste 127, NUR ohne ControlHub -- ist ControlHub verbunden, gilt IMMER
  // Tastenmodus, Hendriks Wunsch: weniger Komplexitaet/Stoerpotenzial), 1=Tastenmodus (jeder Wechsel von
  // Eingang 1 gibt einen kurzen Impuls auf Taste 71, wie ein normaler Taster), 2=echtes Zuendschloss.
  uint8_t bedienSwitchMode;
  uint8_t ignitionPositions;          // nur bei bedienSwitchMode==2: 2 oder 3 Stellungen
  // Je Stellung (Index 0=Nullstellung, 1=Eingang1/D1, 2=Eingang2/D2) frei waehlbar: welcher der drei
  // CraftDeck-Ausgaenge (0->Taste70, 1->Taste71, 2->Taste72) ausgeloest wird, und ob er gehalten
  // bleibt oder nur kurz (0,7s) antippt. Bei ignitionPositions==2 wird Index 2 (Eingang2) ignoriert.
  uint8_t ignitionPosBtn[3];
  uint8_t ignitionPosHold[3];         // 1=gehalten solange Stellung anliegt, 0=kurzer Impuls
};
Settings cfg;
void cfgSave(){ cfg.magic=CFG_MAGIC; cfg.version=CFG_VERSION; EEPROM.put(0,cfg); EEPROM.commit(); }
void cfgDefaults(){ cfg.magic=CFG_MAGIC; cfg.version=CFG_VERSION;
  for(int i=0;i<2;i++){ cfg.thumb[i]={13000,5000,21000}; cfg.thumb2[i]={13000,5000,21000}; }
  for(int i=0;i<4;i++){ cfg.finger[i]={13000,5000,21000}; cfg.fingerMode[i]=0; cfg.fingerThresh[i]=50; cfg.fingerDz[i]=6; cfg.fingerSens[i]=100; }
  cfg.potiLo=0; cfg.potiHi=1023; cfg.as0cen=2048; cfg.as1cen=2048;
  cfg.as0min=-1500; cfg.as0max=1500; cfg.as1min=-1500; cfg.as1max=1500;
  cfg.as2cen=2048; cfg.as3cen=2048; cfg.as2min=-1500; cfg.as2max=1500; cfg.as3min=-1500; cfg.as3max=1500;
  cfg.layerBtn=0; cfg.leverModeA=0; cfg.leverModeB=0; cfg.leverThreshA=50; cfg.leverThreshB=50;
  cfg.leverDzA=6; cfg.leverDzB=6; cfg.leverSensA=100; cfg.leverSensB=100;
  cfg.leverModeA2=0; cfg.leverModeB2=0; cfg.leverThreshA2=50; cfg.leverThreshB2=50;
  cfg.leverDzA2=6; cfg.leverDzB2=6; cfg.leverSensA2=100; cfg.leverSensB2=100;
  cfg.thumbModeX=0; cfg.thumbModeY=0; cfg.thumbModeX2=0; cfg.thumbModeY2=0;
  cfg.thumbDzX=6; cfg.thumbDzY=6; cfg.thumbSensX=100; cfg.thumbSensY=100; cfg.thumbBtnThreshX=50; cfg.thumbBtnThreshY=50;
  cfg.thumbDzX2=6; cfg.thumbDzY2=6; cfg.thumbSensX2=100; cfg.thumbSensY2=100; cfg.thumbBtnThreshX2=50; cfg.thumbBtnThreshY2=50;
  cfg.model=0; cfg.potiInv=0; cfg.calDone=0; cfg.expectMask=0;
  // Werks-Grundkonfiguration: CraftSlot 1 = Directus, CraftSlot 2 = Polaris, beide Fahrhebel/Joystick-Unterbau
  cfg.slot1Unterbau=UNTERBAU_HEBEL_JOYSTICK; cfg.slot2Unterbau=UNTERBAU_HEBEL_JOYSTICK;
  cfg.slot1Griff=GRIFF_DIRECTUS; cfg.slot2Griff=GRIFF_POLARIS;
  // Standard: Tastenmodus (funktioniert ohne weitere Annahmen ueber die angeschlossene Hardware --
  // sicherster Ausgangspunkt, s. bedienSwitchMode-Kommentar oben am Struct).
  cfg.bedienSwitchMode=1; cfg.ignitionPositions=2;
  cfg.ignitionPosBtn[0]=0; cfg.ignitionPosBtn[1]=1; cfg.ignitionPosBtn[2]=2;   // Identitaet: Null->70,E1->71,E2->72
  cfg.ignitionPosHold[0]=1; cfg.ignitionPosHold[1]=1; cfg.ignitionPosHold[2]=1; }
void cfgLoad(){ EEPROM.get(0,cfg); if(cfg.magic!=CFG_MAGIC||cfg.version!=CFG_VERSION){ cfgDefaults(); cfgSave(); } }
enum { CG_NONE,CG_THUMB,CG_LEVA,CG_LEVB,CG_F1,CG_F2,CG_F3,CG_F4,CG_POTI,CG_THUMB2,CG_LEVA2,CG_LEVB2 }; int calGroup=CG_NONE;
// Gespeicherte Einstellungen auf gueltige Werte pruefen (25.10.2026). Format v15 wurde bis 25.08.2026 mehrfach erweitert,
// ohne die Versionsnummer zu erhoehen -- auf frueher eingerichteten Armlehnen stehen in den spaeteren Feldern leere
// Speicherbytes (255). Beispiel: CraftDeck-Schalter als „Zuendschloss“ mit Taste 70+255 = 69 dauerhaft gedrueckt.
// Ungueltige Werte gehen einzeln auf den Standard zurueck (eine Achse samt ihrem „kalibriert“-Bit), der Rest bleibt.
uint8_t cfgPruefen(){
  Settings alt=cfg; cfgDefaults(); const Settings def=cfg; cfg=alt;
  uint8_t n=0;
  auto bis=[&](uint8_t& v,uint8_t dv,int lo,int hi){ if(v<lo||v>hi){ v=dv; n++; } };
  bis(cfg.leverModeA,def.leverModeA,0,1); bis(cfg.leverModeB,def.leverModeB,0,1);
  bis(cfg.leverModeA2,def.leverModeA2,0,1); bis(cfg.leverModeB2,def.leverModeB2,0,1);
  bis(cfg.thumbModeX,def.thumbModeX,0,1); bis(cfg.thumbModeY,def.thumbModeY,0,1);
  bis(cfg.thumbModeX2,def.thumbModeX2,0,1); bis(cfg.thumbModeY2,def.thumbModeY2,0,1);
  bis(cfg.thumbDzX,def.thumbDzX,0,25); bis(cfg.thumbDzY,def.thumbDzY,0,25); bis(cfg.thumbDzX2,def.thumbDzX2,0,25); bis(cfg.thumbDzY2,def.thumbDzY2,0,25);
  bis(cfg.thumbSensX,def.thumbSensX,50,150); bis(cfg.thumbSensY,def.thumbSensY,50,150); bis(cfg.thumbSensX2,def.thumbSensX2,50,150); bis(cfg.thumbSensY2,def.thumbSensY2,50,150);
  bis(cfg.thumbBtnThreshX,def.thumbBtnThreshX,10,95); bis(cfg.thumbBtnThreshY,def.thumbBtnThreshY,10,95);
  bis(cfg.thumbBtnThreshX2,def.thumbBtnThreshX2,10,95); bis(cfg.thumbBtnThreshY2,def.thumbBtnThreshY2,10,95);
  bis(cfg.leverThreshA,def.leverThreshA,10,95); bis(cfg.leverThreshB,def.leverThreshB,10,95); bis(cfg.leverThreshA2,def.leverThreshA2,10,95); bis(cfg.leverThreshB2,def.leverThreshB2,10,95);
  bis(cfg.leverDzA,def.leverDzA,0,25); bis(cfg.leverDzB,def.leverDzB,0,25); bis(cfg.leverDzA2,def.leverDzA2,0,25); bis(cfg.leverDzB2,def.leverDzB2,0,25);
  bis(cfg.leverSensA,def.leverSensA,50,150); bis(cfg.leverSensB,def.leverSensB,50,150); bis(cfg.leverSensA2,def.leverSensA2,50,150); bis(cfg.leverSensB2,def.leverSensB2,50,150);
  for(int i=0;i<4;i++){ bis(cfg.fingerMode[i],def.fingerMode[i],0,1); bis(cfg.fingerThresh[i],def.fingerThresh[i],10,95);
    bis(cfg.fingerDz[i],def.fingerDz[i],0,25); bis(cfg.fingerSens[i],def.fingerSens[i],50,150); }
  bis(cfg.model,def.model,0,9); bis(cfg.potiInv,def.potiInv,0,1);
  bis(cfg.slot1Unterbau,def.slot1Unterbau,0,2); bis(cfg.slot2Unterbau,def.slot2Unterbau,0,2);
  bis(cfg.slot1Griff,def.slot1Griff,0,GRIFF_MAX); bis(cfg.slot2Griff,def.slot2Griff,0,GRIFF_MAX);
  bis(cfg.bedienSwitchMode,def.bedienSwitchMode,0,2); bis(cfg.ignitionPositions,def.ignitionPositions,2,3);
  for(int i=0;i<3;i++){ bis(cfg.ignitionPosBtn[i],def.ignitionPosBtn[i],0,2); bis(cfg.ignitionPosHold[i],def.ignitionPosHold[i],0,1); }
  // Achsen: Kalibrierung muss lo <= Mitte <= hi im Bereich des Wandlers ergeben, sonst Standard und „nicht kalibriert“
  auto achse=[&](Axis& a,const Axis& d,int bit){ if(!(a.lo>=0 && a.lo<=a.cen && a.cen<=a.hi && a.lo<a.hi && a.hi<=32767)){ a=d; cfg.calDone&=~(1u<<bit); n++; } };
  for(int i=0;i<2;i++){ achse(cfg.thumb[i],def.thumb[i],CG_THUMB-1); achse(cfg.thumb2[i],def.thumb2[i],CG_THUMB2-1); }
  for(int i=0;i<4;i++) achse(cfg.finger[i],def.finger[i],CG_F1-1+i);
  auto hebel=[&](int16_t& cen,int16_t& mn,int16_t& mx,int16_t dc,int16_t dmn,int16_t dmx,int bit){
    if(!(cen>=0 && cen<=4095 && mn<=0 && mx>=0 && mn>=-4096 && mx<=4096)){ cen=dc; mn=dmn; mx=dmx; cfg.calDone&=~(1u<<bit); n++; } };
  hebel(cfg.as0cen,cfg.as0min,cfg.as0max,def.as0cen,def.as0min,def.as0max,CG_LEVA-1);
  hebel(cfg.as1cen,cfg.as1min,cfg.as1max,def.as1cen,def.as1min,def.as1max,CG_LEVB-1);
  hebel(cfg.as2cen,cfg.as2min,cfg.as2max,def.as2cen,def.as2min,def.as2max,CG_LEVA2-1);
  hebel(cfg.as3cen,cfg.as3min,cfg.as3max,def.as3cen,def.as3min,def.as3max,CG_LEVB2-1);
  if(!(cfg.potiLo>=0 && cfg.potiLo<cfg.potiHi && cfg.potiHi<=4095)){ cfg.potiLo=def.potiLo; cfg.potiHi=def.potiHi; cfg.calDone&=~(1u<<(CG_POTI-1)); n++; }
  if(n) cfgSave();
  return n;
}

bool okSlot1=0,okSlot2=0,okBedien=0,okAS0=0,okAS1=0,okAds48=0,okAds49=0;   // okSlot1/2 = CraftPad 1/2, okBedien = CraftDeck
bool serverActive=false; unsigned long lastValidOut=0; uint8_t goodOutStreak=0;   // s. emitStatus()/processCmd() weiter unten
// Zeitmessung fuer die Fehlersuche: haelt die LANGSAMSTE Runde seit dem letzten Auslesen fest
// (s. DET maxLoopUs/maxSerialUs/maxCtrlUs in emitStatus() -- wird dort nach dem Senden zurueckgesetzt).
unsigned long maxLoopUs=0, maxSerialUs=0, maxCtrlUs=0;
bool potiEnabled=true, monEnabled=false;
uint8_t btnPrev[16]={0};   // Entprellung Gerät 1 (128 Tasten = 16 Byte)
bool learnMode=false; uint8_t rawPrev[16]={0};   // "Taste lernen": erkennt frischen Tastendruck
uint32_t i2cFailCount=0;        // hochgezählt bei jeder fehlgeschlagenen I2C-Übertragung
bool wdtReboot=false;           // letzter Neustart durch Watchdog?
// Diagnose (25.10.2026, Hendrik: „dass die Firmware auch sendet, was in ihrem Watchdog passiert ist, wenn sie abstuerzt,
// und generell immer mal ein Statusupdate“): Grund des letzten Neustarts und Stand davor aus den Watchdog-Scratch-
// Registern (ueberleben einen Neustart, nicht aber Stromausfall). ControlHub fragt STAT und DIAG beim Verbinden ab.
#define SCR_MARKE   0xC0FFEE42u   // scratch[1]: Rundenzeit [0], Phase [2] und Laufzeit in s [3] stammen von dieser Firmware
#define SCR_ABSTURZ 0xDEADF417u   // scratch[1] nach einem Absturz, s. isr_hardfault()
const char* neustartGrund="unbekannt";   // strom | reset | watchdog | absturz | software | unbekannt
uint32_t vorRundeUs=0, vorPhase=0, vorLaufzeitS=0; bool vorBekannt=false;   // Stand vor dem Neustart
uint8_t cfgRepariert=0;                  // beim Start auf Standard gesetzte ungueltige Einstellungen (cfgPruefen())
uint32_t outGut=0, outFehl=0, cmdZuLang=0;   // seit dem Start: gueltige/verworfene OUT-Nachrichten, zu lange Zeilen
// USB-Sperre (25.10.2026). Ursache der Watchdog-Neustarts in Phase 8 (Diagnose von Hendriks Armlehne: „hing vor dem
// USB-Bericht an den PC“): Im Adafruit-TinyUSB-Port fuer den RP2040 laeuft tud_task() in einem Software-Interrupt,
// und nur der haelt dabei __usb_mutex. sendReport() und die Serial-Zugriffe aus der loop() liefen ohne diese Sperre.
// Unterbrach der Interrupt sie mitten in TinyUSBs Endpunkt-Belegung (tu_edpt_claim() sperrt _usbd_mutex mit „ewig
// warten“), wartete der Interrupt auf die Sperre der unterbrochenen loop() -- Stillstand, nach 4 s der Watchdog.
// Haelt die loop() __usb_mutex, laesst der Interrupt tud_task() aus (so im Port vorgesehen) und holt es beim naechsten
// Mal nach. Nur um Aufrufe, die nicht warten (Lesen, Bericht senden, Frame nur wenn er ganz passt): Serial.write() mit
// vollem Puffer ruft yield() und braucht dann selbst tud_task() -- unter der Sperre wuerde es nie fertig.
#ifdef USE_TINYUSB
extern mutex_t __usb_mutex;
static inline bool usbSperren(){ return mutex_try_enter(&__usb_mutex, NULL); }
static inline void usbFreigeben(){ mutex_exit(&__usb_mutex); }
#else
static inline bool usbSperren(){ return true; }
static inline void usbFreigeben(){}
#endif
// Absturz (HardFault, z. B. ungueltiger Speicherzugriff): Marker setzen und sofort neu starten. Vorher blieb der Pico
// dann stehen, bis nach 4 s der Watchdog zuschlug, und das Log sah aus wie ein Haenger.
extern "C" void isr_hardfault(void){ watchdog_hw->scratch[1]=SCR_ABSTURZ; watchdog_reboot(0,0,0); for(;;){} }
// ---- Ereignis-Log (RAM-Ringpuffer) für die Diagnosedatei ----
#define LOGN 28
#define LOGW 56
char logBuf[LOGN][LOGW]; uint8_t logHead=0; uint16_t logCount=0;
void logEvent(const char* m){ unsigned long s=millis()/1000;
  snprintf(logBuf[logHead],LOGW,"[%lus] %s",s,m); logHead=(logHead+1)%LOGN; if(logCount<60000)logCount++; }
unsigned long lastMon=0,lastLive=0;
char cmdBuf[512]; uint16_t cmdLen=0;   // groß genug für IMPORT <hex> und eingehende <OUT>-Nachrichten
unsigned long lastFrameSend=0; char frameBuf[420];   // FRAME-Stream an ControlHub, 50 Hz, non-blocking
enum { MODE_RUN, MODE_CAL }; int mode=MODE_RUN;

bool i2cPresent(uint8_t a){ Wire.beginTransmission(a); return Wire.endTransmission()==0; }
void muxSelect(int8_t ch){ Wire.beginTransmission(MUX_ADDR); Wire.write(ch<0?0:(1<<ch)); Wire.endTransmission(); }
void mcpWrite(uint8_t a,uint8_t r,uint8_t v){ Wire.beginTransmission(a); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
uint16_t mcpRead(uint8_t a){ Wire.beginTransmission(a); Wire.write(MCP_GPIOA);
  if(Wire.endTransmission(false)!=0){ i2cFailCount++; return 0xFFFF; } Wire.requestFrom(a,(uint8_t)2);
  if(Wire.available()<2){ i2cFailCount++; return 0xFFFF; } uint8_t ga=Wire.read(),gb=Wire.read(); return ((uint16_t)gb<<8)|ga; }
void mcpInit(uint8_t a){ mcpWrite(a,MCP_IODIRA,0xFF); mcpWrite(a,MCP_IODIRB,0xFF); mcpWrite(a,MCP_GPPUA,0xFF); mcpWrite(a,MCP_GPPUB,0xFF); }
int readAS5600(uint8_t addr){ Wire.beginTransmission(addr); Wire.write(0x0C);
  if(Wire.endTransmission(false)!=0){ i2cFailCount++; return -1; } Wire.requestFrom(addr,(uint8_t)2);
  if(Wire.available()<2){ i2cFailCount++; return -1; } int h=Wire.read(),l=Wire.read(); return ((h&0x0F)<<8)|l; }
uint8_t detectEncoderAddr(){ if(i2cPresent(AS5600_ADDR))return AS5600_ADDR; if(i2cPresent(AS5600L_ADDR))return AS5600L_ADDR; return 0; }
int shortestDelta(int a,int c){ int d=a-c; while(d>2048)d-=4096; while(d<-2048)d+=4096; return d; }
int readPoti(){ return potiEnabled?analogRead(POTI_PIN):(cfg.potiLo+cfg.potiHi)/2; }
// tone(pin,freq,ms) ist auf dem RP2040-Core (Earle Philhower) nicht-blockierend (PWM + Timer-Alarm
// schaltet nach ms automatisch ab) -- kein delay(), daher unkritisch fuer den 4s-Watchdog.
void beep(uint16_t ms=150, uint16_t freqHz=PIEZO_FREQ_HZ){ tone(PIEZO_PIN,freqHz,ms); }

// ================= Generisches CraftSlot-Modell =================
// Jeder Slot kann (optional) haben: 1x Grip-MCP (Kanal A) + bis 2x AS5600 (Kanal A/B)
// + Thumbstick (ADS_THUMB) + 5 Aux-Tasten (Mainboard-MCP). Auto-Erkennung der I2C-Geräte beim Boot.
struct SlotCfg {
  uint8_t chA, chB;            // Mux-Kanäle (A = Grip-MCP/Encoder1, B = Encoder2)
  uint8_t adsX, adsY;          // ADS_THUMB-Kanäle für Thumbstick X/Y
  // Vorzeichen fuer die TASTEN-Schwellenlogik (Thumbstick-als-Taste, s. runController()) -- +1/-1,
  // je nachdem ob der jeweilige ADS-Kanal bei "positiver" Auslenkung (X=rechts, Y=runter) einen
  // hoeheren oder niedrigeren Rohwert liefert. NUR die Tasten-Zuordnung betrifft das; der Achsen-
  // Modus hat dafuer bereits einen eigenen Software-"Invertieren"-Schalter (ControlHub Achsen-
  // Zuordnung), braucht also keine eigene Firmware-Konstante. Ermittelt aus Hendriks Tasten-Test-
  // Protokoll 25.08.2026 (CraftSlot1: beide Achsen invertiert; CraftSlot2 nach dem Kanal-Tausch
  // unten: nur X invertiert).
  int8_t thumbSignX, thumbSignY;
  uint8_t btnGrip, btnLever, btnAux;   // globale Tasten-Basen (Gerät 1)
  uint8_t encA, encB;          // Encoder-Index (0..3) für Kalibrier-Speicher
  uint8_t aux[5];              // Bit 0..15 am Mainboard-MCP (MB_MCP_ADDR) fuer Aux D1..D5
};
const SlotCfg slot[2]={
  {0,1, 0,1, -1,-1, BTN_S1_GRIP,BTN_S1_LEVER,BTN_S1_AUX, 0,1, {4,3,2,1,0}},      // Aux=GPA4..0
  // adsX/adsY vertauscht (waren 2,3 -- Kanaele X/Y am Thumbstick-ADS sind hier tatsaechlich gekreuzt
  // verkabelt, analog zum frueheren Fingertip-Kanaltausch). Aux 1-3 (waren 8,9,10) ebenfalls nach
  // Tasten-Test-Protokoll umsortiert; Aux 4/5 (11,12) noch UNGETESTET (nicht verbaut).
  {2,3, 3,2, -1,1, BTN_S2_GRIP,BTN_S2_LEVER,BTN_S2_AUX, 2,3, {10,8,9,11,12}},   // Aux=GPB0..4
};
struct SlotState { bool hasMcp, hasEncA, hasEncB; uint8_t addrEncA, addrEncB; };
SlotState slotSt[2]={{0,0,0,0,0},{0,0,0,0,0}};
bool okMbMcp=false;   // Mainboard-MCP 0x27 (Aux)

// Zugriff auf die (nach Encoder-Index benannten) Kalibrierwerte
int16_t* asCenP(int e){ switch(e){case 0:return &cfg.as0cen;case 1:return &cfg.as1cen;case 2:return &cfg.as2cen;default:return &cfg.as3cen;} }
int16_t* asMinP(int e){ switch(e){case 0:return &cfg.as0min;case 1:return &cfg.as1min;case 2:return &cfg.as2min;default:return &cfg.as3min;} }
int16_t* asMaxP(int e){ switch(e){case 0:return &cfg.as0max;case 1:return &cfg.as1max;case 2:return &cfg.as2max;default:return &cfg.as3max;} }

void scanSlots(){
  for(int s=0;s<2;s++){
    muxSelect(slot[s].chA);
    slotSt[s].hasMcp   = i2cPresent(GRIP_MCP_ADDR);
    slotSt[s].addrEncA = detectEncoderAddr();
    slotSt[s].hasEncA  = (slotSt[s].addrEncA != 0);
    if(slotSt[s].hasMcp) mcpInit(GRIP_MCP_ADDR);
    muxSelect(slot[s].chB);
    slotSt[s].addrEncB = detectEncoderAddr();
    slotSt[s].hasEncB  = (slotSt[s].addrEncB != 0);
  }
  muxSelect(-1);
}

// Alle Module erkennen (Boot + STATUS-Neuabfrage) -> setzt ok*-Flags + initialisiert MCPs
void detectAll(){
  scanSlots();
  okAS0=slotSt[0].hasEncA; okAS1=slotSt[0].hasEncB;
  // Kleine Ruhepausen zwischen den Bus-Zugriffen direkt nach der Mux-Umschaltung (scanSlots()) --
  // ohne die meldeten sich reale, vom I2C-Scanner bestätigte Geräte (0x21/0x23/0x27) fälschlich
  // als nicht vorhanden, vermutlich weil der Bus nach dem letzten Mux-Zugriff noch nicht bereit war.
  delay(10);
  okSlot1=i2cPresent(MCP_ES1); delay(5);
  okSlot2=i2cPresent(MCP_ES2); delay(5);
  okBedien=i2cPresent(MCP_BEDIEN); delay(5);
  okMbMcp=i2cPresent(MB_MCP_ADDR);
  if(okSlot1)mcpInit(MCP_ES1); if(okSlot2)mcpInit(MCP_ES2); if(okBedien)mcpInit(MCP_BEDIEN); if(okMbMcp)mcpInit(MB_MCP_ADDR);
}
// kompakte Bitmaske aller Erkennungs-Flags (für Änderungserkennung)
uint16_t detMask(){ return (slotSt[0].hasMcp?1:0)|(slotSt[0].hasEncA?2:0)|(slotSt[0].hasEncB?4:0)
  |(slotSt[1].hasMcp?8:0)|(slotSt[1].hasEncA?16:0)|(slotSt[1].hasEncB?32:0)
  |(okSlot1?64:0)|(okSlot2?128:0)|(okBedien?256:0)|(okAds48?512:0)|(okAds49?1024:0)|(okMbMcp?2048:0); }
// Modul-Status an ControlHub bzw. den Konfigurator (DET <key> <0/1>, dazu Encoder-Adressen und Zeitmessung)
// Kurzer Zustand in einer Zeile (Diagnose, 25.10.2026) -- ohne Modulsuche (anders als STATUS), daher auch im laufenden
// Betrieb abfragbar. ControlHub fragt einmal beim Verbinden ab. Maxima gelten seit der letzten Abfrage.
void emitStat(){
  char b[240]; snprintf(b,sizeof b,"STAT fw=%s up=%lu grund=%s vorphase=%lu vorlief=%lu vorrunde=%lu cfgfix=%u loop=%lu ser=%lu ctrl=%lu i2c=%lu out=%lu outfehl=%lu zulang=%lu srv=%d module=0x%03X",
    FW_VERSION,(unsigned long)(millis()/1000),neustartGrund,(unsigned long)vorPhase,(unsigned long)vorLaufzeitS,(unsigned long)vorRundeUs,
    cfgRepariert,(unsigned long)maxLoopUs,(unsigned long)maxSerialUs,(unsigned long)maxCtrlUs,(unsigned long)i2cFailCount,
    (unsigned long)outGut,(unsigned long)outFehl,(unsigned long)cmdZuLang,serverActive?1:0,detMask());
  Serial.println(b);
  maxLoopUs=0; maxSerialUs=0; maxCtrlUs=0;
}
void emitStatus(){
  Serial.print("DET s1grip ");Serial.println(slotSt[0].hasMcp?1:0);
  Serial.print("DET s1encA ");Serial.println(slotSt[0].hasEncA?1:0);
  Serial.print("DET s1encB ");Serial.println(slotSt[0].hasEncB?1:0);
  Serial.print("DET s2grip ");Serial.println(slotSt[1].hasMcp?1:0);
  Serial.print("DET s2encA ");Serial.println(slotSt[1].hasEncA?1:0);
  Serial.print("DET s2encB ");Serial.println(slotSt[1].hasEncB?1:0);
  Serial.print("DET s1encAaddr 0x");Serial.println(slotSt[0].addrEncA,HEX);
  Serial.print("DET s1encBaddr 0x");Serial.println(slotSt[0].addrEncB,HEX);
  Serial.print("DET s2encAaddr 0x");Serial.println(slotSt[1].addrEncA,HEX);
  Serial.print("DET s2encBaddr 0x");Serial.println(slotSt[1].addrEncB,HEX);
  Serial.print("DET es1 ");Serial.println(okSlot1?1:0);
  Serial.print("DET es2 ");Serial.println(okSlot2?1:0);
  Serial.print("DET bedien ");Serial.println(okBedien?1:0);
  Serial.print("DET finger ");Serial.println(okAds48?1:0);
  Serial.print("DET thumb ");Serial.println(okAds49?1:0);
  Serial.print("DET serverActive ");Serial.println(serverActive?1:0);
  Serial.print("DET goodOutStreak ");Serial.println(goodOutStreak);
  Serial.print("DET maxLoopUs ");Serial.println(maxLoopUs);
  Serial.print("DET maxSerialUs ");Serial.println(maxSerialUs);
  Serial.print("DET maxCtrlUs ");Serial.println(maxCtrlUs);
  maxLoopUs=0; maxSerialUs=0; maxCtrlUs=0;   // "seit dem letzten Auslesen" -- fuer den naechsten Check zuruecksetzen
}


inline void setBtn(uint8_t*arr,uint8_t g,uint8_t maxbits,bool on){ uint8_t i=g-1; if(i>=maxbits)return; if(on)arr[i>>3]|=(1<<(i&7)); else arr[i>>3]&=~(1<<(i&7)); }
void mapMcp(uint16_t gpio,const uint8_t*m,uint8_t n,uint8_t base,uint8_t*out){ for(uint8_t i=0;i<n;i++) setBtn(out,base+i,MAIN_BTNS,!((gpio>>m[i])&1)); }
int16_t mapAxisCfg(int raw,const Axis&a,int dzPct,int sensPct){ long d=raw-a.cen; long span=(d>0)?(a.hi-a.cen):(a.cen-a.lo); if(span<1)span=1;
  long dz=span*dzPct/100; if(d>-dz&&d<dz)return 0; long o=d*32767L/span; o=o*sensPct/100; if(o>32767)o=32767; if(o<-32767)o=-32767; return (int16_t)o; }
// Totbereich/Empfindlichkeit jetzt auch fuer Hebel-Achsen (25.08.2026, war vorher keins) -- gleiches
// Prinzip wie mapAxisCfg() oben, nur bezogen auf die (bereits als Delta vorliegende) Hebel-Auslenkung d.
int16_t leverAxis(int d,int mn,int mx,int dzPct,int sensPct){ int m=max(abs(mn),abs(mx)); if(m<1)m=1;
  int dz=m*dzPct/100; if(d>-dz&&d<dz)return 0; long o=(long)d*32767/m; o=o*sensPct/100; if(o>32767)o=32767; if(o<-32767)o=-32767; return (int16_t)o; }
int leverTh(int16_t mn,int16_t mx,int pct){ int m=max(abs((int)mn),abs((int)mx)); int t=(int)((long)m*pct/100); return t<150?150:t; }
inline void updLoHi(Axis&a,int v){ if(v<a.lo)a.lo=v; if(v>a.hi)a.hi=v; }

// ---- Tastatur-HID: Tastenname -> HID-Keycode/Modifier (USB HID Usage Tables, Boot-Keyboard) ----
#define HID_MOD_LCTRL  0x01
#define HID_MOD_LSHIFT 0x02
#define HID_MOD_LALT   0x04
#define HID_MOD_LGUI   0x08
#define HID_MOD_RCTRL  0x10
#define HID_MOD_RSHIFT 0x20
#define HID_MOD_RALT   0x40
#define HID_MOD_RGUI   0x80
// Alle Tasten einer vollen Tastatur, Namen wie in ControlHub (seit 2.13 auch Feststell, Num-Lock, Num-Enter,
// Num-Komma, Windows, Menue, F13-F24). Laengster Name: PRINTSCREEN (11 Zeichen, s. outKeys[]).
// Modifier-Tasten liefern Keycode 0 und setzen stattdessen ein Bit in *mod (ODER-verknuepft).
uint8_t hidKeycodeFor(const char* name, uint8_t* mod){
  if(!strcmp(name,"LSHIFT")){ *mod|=HID_MOD_LSHIFT; return 0; }
  if(!strcmp(name,"RSHIFT")){ *mod|=HID_MOD_RSHIFT; return 0; }
  if(!strcmp(name,"LCTRL")){  *mod|=HID_MOD_LCTRL;  return 0; }
  if(!strcmp(name,"RCTRL")){  *mod|=HID_MOD_RCTRL;  return 0; }
  if(!strcmp(name,"LALT")){   *mod|=HID_MOD_LALT;   return 0; }
  if(!strcmp(name,"RALT")){   *mod|=HID_MOD_RALT;   return 0; }
  if(strlen(name)==1){
    char c=name[0];
    if(c>='A'&&c<='Z') return 0x04+(c-'A');
    if(c>='a'&&c<='z') return 0x04+(c-'a');
    if(c>='1'&&c<='9') return 0x1E + (c-'1');
    if(c=='0') return 0x27;
  }
  if(!strcmp(name,"SPACE"))     return 0x2C;
  if(!strcmp(name,"ENTER"))     return 0x28;
  if(!strcmp(name,"ESC"))       return 0x29;
  if(!strcmp(name,"TAB"))       return 0x2B;
  if(!strcmp(name,"BACKSPACE")) return 0x2A;
  if(!strcmp(name,"UP"))        return 0x52;
  if(!strcmp(name,"DOWN"))      return 0x51;
  if(!strcmp(name,"LEFT"))      return 0x50;
  if(!strcmp(name,"RIGHT"))     return 0x4F;
  if(name[0]=='F' && strlen(name)<=3){ int n=atoi(name+1); if(n>=1&&n<=12) return 0x3A+(n-1); if(n>=13&&n<=24) return 0x68+(n-13); }
  // Erweiterung (08.08.2026) fuer den FS25-inputBinding.xml-Import (Tastatur-Actions):
  if(!strcmp(name,"MINUS"))       return 0x2D;
  if(!strcmp(name,"EQUALS"))      return 0x2E;
  if(!strcmp(name,"LBRACKET"))    return 0x2F;
  if(!strcmp(name,"RBRACKET"))    return 0x30;
  if(!strcmp(name,"BACKSLASH"))   return 0x31;
  if(!strcmp(name,"SEMICOLON"))   return 0x33;
  if(!strcmp(name,"QUOTE"))       return 0x34;
  if(!strcmp(name,"GRAVE"))       return 0x35;
  if(!strcmp(name,"COMMA"))       return 0x36;
  if(!strcmp(name,"PERIOD"))      return 0x37;
  if(!strcmp(name,"SLASH"))       return 0x38;
  if(!strcmp(name,"PRINTSCREEN")) return 0x46;
  if(!strcmp(name,"SCROLLLOCK"))  return 0x47;
  if(!strcmp(name,"PAUSE"))       return 0x48;
  if(!strcmp(name,"INSERT"))      return 0x49;
  if(!strcmp(name,"HOME"))        return 0x4A;
  if(!strcmp(name,"PAGEUP"))      return 0x4B;
  if(!strcmp(name,"DELETE"))      return 0x4C;
  if(!strcmp(name,"END"))         return 0x4D;
  if(!strcmp(name,"PAGEDOWN"))    return 0x4E;
  if(!strcmp(name,"KPDIV"))       return 0x54;
  if(!strcmp(name,"KPMUL"))       return 0x55;
  if(!strcmp(name,"KPMINUS"))     return 0x56;
  if(!strcmp(name,"KPPLUS"))      return 0x57;
  if(!strncmp(name,"KP",2) && strlen(name)==3){ char d=name[2]; if(d>='1'&&d<='9') return 0x59+(d-'1'); if(d=='0') return 0x62; }
  if(!strcmp(name,"LESS"))        return 0x64; // ISO-Zusatztaste "< >" neben LSHIFT
  // 2.13 (24.10.2026): die restlichen Tasten einer vollen Tastatur, damit die Bildschirmtastatur in ControlHub alles
  // senden kann (Hendrik: „bevor wir es doch irgendwo brauchen … jetzt mit aufnehmen“) -- Feststell, Num-Lock,
  // Num-Enter, Num-Komma, Windows links/rechts, Menütaste; F13–F24 oben bei den Funktionstasten
  if(!strcmp(name,"LGUI")){ *mod|=HID_MOD_LGUI; return 0; }
  if(!strcmp(name,"RGUI")){ *mod|=HID_MOD_RGUI; return 0; }
  if(!strcmp(name,"CAPSLOCK"))    return 0x39;
  if(!strcmp(name,"NUMLOCK"))     return 0x53;
  if(!strcmp(name,"KPENTER"))     return 0x58;
  if(!strcmp(name,"KPDOT"))       return 0x63;
  if(!strcmp(name,"MENU"))        return 0x65;
  return 0;
}
// Schubhebel (Unterbau ohne Federrueckstellung): seine Mitte nimmt die Firmware nie selbst -- nicht beim Start, nicht bei
// „Mittelstellung festlegen“ (CAL CENTER) und nicht beim Kalibrieren eines anderen Hebels. Sie kommt nur aus seiner eigenen
// Kalibrierung mit dem Schritt „Mitte festlegen“ (CAL CENTER leverA …; Hendrik 24.10.2026: „gerade beim Schubhebel muessen
// wir die Mittelstellung mit kalibrieren“). Welcher Unterbau verbaut ist, stellen Konfigurator bzw. ControlHub ein.
bool istSchubhebel(int s){ return (s==0?cfg.slot1Unterbau:cfg.slot2Unterbau)==UNTERBAU_SCHUBHEBEL; }
// Mittelstellung der Hebel-Encoder aus der aktuellen Stellung erfassen (Mittel aus 8 Messungen). nurEnc<0: alle ausser
// Schubhebeln (Start), sonst nur dieser Encoder (Beginn seiner Kalibrierung, auch beim Schubhebel als vorlaeufige Mitte).
void captureLeverCenter(int nurEnc=-1){
  for(int s=0;s<2;s++){
    const SlotCfg& sc=slot[s]; SlotState& st=slotSt[s];
    if(nurEnc<0 && istSchubhebel(s)) continue;
    if(st.hasEncA && (nurEnc<0||nurEnc==sc.encA)){ muxSelect(sc.chA); long su=0; for(int i=0;i<8;i++){int v=readAS5600(st.addrEncA); if(v>=0)su+=v;} *asCenP(sc.encA)=su/8; }
    if(st.hasEncB && (nurEnc<0||nurEnc==sc.encB)){ muxSelect(sc.chB); long su=0; for(int i=0;i<8;i++){int v=readAS5600(st.addrEncB); if(v>=0)su+=v;} *asCenP(sc.encB)=su/8; }
  }
  muxSelect(-1);
}
// Mitte eines Hebels auf die aktuelle Stellung setzen, die Anschlaege (min/max relativ zur Mitte) wandern mit
bool hebelMitte(int s,bool b){ const SlotCfg& sc=slot[s]; SlotState& st=slotSt[s];
  if(!(b?st.hasEncB:st.hasEncA)) return false;
  muxSelect(b?sc.chB:sc.chA); int a=readAS5600(b?st.addrEncB:st.addrEncA); muxSelect(-1); if(a<0) return false;
  int e=b?sc.encB:sc.encA; int16_t* cen=asCenP(e); int sh=shortestDelta(a,*cen); *cen=a; *asMinP(e)-=sh; *asMaxP(e)-=sh; return true; }

const char* calLbl(int g){ switch(g){case CG_THUMB:return"Thumbstick";case CG_LEVA:return"Hebel vor/zur.";case CG_LEVB:return"Hebel li/re";
  case CG_F1:return"Fingertip 1";case CG_F2:return"Fingertip 2";case CG_F3:return"Fingertip 3";case CG_F4:return"Fingertip 4";case CG_POTI:return"Poti";
  case CG_THUMB2:return"Thumbstick 2";case CG_LEVA2:return"Hebel 2 vor/zur.";case CG_LEVB2:return"Hebel 2 li/re";} return""; }
void calStart(int g){ calGroup=g; mode=MODE_CAL;
  switch(g){ case CG_THUMB: if(okAds49){ long sx=0,sy=0; for(int i=0;i<8;i++){sx+=ads49.readADC_SingleEnded(0);sy+=ads49.readADC_SingleEnded(1);}
      cfg.thumb[0].cen=sx/8; cfg.thumb[1].cen=sy/8; cfg.thumb[0].lo=cfg.thumb[0].hi=cfg.thumb[0].cen; cfg.thumb[1].lo=cfg.thumb[1].hi=cfg.thumb[1].cen; } break;
    // Kanaele 3/2 statt 2/3 (X/Y vertauscht, s. slot[1].adsX/adsY oben) -- Tasten-Test-Protokoll 25.08.2026.
    case CG_THUMB2: if(okAds49){ long sx=0,sy=0; for(int i=0;i<8;i++){sx+=ads49.readADC_SingleEnded(3);sy+=ads49.readADC_SingleEnded(2);}
      cfg.thumb2[0].cen=sx/8; cfg.thumb2[1].cen=sy/8; cfg.thumb2[0].lo=cfg.thumb2[0].hi=cfg.thumb2[0].cen; cfg.thumb2[1].lo=cfg.thumb2[1].hi=cfg.thumb2[1].cen; } break;
    case CG_F1: case CG_F2: case CG_F3: case CG_F4:{ int c=g-CG_F1; if(okAds48){ long s=0; for(int i=0;i<8;i++) s+=ads48.readADC_SingleEnded(FINGER_CH(c)); cfg.finger[c].cen=s/8; cfg.finger[c].lo=cfg.finger[c].hi=cfg.finger[c].cen; } } break;
    case CG_POTI:{ int p=readPoti(); cfg.potiLo=cfg.potiHi=p; } break;
    case CG_LEVA: captureLeverCenter(0); cfg.as0min=cfg.as0max=0; break;
    case CG_LEVB: captureLeverCenter(1); cfg.as1min=cfg.as1max=0; break;
    case CG_LEVA2: captureLeverCenter(2); cfg.as2min=cfg.as2max=0; break;
    case CG_LEVB2: captureLeverCenter(3); cfg.as3min=cfg.as3max=0; break; }
  Serial.print("OK CAL "); Serial.println(calLbl(g)); }
void calStop(){ if(calGroup>=CG_THUMB && calGroup<=CG_LEVB2){ cfg.calDone|=(1<<(calGroup-1));   // Achse als kalibriert markieren
    char b[44]; snprintf(b,44,"Kalibriert: %s",calLbl(calGroup)); logEvent(b); }
  calGroup=CG_NONE; mode=MODE_RUN; cfgSave(); Serial.println("OK CAL STOP"); }
// Mittelstellung ALLER Achsen aus den aktuellen (losgelassenen) Werten setzen – ganz zum Schluss (Schubhebel ausgenommen)
void calCenterAll(){
  if(okAds49){ long sx=0,sy=0,sx2=0,sy2=0; for(int i=0;i<8;i++){sx+=ads49.readADC_SingleEnded(0);sy+=ads49.readADC_SingleEnded(1);sx2+=ads49.readADC_SingleEnded(3);sy2+=ads49.readADC_SingleEnded(2);}
    cfg.thumb[0].cen=sx/8; cfg.thumb[1].cen=sy/8; cfg.thumb2[0].cen=sx2/8; cfg.thumb2[1].cen=sy2/8; }
  if(okAds48){ for(int c=0;c<4;c++){ long s=0; for(int i=0;i<8;i++) s+=ads48.readADC_SingleEnded(FINGER_CH(c)); cfg.finger[c].cen=s/8; } }
  for(int s=0;s<2;s++){ if(istSchubhebel(s)) continue; hebelMitte(s,false); hebelMitte(s,true); }
  logEvent("Mittelstellung gesetzt"); cfgSave(); }
void live(const char*k,int c,int lo,int hi){ Serial.print("LIVE ");Serial.print(k);Serial.print(' ');Serial.print(c);Serial.print(' ');Serial.print(lo);Serial.print(' ');Serial.println(hi); }
void calLoop(){ if(millis()-lastLive<120){ delay(2); return; } lastLive=millis();   // nur ~8×/s lesen+senden (sonst staut der CDC-Kanal)
  switch(calGroup){
    case CG_THUMB: if(okAds49){ int x=ads49.readADC_SingleEnded(0),y=ads49.readADC_SingleEnded(1); updLoHi(cfg.thumb[0],x); updLoHi(cfg.thumb[1],y);
      live("thumbX",x,cfg.thumb[0].lo,cfg.thumb[0].hi); live("thumbY",y,cfg.thumb[1].lo,cfg.thumb[1].hi); } break;
    case CG_THUMB2: if(okAds49){ int x=ads49.readADC_SingleEnded(3),y=ads49.readADC_SingleEnded(2); updLoHi(cfg.thumb2[0],x); updLoHi(cfg.thumb2[1],y);
      live("thumb2X",x,cfg.thumb2[0].lo,cfg.thumb2[0].hi); live("thumb2Y",y,cfg.thumb2[1].lo,cfg.thumb2[1].hi); } break;
    case CG_F1: case CG_F2: case CG_F3: case CG_F4:{ int c=calGroup-CG_F1; if(okAds48){ int v=ads48.readADC_SingleEnded(FINGER_CH(c)); updLoHi(cfg.finger[c],v);
      char k[9]; sprintf(k,"finger%d",c+1); live(k,v,cfg.finger[c].lo,cfg.finger[c].hi); } } break;
    case CG_POTI:{ int p=readPoti(); if(p<cfg.potiLo)cfg.potiLo=p; if(p>cfg.potiHi)cfg.potiHi=p; live("poti",p,cfg.potiLo,cfg.potiHi); } break;
    case CG_LEVA: if(okAS0){ muxSelect(0); int a=readAS5600(slotSt[0].addrEncA); muxSelect(-1); if(a>=0){ int d=shortestDelta(a,cfg.as0cen); if(d<cfg.as0min)cfg.as0min=d; if(d>cfg.as0max)cfg.as0max=d; live("leverA",d,cfg.as0min,cfg.as0max);} } break;
    case CG_LEVB: if(okAS1){ muxSelect(1); int a=readAS5600(slotSt[0].addrEncB); muxSelect(-1); if(a>=0){ int d=shortestDelta(a,cfg.as1cen); if(d<cfg.as1min)cfg.as1min=d; if(d>cfg.as1max)cfg.as1max=d; live("leverB",d,cfg.as1min,cfg.as1max);} } break;
    case CG_LEVA2: if(slotSt[1].hasEncA){ muxSelect(slot[1].chA); int a=readAS5600(slotSt[1].addrEncA); muxSelect(-1); if(a>=0){ int d=shortestDelta(a,cfg.as2cen); if(d<cfg.as2min)cfg.as2min=d; if(d>cfg.as2max)cfg.as2max=d; live("leverA2",d,cfg.as2min,cfg.as2max);} } break;
    case CG_LEVB2: if(slotSt[1].hasEncB){ muxSelect(slot[1].chB); int a=readAS5600(slotSt[1].addrEncB); muxSelect(-1); if(a>=0){ int d=shortestDelta(a,cfg.as3cen); if(d<cfg.as3min)cfg.as3min=d; if(d>cfg.as3max)cfg.as3max=d; live("leverB2",d,cfg.as3min,cfg.as3max);} } break;
  } }

void emitAll(){
  Serial.print("firmware=");Serial.println(FW_VERSION);
  Serial.print("wdtreboot=");Serial.println(wdtReboot?1:0);
  Serial.print("model=");Serial.println(cfg.model);
  Serial.print("expect=");Serial.println(cfg.expectMask);
  Serial.print("caldone=");Serial.println(cfg.calDone);
  Serial.print("levera=");Serial.println(cfg.leverModeA?"achse":"taster");
  Serial.print("leverb=");Serial.println(cfg.leverModeB?"achse":"taster");
  Serial.print("levera2=");Serial.println(cfg.leverModeA2?"achse":"taster");
  Serial.print("leverb2=");Serial.println(cfg.leverModeB2?"achse":"taster");
  Serial.print("leverthresha=");Serial.println(cfg.leverThreshA);
  Serial.print("leverthreshb=");Serial.println(cfg.leverThreshB);
  Serial.print("leverthresha2=");Serial.println(cfg.leverThreshA2);
  Serial.print("leverthreshb2=");Serial.println(cfg.leverThreshB2);
  Serial.print("leverdza=");Serial.println(cfg.leverDzA);
  Serial.print("leverdzb=");Serial.println(cfg.leverDzB);
  Serial.print("leverdza2=");Serial.println(cfg.leverDzA2);
  Serial.print("leverdzb2=");Serial.println(cfg.leverDzB2);
  Serial.print("leversensa=");Serial.println(cfg.leverSensA);
  Serial.print("leversensb=");Serial.println(cfg.leverSensB);
  Serial.print("leversensa2=");Serial.println(cfg.leverSensA2);
  Serial.print("leversensb2=");Serial.println(cfg.leverSensB2);
  for(int i=0;i<4;i++){ Serial.print("finger");Serial.print(i+1);Serial.print('=');Serial.println(cfg.fingerMode[i]?"taste":"achse"); }
  for(int i=0;i<4;i++){ Serial.print("fingerthresh");Serial.print(i+1);Serial.print('=');Serial.println(cfg.fingerThresh[i]); }
  for(int i=0;i<4;i++){ Serial.print("fingerdz");Serial.print(i+1);Serial.print('=');Serial.println(cfg.fingerDz[i]); }
  for(int i=0;i<4;i++){ Serial.print("fingersens");Serial.print(i+1);Serial.print('=');Serial.println(cfg.fingerSens[i]); }
  Serial.print("thumbmodex=");Serial.println(cfg.thumbModeX?"taste":"achse");
  Serial.print("thumbmodey=");Serial.println(cfg.thumbModeY?"taste":"achse");
  Serial.print("thumbmodex2=");Serial.println(cfg.thumbModeX2?"taste":"achse");
  Serial.print("thumbmodey2=");Serial.println(cfg.thumbModeY2?"taste":"achse");
  Serial.print("thumbdzx=");Serial.println(cfg.thumbDzX);
  Serial.print("thumbdzy=");Serial.println(cfg.thumbDzY);
  Serial.print("thumbsensx=");Serial.println(cfg.thumbSensX);
  Serial.print("thumbsensy=");Serial.println(cfg.thumbSensY);
  Serial.print("thumbbtnthreshx=");Serial.println(cfg.thumbBtnThreshX);
  Serial.print("thumbbtnthreshy=");Serial.println(cfg.thumbBtnThreshY);
  Serial.print("thumbdzx2=");Serial.println(cfg.thumbDzX2);
  Serial.print("thumbdzy2=");Serial.println(cfg.thumbDzY2);
  Serial.print("thumbsensx2=");Serial.println(cfg.thumbSensX2);
  Serial.print("thumbsensy2=");Serial.println(cfg.thumbSensY2);
  Serial.print("thumbbtnthreshx2=");Serial.println(cfg.thumbBtnThreshX2);
  Serial.print("thumbbtnthreshy2=");Serial.println(cfg.thumbBtnThreshY2);
  Serial.print("potiinv=");Serial.println(cfg.potiInv?"on":"off");
  { const char* m = cfg.bedienSwitchMode==0?"ebene":(cfg.bedienSwitchMode==1?"taste":"zuendschloss");
    Serial.print("bedienmode=");Serial.println(m); }
  Serial.print("ignitionpositions=");Serial.println(cfg.ignitionPositions);
  for(int i=0;i<3;i++){ Serial.print("ignitionpos");Serial.print(i);Serial.print("btn=");Serial.println(BTN_BEDIEN+cfg.ignitionPosBtn[i]); }
  for(int i=0;i<3;i++){ Serial.print("ignitionpos");Serial.print(i);Serial.print("hold=");Serial.println(cfg.ignitionPosHold[i]?"on":"off"); }
  Serial.print("slot1unterbau=");Serial.println(cfg.slot1Unterbau);
  Serial.print("slot2unterbau=");Serial.println(cfg.slot2Unterbau);
  Serial.print("slot1griff=");Serial.println(cfg.slot1Griff);
  Serial.print("slot2griff=");Serial.println(cfg.slot2Griff);
  // gespeicherte Kalibrierwerte als Schnappschuss (fuellt die Inline-Felder)
  live("thumbX",cfg.thumb[0].cen,cfg.thumb[0].lo,cfg.thumb[0].hi);
  live("thumbY",cfg.thumb[1].cen,cfg.thumb[1].lo,cfg.thumb[1].hi);
  live("thumb2X",cfg.thumb2[0].cen,cfg.thumb2[0].lo,cfg.thumb2[0].hi);
  live("thumb2Y",cfg.thumb2[1].cen,cfg.thumb2[1].lo,cfg.thumb2[1].hi);
  for(int c=0;c<4;c++){ char k[9]; sprintf(k,"finger%d",c+1); live(k,cfg.finger[c].cen,cfg.finger[c].lo,cfg.finger[c].hi); }
  live("poti",(cfg.potiLo+cfg.potiHi)/2,cfg.potiLo,cfg.potiHi);
  live("leverA",0,cfg.as0min,cfg.as0max);
  live("leverB",0,cfg.as1min,cfg.as1max);
  live("leverA2",0,cfg.as2min,cfg.as2max);
  live("leverB2",0,cfg.as3min,cfg.as3max);
  emitStatus();
  Serial.println("END"); }
int clampi(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }
bool applySet(const char*key,const char*val){
  if(!strcmp(key,"levera")){ cfg.leverModeA=!strcmp(val,"achse")?1:0; return true; }
  if(!strcmp(key,"leverb")){ cfg.leverModeB=!strcmp(val,"achse")?1:0; return true; }
  if(!strcmp(key,"levera2")){ cfg.leverModeA2=!strcmp(val,"achse")?1:0; return true; }
  if(!strcmp(key,"leverb2")){ cfg.leverModeB2=!strcmp(val,"achse")?1:0; return true; }
  if(!strcmp(key,"leverthresha")){ cfg.leverThreshA=clampi(atoi(val),10,95); return true; }
  if(!strcmp(key,"leverthreshb")){ cfg.leverThreshB=clampi(atoi(val),10,95); return true; }
  if(!strcmp(key,"leverthresha2")){ cfg.leverThreshA2=clampi(atoi(val),10,95); return true; }
  if(!strcmp(key,"leverthreshb2")){ cfg.leverThreshB2=clampi(atoi(val),10,95); return true; }
  if(!strcmp(key,"leverdza")){ cfg.leverDzA=clampi(atoi(val),0,25); return true; }
  if(!strcmp(key,"leverdzb")){ cfg.leverDzB=clampi(atoi(val),0,25); return true; }
  if(!strcmp(key,"leverdza2")){ cfg.leverDzA2=clampi(atoi(val),0,25); return true; }
  if(!strcmp(key,"leverdzb2")){ cfg.leverDzB2=clampi(atoi(val),0,25); return true; }
  if(!strcmp(key,"leversensa")){ cfg.leverSensA=clampi(atoi(val),50,150); return true; }
  if(!strcmp(key,"leversensb")){ cfg.leverSensB=clampi(atoi(val),50,150); return true; }
  if(!strcmp(key,"leversensa2")){ cfg.leverSensA2=clampi(atoi(val),50,150); return true; }
  if(!strcmp(key,"leversensb2")){ cfg.leverSensB2=clampi(atoi(val),50,150); return true; }
  for(int i=0;i<4;i++){ char k[14]; sprintf(k,"fingerthresh%d",i+1); if(!strcmp(key,k)){ cfg.fingerThresh[i]=clampi(atoi(val),10,95); return true; } }
  for(int i=0;i<4;i++){ char k[10]; sprintf(k,"fingerdz%d",i+1); if(!strcmp(key,k)){ cfg.fingerDz[i]=clampi(atoi(val),0,25); return true; } }
  for(int i=0;i<4;i++){ char k[12]; sprintf(k,"fingersens%d",i+1); if(!strcmp(key,k)){ cfg.fingerSens[i]=clampi(atoi(val),50,150); return true; } }
  if(!strcmp(key,"thumbdzx")){ cfg.thumbDzX=clampi(atoi(val),0,25); return true; }
  if(!strcmp(key,"thumbdzy")){ cfg.thumbDzY=clampi(atoi(val),0,25); return true; }
  if(!strcmp(key,"thumbsensx")){ cfg.thumbSensX=clampi(atoi(val),50,150); return true; }
  if(!strcmp(key,"thumbsensy")){ cfg.thumbSensY=clampi(atoi(val),50,150); return true; }
  if(!strcmp(key,"thumbbtnthreshx")){ cfg.thumbBtnThreshX=clampi(atoi(val),10,95); return true; }
  if(!strcmp(key,"thumbbtnthreshy")){ cfg.thumbBtnThreshY=clampi(atoi(val),10,95); return true; }
  if(!strcmp(key,"thumbdzx2")){ cfg.thumbDzX2=clampi(atoi(val),0,25); return true; }
  if(!strcmp(key,"thumbdzy2")){ cfg.thumbDzY2=clampi(atoi(val),0,25); return true; }
  if(!strcmp(key,"thumbsensx2")){ cfg.thumbSensX2=clampi(atoi(val),50,150); return true; }
  if(!strcmp(key,"thumbsensy2")){ cfg.thumbSensY2=clampi(atoi(val),50,150); return true; }
  if(!strcmp(key,"thumbbtnthreshx2")){ cfg.thumbBtnThreshX2=clampi(atoi(val),10,95); return true; }
  if(!strcmp(key,"thumbbtnthreshy2")){ cfg.thumbBtnThreshY2=clampi(atoi(val),10,95); return true; }
  if(!strcmp(key,"model")){ cfg.model=clampi(atoi(val),0,9); return true; }
  if(!strcmp(key,"expect")){ cfg.expectMask=(uint16_t)atol(val); return true; }
  if(!strcmp(key,"potiinv")){ cfg.potiInv=!strcmp(val,"on")?1:0; return true; }
  if(!strcmp(key,"bedienmode")){
    if(!strcmp(val,"ebene")) cfg.bedienSwitchMode=0;
    else if(!strcmp(val,"taste")) cfg.bedienSwitchMode=1;
    else if(!strcmp(val,"zuendschloss")) cfg.bedienSwitchMode=2;
    else return false;
    return true; }
  if(!strcmp(key,"ignitionpositions")){ int p=clampi(atoi(val),2,3); cfg.ignitionPositions=p; return true; }
  for(int i=0;i<3;i++){ char k[16]; sprintf(k,"ignitionpos%dbtn",i);
    if(!strcmp(key,k)){ int b=clampi(atoi(val),BTN_BEDIEN,BTN_BEDIEN+2); cfg.ignitionPosBtn[i]=b-BTN_BEDIEN; return true; } }
  for(int i=0;i<3;i++){ char k[17]; sprintf(k,"ignitionpos%dhold",i);
    if(!strcmp(key,k)){ cfg.ignitionPosHold[i]=!strcmp(val,"on")?1:0; return true; } }
  if(!strcmp(key,"slot1unterbau")){ cfg.slot1Unterbau=clampi(atoi(val),0,2); return true; }
  if(!strcmp(key,"slot2unterbau")){ cfg.slot2Unterbau=clampi(atoi(val),0,2); return true; }
  if(!strcmp(key,"slot1griff")){ cfg.slot1Griff=clampi(atoi(val),0,GRIFF_MAX); return true; }
  if(!strcmp(key,"slot2griff")){ cfg.slot2Griff=clampi(atoi(val),0,GRIFF_MAX); return true; }
  for(int i=0;i<4;i++){ char k[9]; sprintf(k,"finger%d",i+1); if(!strcmp(key,k)){ cfg.fingerMode[i]=!strcmp(val,"taste")?1:0; return true; } }
  if(!strcmp(key,"thumbmodex")){ cfg.thumbModeX=!strcmp(val,"taste")?1:0; return true; }
  if(!strcmp(key,"thumbmodey")){ cfg.thumbModeY=!strcmp(val,"taste")?1:0; return true; }
  if(!strcmp(key,"thumbmodex2")){ cfg.thumbModeX2=!strcmp(val,"taste")?1:0; return true; }
  if(!strcmp(key,"thumbmodey2")){ cfg.thumbModeY2=!strcmp(val,"taste")?1:0; return true; }
  return false; }

uint8_t hx1(char c){ if(c>='0'&&c<='9')return c-'0'; if(c>='a'&&c<='f')return c-'a'+10; if(c>='A'&&c<='F')return c-'A'+10; return 0; }

// ---- OUT-Verarbeitung: Mapping-Ergebnis von ControlHub ----
int16_t outAxis[16]={0};                      // -32768..32767, gleiche Reihenfolge wie sendFrame()
// 16 Byte -- MUSS zu rMain.btn (MainReport) passen, sonst liest memcpy(rMain.btn,outBtn,sizeof(rMain.btn))
// weiter unten ueber das Ende von outBtn hinaus.
uint8_t outBtn[16]={0};                       // gleiches Bit-Layout wie rMain.btn (Tasten 1..128)
#define OUT_KEY_MAX 6
#define OUT_KEY_LEN 12                        // laengster Tastenname PRINTSCREEN = 11 Zeichen + Nullbyte (bis 24.10.2026
                                              // 10: PRINTSCREEN und SCROLLLOCK wurden abgeschnitten und kamen nie an)
char outKeys[OUT_KEY_MAX][OUT_KEY_LEN]; uint8_t outKeyCount=0;   // aktuell laut ControlHub gehaltene Tasten (Vollzustand)

// CraftDeck-Schalter (D1=GPB2 + D2=GPB3): frei konfigurierbar als Ebenenschalter/Tastenmodus/
// Zuendschloss, s. cfg.bedienSwitchMode + runController(). ignitionPrevZustand/ignitionPulseBtn/
// -Until werden von Tastenmodus UND Zuendschloss-Impuls-Positionen gemeinsam genutzt (nie gleichzeitig
// aktiv, da immer nur EIN Modus zur Zeit gilt).
uint8_t ignitionPrevZustand=0; unsigned long ignitionPulseUntil=0; uint8_t ignitionPulseBtn=0;
#define IGNITION_PULSE_MS 500       // Tastenmodus: Dauer des kurzen Tasten-Impulses bei jedem Wechsel
#define IGNITION_POS_PULSE_MS 700   // Zuendschloss-Positionen im Impuls-Modus: 0,7s (Hendriks Vorgabe)

// Parst "<OUT>GP:AXES:v0,...,v15 GP:BUTTONS:v0,...,v127 K:key1,key2,...</OUT>" (ohne Zeilenende, das ist schon in handleSerial() entfernt).
// Gibt true zurueck, wenn beide Pflichtblöcke (Achsen+Buttons) vollständig und wohlgeformt waren.
bool parseOut(char* s){
  char* body=s+5;                       // hinter "<OUT>"
  char* end=strstr(body,"</OUT>");
  if(!end) return false;
  *end=0;
  int16_t nAxis[16]; uint8_t nBtn[16]={0};   // 16 statt 14 Byte -- muss zu outBtn[] passen (s. dort)
  char nKeys[OUT_KEY_MAX][OUT_KEY_LEN]; uint8_t nKeyCount=0;
  bool gotAxes=false, gotBtns=false;
  char* saveOuter; char* saveInner;
  char* tok=strtok_r(body," ",&saveOuter);
  while(tok){
    if(!strncmp(tok,"GP:AXES:",8)){
      int i=0; char* v=strtok_r(tok+8,",",&saveInner);
      while(v && i<16){ nAxis[i]=(int16_t)(atol(v)-32768); i++; v=strtok_r(NULL,",",&saveInner); }
      gotAxes=(i==16);
    } else if(!strncmp(tok,"GP:BUTTONS:",11)){
      // Alle MAIN_BTNS Tasten (bis 25.08.2026 bei 100 gedeckelt: CraftSlot 2 ab Taste 101 kam im Server-Modus
      // nie an). Gegenstueck in ControlHub: outputGenerator.js, MAIN_BUTTON_COUNT = 128.
      int i=0; char* v=strtok_r(tok+11,",",&saveInner);
      while(v && i<MAIN_BTNS){ setBtn(nBtn,i+1,MAIN_BTNS, atoi(v)!=0); i++; v=strtok_r(NULL,",",&saveInner); }
      gotBtns=(i==MAIN_BTNS);
    } else if(!strncmp(tok,"K:",2)){
      char* p=tok+2;
      if(*p){ char* v=strtok_r(p,",",&saveInner);
        while(v && nKeyCount<OUT_KEY_MAX){ strncpy(nKeys[nKeyCount],v,OUT_KEY_LEN-1); nKeys[nKeyCount][OUT_KEY_LEN-1]=0; nKeyCount++; v=strtok_r(NULL,",",&saveInner); } }
    }
    tok=strtok_r(NULL," ",&saveOuter);
  }
  if(!gotAxes || !gotBtns) return false;   // unvollständige Nachricht -- verwerfen, nicht als gültig werten
  memcpy(outAxis,nAxis,sizeof(outAxis)); memcpy(outBtn,nBtn,sizeof(outBtn));
  memcpy(outKeys,nKeys,sizeof(outKeys)); outKeyCount=nKeyCount;
  return true;
}

void processCmd(char*s){
  if(!strncmp(s,"<OUT>",5)){
    if(parseOut(s)){ outGut++; lastValidOut=millis(); if(goodOutStreak<255) goodOutStreak++; if(!serverActive && goodOutStreak>=3) serverActive=true; }
    else outFehl++;
    return;
  }
  if(!strcmp(s,"PING")){ Serial.println("PONG"); return; }
  if(!strcmp(s,"EXPORT")){ const uint8_t*p=(const uint8_t*)&cfg; Serial.print("CFG ");
    for(size_t i=0;i<sizeof(cfg);i++){ if(p[i]<16)Serial.print('0'); Serial.print(p[i],HEX); } Serial.println(); return; }
  if(!strncmp(s,"IMPORT ",7)){ const char*h=s+7; if(strlen(h)!=sizeof(cfg)*2){ Serial.println("ERR IMPORT len"); return; }
    Settings tmp; uint8_t*d=(uint8_t*)&tmp; for(size_t i=0;i<sizeof(cfg);i++) d[i]=(hx1(h[i*2])<<4)|hx1(h[i*2+1]);
    if(tmp.magic!=CFG_MAGIC||tmp.version!=CFG_VERSION){ Serial.println("ERR IMPORT version"); return; }
    cfg=tmp; cfgSave(); mode=MODE_RUN; logEvent("Konfiguration importiert"); Serial.println("OK IMPORT"); return; }
  if(!strcmp(s,"MON")){ monEnabled=!monEnabled; Serial.print("MON=");Serial.println(monEnabled?"on":"off"); return; }
  if(!strcmp(s,"GETALL")){ emitAll(); return; }
  if(!strncmp(s,"LEARN",5)){ const char*p=s+5; while(*p==' ')p++; learnMode=!strncmp(p,"ON",2); memset(rawPrev,0,sizeof(rawPrev)); Serial.print("LEARN=");Serial.println(learnMode?"on":"off"); return; }
  if(!strcmp(s,"STATUS")){ detectAll(); emitStatus(); return; }
  if(!strcmp(s,"DIAG")){ uint16_t n=(logCount<LOGN)?logCount:LOGN; uint8_t start=(logCount<LOGN)?0:logHead;
    for(uint16_t i=0;i<n;i++){ Serial.print("DIAG "); Serial.println(logBuf[(start+i)%LOGN]); }
    Serial.println("DIAGEND"); return; }
  if(!strcmp(s,"VER")){ Serial.print("firmware=");Serial.println(FW_VERSION); return; }
  if(!strcmp(s,"STAT")){ emitStat(); return; }
  if(!strcmp(s,"POTI ON")){ potiEnabled=true; Serial.println("POTI=on"); return; }
  if(!strcmp(s,"POTI OFF")){ potiEnabled=false; Serial.println("POTI=off"); return; }
  if(!strncmp(s,"BEEP",4)){ char*p=s+4; while(*p==' ')p++; long ms=*p?atol(p):150; if(ms<1)ms=1; if(ms>5000)ms=5000;
    beep((uint16_t)ms); Serial.print("OK BEEP ");Serial.println(ms); return; }
  if(!strncmp(s,"CAL ",4)){ char*g=s+4;
    if(!strcmp(g,"STOP")) calStop();
    else if(!strcmp(g,"ABORT")){ cfgLoad(); calGroup=CG_NONE; mode=MODE_RUN; Serial.println("OK CAL ABORT"); }  // verwirft die aktuelle Gruppe (kein Speichern)
    else if(!strcmp(g,"CENTER")){ calCenterAll(); Serial.println("OK CAL CENTER"); }
    else if(!strncmp(g,"CENTER ",7)){   // Mitte eines einzelnen Hebels (Schubhebel), waehrend seiner Kalibrierung: CAL STOP speichert
      const char* h=g+7; int s=!strcmp(h,"leverA")||!strcmp(h,"leverB")?0:(!strcmp(h,"leverA2")||!strcmp(h,"leverB2")?1:-1); bool b=(h[5]=='B');
      if(s>=0 && hebelMitte(s,b)){ if(mode!=MODE_CAL) cfgSave(); logEvent("Hebel-Mitte gesetzt"); Serial.print("OK CAL CENTER "); Serial.println(h); }
      else Serial.println("ERR CAL CENTER"); }
    else if(!strcmp(g,"RESET")){ cfgDefaults(); captureLeverCenter(); cfgSave(); calGroup=CG_NONE; mode=MODE_RUN; logEvent("Werkseinstellung"); Serial.println("OK CAL RESET"); }
    else if(!strcmp(g,"thumb"))calStart(CG_THUMB); else if(!strcmp(g,"leverA"))calStart(CG_LEVA); else if(!strcmp(g,"leverB"))calStart(CG_LEVB);
    else if(!strcmp(g,"finger1"))calStart(CG_F1); else if(!strcmp(g,"finger2"))calStart(CG_F2); else if(!strcmp(g,"finger3"))calStart(CG_F3); else if(!strcmp(g,"finger4"))calStart(CG_F4);
    else if(!strcmp(g,"poti"))calStart(CG_POTI);
    else if(!strcmp(g,"thumb2"))calStart(CG_THUMB2); else if(!strcmp(g,"leverA2"))calStart(CG_LEVA2); else if(!strcmp(g,"leverB2"))calStart(CG_LEVB2);
    else Serial.println("ERR CAL");
    return; }
  if(!strncmp(s,"SET ",4)){ char*r=s+4; char*sp=strchr(r,' '); if(!sp){Serial.println("ERR SET");return;} *sp=0; const char*key=r,*val=sp+1;
    if(applySet(key,val)){ cfgSave(); Serial.print("OK ");Serial.print(key);Serial.print('=');Serial.println(val); }
    else { Serial.print("ERR key "); Serial.println(key); }
    return; }
  Serial.println("ERR");
}
// Lesen in Stuecken unter der USB-Sperre (s. usbSperren()); ausgewertet wird danach ohne Sperre, denn die Antworten
// (z. B. GETALL) koennen den Sendepuffer fuellen und auf USB warten.
void handleSerial(){
  for(;;){
    char st[64]; int n=0;
    if(!usbSperren()) return;
    while(n<(int)sizeof(st) && Serial.available()) st[n++]=Serial.read();
    usbFreigeben();
    if(!n) return;
    for(int i=0;i<n;i++){ char c=st[i];
      if(c=='\n'||c=='\r'){ if(cmdLen>=sizeof(cmdBuf)) cmdLen=0;   // zu lange Zeile: verwerfen (s. unten)
        else if(cmdLen){cmdBuf[cmdLen]=0; processCmd(cmdBuf); cmdLen=0;} }
      else if(cmdLen<sizeof(cmdBuf)-1) cmdBuf[cmdLen++]=c;
      else if(cmdLen==sizeof(cmdBuf)-1){ cmdZuLang++; cmdLen=sizeof(cmdBuf); }   // Zeile zu lang: einmal zaehlen, Rest bis zum Zeilenende verwerfen
    }
  }
}

void setup(){
  wdtReboot = watchdog_caused_reboot();   // war der letzte Neustart ein Watchdog-Reboot?
  // watchdog_caused_reboot() ist ungenau: "reason"-Register wird auch von ANDEREN Resets ueber die
  // Watchdog-Peripherie gesetzt (z.B. interne Crash-/Fault-Handler), nicht nur vom eigenen
  // 4-Sekunden-Timeout (wdt_begin/wdt_reset). watchdog_enable_caused_reboot() prueft zusaetzlich
  // einen Marker, der NUR von einem echten Timeout gesetzt wird -- damit koennen wir unterscheiden,
  // ob es wirklich unser Timeout war oder ein anderer interner Reset/Absturz.
  bool wdtTimeoutSpecifically = watchdog_enable_caused_reboot();
  // Grund des letzten Neustarts (Diagnose, 25.10.2026) -- vor allem anderen, die Scratch-Register werden unten geleert
  vorBekannt = watchdog_hw->scratch[1]==SCR_MARKE || watchdog_hw->scratch[1]==SCR_ABSTURZ;
  if(vorBekannt){ vorRundeUs=watchdog_hw->scratch[0]; vorPhase=watchdog_hw->scratch[2]; vorLaufzeitS=watchdog_hw->scratch[3]; }
  if(watchdog_hw->scratch[1]==SCR_ABSTURZ) neustartGrund="absturz";
  else if(wdtTimeoutSpecifically) neustartGrund="watchdog";   // loop() hing ueber 4 s
  else if(wdtReboot) neustartGrund="software";                // z. B. nach dem Aufspielen einer Firmware
#if PICO_RP2040
  else if(vreg_and_chip_reset_hw->chip_reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS) neustartGrund="reset";
  else if(vreg_and_chip_reset_hw->chip_reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) neustartGrund="strom";   // Einstecken/Strom weg
#endif
  // ---- USB ZUERST: HID-Interfaces registrieren, BEVOR die langsame I2C-Init laeuft ----
  // (sonst mountet sich das Geraet frueh als reines CDC-Geraet und die HID-Gamepads fehlen komplett)
  TinyUSBDevice.setID(USB_VID,USB_PID); TinyUSBDevice.setDeviceVersion(0x0200);   // bcdDevice hoch -> Windows liest den Deskriptor frisch ein
  TinyUSBDevice.setManufacturerDescriptor(NAME_MANUF); TinyUSBDevice.setProductDescriptor(NAME_PROD);
  usb_main.setPollInterval(2); usb_main.setReportDescriptor(desc_main,sizeof(desc_main)); usb_main.setStringDescriptor(NAME_MAIN);
#if USB_SEC_ENABLED
  usb_sec.setPollInterval(2);  usb_sec.setReportDescriptor(desc_sec,sizeof(desc_sec));    usb_sec.setStringDescriptor(NAME_SEC);
#endif
  usb_kbd.setPollInterval(2);  usb_kbd.setReportDescriptor(desc_kbd,sizeof(desc_kbd));    usb_kbd.setStringDescriptor(NAME_KBD);
  // Rueckgabewerte von begin() mitschreiben (wurden bisher ignoriert) -- false heisst, das
  // HID-Interface wurde NICHT registriert (z.B. weil CFG_TUD_HID der TinyUSB-Bibliothek zu klein ist), ohne dass das
  // sonst irgendwo auffaellt. Diagnose ueber DIAG-Boot-Log sichtbar (s. logEvent() weiter unten).
  bool okUsbMain=usb_main.begin();
#if USB_SEC_ENABLED
  bool okUsbSec=usb_sec.begin();
#else
  bool okUsbSec=false;
#endif
  bool okUsbKbd=usb_kbd.begin();
  if(TinyUSBDevice.mounted()){ TinyUSBDevice.detach(); delay(10); TinyUSBDevice.attach(); }   // schon (CDC) gemountet -> neu enumerieren, damit HID mitkommt
  Serial.begin(115200); EEPROM.begin(EE_SIZE); cfgLoad(); cfgRepariert=cfgPruefen();
  pinMode(PIEZO_PIN,OUTPUT); digitalWrite(PIEZO_PIN,LOW);
  Wire.setSDA(I2C_SDA); Wire.setSCL(I2C_SCL); Wire.begin(); Wire.setClock(100000); // Standard-Takt 100 kHz statt 400 kHz -- sicherer fuer die Signalqualitaet am langen Kabelweg zu CraftPad/CraftDeck
  Wire.setTimeout(50, true);   // hängende I2C-Übertragung nach 50ms abbrechen + Bus zuruecksetzen (kein Dauer-Freeze)
  okAds48=ads48.begin(ADS_FINGER_ADDR); if(okAds48){ads48.setGain(GAIN_ONE);ads48.setDataRate(RATE_ADS1115_860SPS);}
  okAds49=ads49.begin(ADS_THUMB_ADDR);  if(okAds49){ads49.setGain(GAIN_ONE);ads49.setDataRate(RATE_ADS1115_860SPS);}
  // Deutlich längere Wartezeit vor der ersten Erkennung: MCP23017-Boards (CraftPad/CraftDeck)
  // haben ein RC-Power-On-Reset-Netzwerk am RESET-Pin (R=10k, C=100nF) -- bei längerem Kabelweg
  // vom Mainboard kann die Versorgungsspannung dort langsamer ansteigen, wodurch der Chip zum
  // Zeitpunkt der ersten Prüfung noch im Reset haengen und dauerhaft als "nicht erkannt" gelten
  // wuerde (detectAll() legt das Ergebnis beim Boot fest und prueft danach nicht mehr automatisch nach).
  delay(300); detectAll(); delay(100); detectAll();
  captureLeverCenter();   // Hebel mit Federrueckstellung stehen beim Start in der Mitte (Schubhebel ausgenommen)
  { unsigned long t0=millis(); while(!TinyUSBDevice.mounted() && millis()-t0<2000) delay(1); }   // kurz auf Host-Enumerierung warten (gebounded, haengt nicht bei Nur-Strom)
  Serial.println("CraftConsole Pro bereit.");
  beep(80);   // Startton: kurzer Piep als hoerbares Boot-Feedback (Hendriks Wunsch, 13.08.2026)
  { char b[64]; snprintf(b,64,"Boot FW %s wdt=%d wdtTimeout=%d module=0x%03X",FW_VERSION,wdtReboot?1:0,wdtTimeoutSpecifically?1:0,detMask()); logEvent(b); }
  { char b[56]; snprintf(b,56,"USB HID: CFG_TUD_HID=%d main=%d sec=%d kbd=%d",CFG_TUD_HID,okUsbMain?1:0,okUsbSec?1:0,okUsbKbd?1:0); logEvent(b); }
  { char b[LOGW]; snprintf(b,LOGW,"Neustart: %s",neustartGrund); logEvent(b); }
  // Stand VOR diesem Neustart (ueberlebt in den Watchdog-Scratch-Registern, s. loop()) -- nur gueltig, wenn der Marker
  // zeigt, dass diese Firmware die Werte geschrieben hat. Phase = Stelle, an der die loop() zuletzt war:
  // 1=vor handleSerial 2=vor runController 3=CraftSlots 4=Fingertips 5=CraftPads/CraftDeck
  // 6=vor sendFrame 7=nach sendFrame 8=vor dem USB-HID-Report 11=HID-Report fertig.
  if(vorBekannt){
    char b[48]; snprintf(b,48,"Letzte Rundenzeit vor Reset: %lu us",(unsigned long)vorRundeUs); logEvent(b);
    char b2[48]; snprintf(b2,48,"Letzte Phase vor Reset: %lu",(unsigned long)vorPhase); logEvent(b2);
    char b3[48]; snprintf(b3,48,"Lief vor dem Reset: %lu s",(unsigned long)vorLaufzeitS); logEvent(b3);
  }
  if(cfgRepariert){ char b[LOGW]; snprintf(b,LOGW,"Einstellungen: %u ungueltige Werte auf Standard",cfgRepariert); logEvent(b); }
  watchdog_hw->scratch[0]=0; watchdog_hw->scratch[1]=0; watchdog_hw->scratch[2]=0; watchdog_hw->scratch[3]=0;
  rp2040.wdt_begin(4000);   // Watchdog: haengt die loop() >4s (z.B. I2C-Lockup), rebootet der Pico selbst
}

// Streamt den rohen physischen Zustand (bn[] + aktuelle Achsen) an ControlHub, max. 50 Hz, non-blocking.
// Format kompatibel zum bestehenden frameParser.js (Praefix nur beim ersten Wert je Block, Achsen 0-65535-kodiert).
void sendFrame(uint8_t* bn){
  if(millis()-lastFrameSend<20) return;   // 50 Hz Obergrenze, entkoppelt von der HID-Sende-Rate
  lastFrameSend=millis();
  int16_t ax[16]={ rMain.x,rMain.y,rMain.z,rMain.rx,rMain.ry,rMain.rz,rMain.slider,rMain.dial,
                   rSec.x,rSec.y,rSec.z,rSec.rx,rSec.ry,rSec.rz,rSec.slider,rSec.dial };
  int n=0;
  // Alle MAIN_BTNS Tasten (bis 24.08.2026 fest 100 -- ControlHub sah CraftSlot 2 ab Taste 101 nicht).
  n+=snprintf(frameBuf+n,sizeof(frameBuf)-n,"<FRAME>B:");
  for(int g=1;g<=MAIN_BTNS;g++){
    bool v=bn[(g-1)>>3]&(1<<((g-1)&7));
    n+=snprintf(frameBuf+n,sizeof(frameBuf)-n, g==1?"%d":" %d", v?1:0);
  }
  n+=snprintf(frameBuf+n,sizeof(frameBuf)-n," A:");
  for(int i=0;i<16;i++){
    uint16_t u=(uint16_t)((int32_t)ax[i]+32768);
    n+=snprintf(frameBuf+n,sizeof(frameBuf)-n, i==0?"%u":" %u", u);
  }
  n+=snprintf(frameBuf+n,sizeof(frameBuf)-n,"</FRAME>\n");
  if(n<=0 || n>=(int)sizeof(frameBuf)) return;   // Puffer-Sicherung, sollte bei den festen Zaehlern nie eintreten
  // Nur schreiben, wenn der KOMPLETTE Frame auf einmal passt (Sendepuffer 512 Byte, Voreinstellung der
  // TinyUSB-Bibliothek -- reicht dafuer fast immer). Wichtig: "nur pruefen ob >0 Byte frei sind" reicht
  // NICHT -- wenn der Puffer WAEHREND des Schreibens vollläuft, wartet Serial.write() intern bis zu
  // 1 Sekunde auf Nachschub (blockiert dabei die GESAMTE loop()). Mehrere solcher Frames in Folge
  // haben den Watchdog (4s) ausgeloest -> kompletter Neustart -> USB-Verbindungsabbruch. Mit dieser
  // Prüfung wird Serial.write() nur aufgerufen, wenn es in einem Zug fertig wird -- sonst wird der
  // Frame komplett verworfen (naechster Versuch in 20ms) statt teilweise zu schreiben und zu warten.
  if(usbSperren()){   // s. usbSperren(): passt der Frame ganz, wartet write() nicht
    if(Serial.availableForWrite()>=n) Serial.write((const uint8_t*)frameBuf,n);
    usbFreigeben();
  }
  // sonst: Zyklus verwerfen statt zu blockieren -- naechster Versuch in 20 ms
}

void runController(){
  uint8_t bn[16]={0};                                 // Gerät 1: alle Tasten
  int16_t slotAx[2][4]={{0,0,0,0},{0,0,0,0}};         // je Slot: ThumbX, ThumbY, HebelA, HebelB

  // ---- CraftSlots (generisch, auto-erkannt) ----
  watchdog_hw->scratch[2]=3;   // Phase-Marker fuer die Fehlersuche -- s. loop()/emitStatus()
  for(int s=0;s<2;s++){
    const SlotCfg& sc=slot[s]; SlotState& st=slotSt[s];
    muxSelect(sc.chA);
    if(st.hasMcp) mapMcp(mcpRead(GRIP_MCP_ADDR),mapDirectus,16,sc.btnGrip,bn);   // Grip-MCP 16 Tasten
    int aA = st.hasEncA ? readAS5600(st.addrEncA) : -1;
    int aB = -1; if(st.hasEncB){ muxSelect(sc.chB); aB=readAS5600(st.addrEncB); }
    muxSelect(-1);
    // Thumbstick (Hauptbus-ADS 0x49) -- Slot0 nutzt cfg.thumb, Slot1 sein eigenes cfg.thumb2
    // (beide sitzen als getrennte physische Sticks auf demselben ADS1115, je 2 eigene Kanaele).
    // Thumbstick-als-Taste (24.08.2026): je Achse X/Y einzeln umschaltbar, analog zum Fingertip-Muster
    // weiter unten (Rohwert-Differenz zum kalibrierten Mittelpunkt, ueber Schwelle = Taste). Vorzeichen
    // (sc.thumbSignX/Y) aus dem Tasten-Test-Protokoll 25.08.2026 ermittelt -- Span/Threshold bleiben
    // bewusst auf dem UNVORZEICHENTEN Rohwert (d), da .lo/.hi in calLoop() ebenfalls unvorzeichnet
    // (roher ADC-Extremwert) erfasst werden; nur die finale Links/Rechts- bzw. Hoch/Runter-Entscheidung
    // (dSigned) braucht die Korrektur.
    if(okAds49){ int tx=ads49.readADC_SingleEnded(sc.adsX), ty=ads49.readADC_SingleEnded(sc.adsY);
      const Axis& cx=(s==0)?cfg.thumb[0]:cfg.thumb2[0]; const Axis& cy=(s==0)?cfg.thumb[1]:cfg.thumb2[1];
      uint8_t tmX=(s==0)?cfg.thumbModeX:cfg.thumbModeX2, tmY=(s==0)?cfg.thumbModeY:cfg.thumbModeY2;
      uint8_t thumbBtnBase=(s==0)?BTN_S1_THUMB:BTN_S2_THUMB;
      // Totbereich/Empfindlichkeit/Ausloeseschwelle PRO EINZELNER ACHSE (25.08.2026, Hendriks Wunsch) --
      // je Slot getrennte X- und Y-Werte statt eines fuer alle 4 Thumbstick-Achsen geteilten Wertes.
      uint8_t dzX=(s==0)?cfg.thumbDzX:cfg.thumbDzX2, dzY=(s==0)?cfg.thumbDzY:cfg.thumbDzY2;
      uint8_t sensX=(s==0)?cfg.thumbSensX:cfg.thumbSensX2, sensY=(s==0)?cfg.thumbSensY:cfg.thumbSensY2;
      uint8_t threshX=(s==0)?cfg.thumbBtnThreshX:cfg.thumbBtnThreshX2, threshY=(s==0)?cfg.thumbBtnThreshY:cfg.thumbBtnThreshY2;
      if(tmX==1){ long d=tx-cx.cen; long span=(d>0)?(cx.hi-cx.cen):(cx.cen-cx.lo); if(span<1)span=1;
        bool over=(labs(d)*100/span)>threshX; long dSigned=d*sc.thumbSignX;
        setBtn(bn,thumbBtnBase+0,MAIN_BTNS, over&&dSigned<0);   // links
        setBtn(bn,thumbBtnBase+1,MAIN_BTNS, over&&dSigned>0);   // rechts
        slotAx[s][0]=0; }
      else slotAx[s][0]=mapAxisCfg(tx,cx,dzX,sensX);
      if(tmY==1){ long d=ty-cy.cen; long span=(d>0)?(cy.hi-cy.cen):(cy.cen-cy.lo); if(span<1)span=1;
        bool over=(labs(d)*100/span)>threshY; long dSigned=d*sc.thumbSignY;
        setBtn(bn,thumbBtnBase+2,MAIN_BTNS, over&&dSigned<0);   // hoch
        setBtn(bn,thumbBtnBase+3,MAIN_BTNS, over&&dSigned>0);   // runter
        slotAx[s][1]=0; }
      else slotAx[s][1]=mapAxisCfg(ty,cy,dzY,sensY); }
    // Hebel-Encoder -> Achse oder 2 Tasten (vor/zurück)
    int dA=(aA>=0)?shortestDelta(aA,*asCenP(sc.encA)):0;
    int dB=(aB>=0)?shortestDelta(aB,*asCenP(sc.encB)):0;
    // Vor/zurueck bei BEIDEN CraftSlots invertiert (war bis 25.08.2026 nur bei Slot0 angenommen --
    // Hendriks Tasten-Test-Protokoll zeigt denselben Effekt jetzt auch bei CraftSlot2, Taste 102/103).
    dA=-dA;
    uint8_t lmA=(s==0)?cfg.leverModeA:cfg.leverModeA2, lmB=(s==0)?cfg.leverModeB:cfg.leverModeB2;
    // Ausloeseschwelle/Totbereich/Empfindlichkeit PRO EINZELNER Hebel-Achse (25.08.2026) statt eines
    // geteilten Wertes fuer alle 4.
    uint8_t threshA=(s==0)?cfg.leverThreshA:cfg.leverThreshA2, threshB=(s==0)?cfg.leverThreshB:cfg.leverThreshB2;
    uint8_t dzA=(s==0)?cfg.leverDzA:cfg.leverDzA2, dzB=(s==0)?cfg.leverDzB:cfg.leverDzB2;
    uint8_t sensA=(s==0)?cfg.leverSensA:cfg.leverSensA2, sensB=(s==0)?cfg.leverSensB:cfg.leverSensB2;
    int thA=leverTh(*asMinP(sc.encA),*asMaxP(sc.encA),threshA);
    int thB=leverTh(*asMinP(sc.encB),*asMaxP(sc.encB),threshB);
    if(aA>=0){ if(lmA==0){ setBtn(bn,sc.btnLever+0,MAIN_BTNS,dA>thA); setBtn(bn,sc.btnLever+1,MAIN_BTNS,dA<-thA); }
               else slotAx[s][2]=leverAxis(dA,*asMinP(sc.encA),*asMaxP(sc.encA),dzA,sensA); }
    if(aB>=0){ if(lmB==0){ setBtn(bn,sc.btnLever+2,MAIN_BTNS,dB>thB); setBtn(bn,sc.btnLever+3,MAIN_BTNS,dB<-thB); }
               else slotAx[s][3]=leverAxis(dB,*asMinP(sc.encB),*asMaxP(sc.encB),dzB,sensB); }
    // Aux-Tasten (D1..D5)
    if(okMbMcp){   // Aux immer aktiv (offene Leitung = nicht gedrückt, stört nicht)
      uint16_t mb=mcpRead(MB_MCP_ADDR);
      for(int k=0;k<5;k++) setBtn(bn,sc.btnAux+k,MAIN_BTNS,!((mb>>sc.aux[k])&1));
    }
  }

  // ---- Fingertips (ADS 0x48): je Achse vor/zurück = Taste, sonst Achse ----
  watchdog_hw->scratch[2]=4;
  int16_t fa[4];
  for(int c=0;c<4;c++){ int f=okAds48?ads48.readADC_SingleEnded(FINGER_CH(c)):cfg.finger[c].cen;
    if(cfg.fingerMode[c]==1){
      long d=f-cfg.finger[c].cen; long span=(d>0)?(cfg.finger[c].hi-cfg.finger[c].cen):(cfg.finger[c].cen-cfg.finger[c].lo); if(span<1)span=1;
      bool over=(labs(d)*100/span)>cfg.fingerThresh[c];   // pro Fingertip einzeln (25.08.2026, war geteilt)
      // vor/zurueck vertauscht (Tasten-Test-Protokoll 25.08.2026, alle 4 Fingertips gleichermassen
      // betroffen) -- reine Bedingungs-Vertauschung, betrifft nur den Tasten-Modus, nicht die Achse.
      setBtn(bn,BTN_FINGER+c*2+0,MAIN_BTNS, over&&d<0);   // vor
      setBtn(bn,BTN_FINGER+c*2+1,MAIN_BTNS, over&&d>0);   // zurück
      fa[c]=0; }
    else fa[c]=mapAxisCfg(f,cfg.finger[c],cfg.fingerDz[c],cfg.fingerSens[c]);   // pro Fingertip einzeln (25.08.2026, war FINGER_AXIS_DZ=6 fest + Empfindlichkeit immer 100%)
  }

  // ---- Feste Panels (Hauptbus) ----
  watchdog_hw->scratch[2]=5;
  muxSelect(-1);
  if(okSlot1) mapMcp(mcpRead(MCP_ES1),  mapSlot1,15,BTN_ES1,bn);
  if(okSlot2) mapMcp(mcpRead(MCP_ES2),  mapSlot2,15,BTN_ES2,bn);
  if(okBedien){
    uint16_t bedienGpio=mcpRead(MCP_BEDIEN);
    mapMcp(bedienGpio,mapBedien,10,BTN_BEDIEN+3,bn);   // 10 Gateron-Tasten -> 73..82 (waren 72..81)

    // CraftDeck-Schalter (D1=GPB2/Bit10, D2=GPB3/Bit11, aktiv-low wie die Tasten) -- frei
    // konfigurierbar, was dort angeschlossen ist (25.08.2026, Hendriks Wunsch: in der Ersteinrichtung
    // waehlbar). "Es soll auch mit nur einem angeschlossenen Eingang klappen" -- ist z.B. nur D1
    // verkabelt (D2 offen), bleibt alles, was von D2 abhaengt, einfach dauerhaft unbetaetigt, ganz von
    // selbst (offene Leitung = nicht aktiv, wie ueberall sonst in der Firmware), keine eigene
    // Fallunterscheidung noetig. D1 gewinnt, falls (elektrisch eigentlich unmoeglich) beide gleichzeitig
    // aktiv waeren.
    bool eingang1On=!((bedienGpio>>10)&1), eingang2On=!((bedienGpio>>11)&1);
    uint8_t zustand = eingang1On ? 1 : (eingang2On ? 2 : 0);   // 0=Nullstellung,1=Eingang1,2=Eingang2

    // Ist ControlHub verbunden, gilt IMMER Tastenmodus statt Ebenenschalter (Hendriks Wunsch: weniger
    // Komplexitaet/Stoerpotenzial -- ControlHub hat sein eigenes, viel maechtigeres Ebenen-System,
    // der Ebenenschalter-Modifier waere dort ohnehin nur totes Gewicht in der rohen Zustands-Uebertragung).
    uint8_t effMode = (cfg.bedienSwitchMode==0 && serverActive) ? 1 : cfg.bedienSwitchMode;

    if(effMode==0){
      // Ebenenschalter: D1 haelt Taste BTN_LAYER_MOD (127) dauerhaft, solange aktiv -- NUR relevant
      // im Server-Fallback (s. effMode oben), macht dort eine zweite Ebene ueber ein Game-Binding auf
      // Taste 127 zugaenglich.
      if(eingang1On) setBtn(bn,BTN_LAYER_MOD,MAIN_BTNS,true);
    } else if(effMode==1){
      // Tastenmodus: jeder Wechsel von Eingang 1 (D1, beide Richtungen) loest einen kurzen Impuls auf Taste 71 aus --
      // er verhaelt sich wie ein normaler (momentaner) Taster. Eingang 2 (D2) wirkt hier nicht (Hendrik 24.10.2026).
      uint8_t d1=eingang1On?1:0;
      if(d1!=ignitionPrevZustand){ ignitionPulseBtn=BTN_BEDIEN+1; ignitionPulseUntil=millis()+IGNITION_PULSE_MS; ignitionPrevZustand=d1; }
      if(millis()<ignitionPulseUntil) setBtn(bn,ignitionPulseBtn,MAIN_BTNS,true);
    } else {
      // Zuendschloss: 2 oder 3 Stellungen, je Stellung frei waehlbares Zielbutton (70/71/72) + gehalten
      // oder kurzer Impuls (0,7s). Bei 2 Stellungen wird Eingang2 (D2) schlicht ignoriert.
      uint8_t z = (cfg.ignitionPositions==2 && zustand==2) ? 0 : zustand;
      uint8_t targetBtn = BTN_BEDIEN + cfg.ignitionPosBtn[z];
      if(cfg.ignitionPosHold[z]){
        setBtn(bn,targetBtn,MAIN_BTNS,true);
        ignitionPrevZustand=z;
      } else {
        if(z!=ignitionPrevZustand){ ignitionPulseBtn=targetBtn; ignitionPulseUntil=millis()+IGNITION_POS_PULSE_MS; ignitionPrevZustand=z; }
        if(millis()<ignitionPulseUntil) setBtn(bn,ignitionPulseBtn,MAIN_BTNS,true);
      }
    }
  }

  // ---- "Taste lernen": frischen Druck einer physischen Taste melden ----
  // Alle MAIN_BTNS Tasten; die rawPrev-Kopie unten MUSS alle 16 Byte umfassen, sonst meldet LEARN fuer die
  // fehlenden Tasten bei jedem Durchlauf faelschlich einen Druck.
  if(learnMode){
    for(int g=1;g<=MAIN_BTNS;g++){ bool now=bn[(g-1)>>3]&(1<<((g-1)&7)); bool was=rawPrev[(g-1)>>3]&(1<<((g-1)&7));
      if(now&&!was){ Serial.print("LRN ");Serial.println(g); } }
    for(int i=0;i<16;i++) rawPrev[i]=bn[i];
  }

  // ---- Poti als Achse (kein Modus-Konzept mehr -- Shift/Ebenen übernimmt ControlHub) ----
  int poti=readPoti(); long pspan=max(1,cfg.potiHi-cfg.potiLo);
  long prel=poti-cfg.potiLo; if(cfg.potiInv) prel=pspan-prel;   // Poti-Rad spiegeln (Anschlag links<->rechts)
  long o=-(prel*65535L/pspan-32768L); if(o>32767)o=32767; if(o<-32767)o=-32767; int16_t potiAxis=(int16_t)o;

  // ---- Entprellung (Gerät 1) ----
  for(uint8_t i=0;i<16;i++){ uint8_t same=~(bn[i]^btnPrev[i]); rMain.btn[i]=(rMain.btn[i]&~same)|(bn[i]&same); btnPrev[i]=bn[i]; }   // 16 Byte = 128 Tasten

  // ---- Achsen ----
  rMain.x=slotAx[0][0]; rMain.y=slotAx[0][1]; rMain.z=slotAx[0][2]; rMain.rx=slotAx[0][3];     // Slot1
  rMain.ry=slotAx[1][0]; rMain.rz=slotAx[1][1]; rMain.slider=slotAx[1][2]; rMain.dial=slotAx[1][3]; // Slot2
  rSec.x=potiAxis; rSec.y=fa[0]; rSec.z=fa[1]; rSec.rx=fa[2]; rSec.ry=fa[3]; rSec.rz=0; rSec.slider=0; rSec.dial=0;

  watchdog_hw->scratch[2]=6;
  sendFrame(bn);   // rohen physischen Zustand an ControlHub streamen -- unabhaengig davon, ob gerade ein Server verbunden ist
  watchdog_hw->scratch[2]=7;

  // ---- Fallback-Zustandsmaschine: faellt automatisch auf die lokale 1:1-Belegung zurueck, wenn ControlHub verstummt ----
  if(serverActive && millis()-lastValidOut>200){ serverActive=false; goodOutStreak=0; }

  if(serverActive){
    // Server-Modus: HID-Report komplett aus dem OUT-Ergebnis uebernehmen (Vollzustand, keine eigene Entprellung noetig)
    memcpy(rMain.btn,outBtn,sizeof(rMain.btn));
    rMain.x=outAxis[0]; rMain.y=outAxis[1]; rMain.z=outAxis[2]; rMain.rx=outAxis[3];
    rMain.ry=outAxis[4]; rMain.rz=outAxis[5]; rMain.slider=outAxis[6]; rMain.dial=outAxis[7];
    rSec.x=outAxis[8]; rSec.y=outAxis[9]; rSec.z=outAxis[10]; rSec.rx=outAxis[11];
    rSec.ry=outAxis[12]; rSec.rz=outAxis[13]; rSec.slider=outAxis[14]; rSec.dial=outAxis[15];
    rKbd.modifier=0; memset(rKbd.keycode,0,sizeof(rKbd.keycode));
    uint8_t slot=0;
    for(uint8_t k=0;k<outKeyCount && slot<6;k++){
      uint8_t code=hidKeycodeFor(outKeys[k],&rKbd.modifier);
      if(code) rKbd.keycode[slot++]=code;
    }
  } else {
    // Fallback: reines Gamepad, ohne ControlHub keine Tastatur -- rMain/rSec sind bereits die lokal berechneten Werte von oben
    rKbd.modifier=0; memset(rKbd.keycode,0,sizeof(rKbd.keycode));
  }

  // Ursache der Reset-Serie gefunden: TinyUSB's eigene Race-Condition-Absicherung ("Attempt to
  // transfer on a busy endpoint") schlaegt zu, wenn sendReport() fuer ZWEI VERSCHIEDENE
  // HID-Interfaces kurz hintereinander in derselben Runde aufgerufen wird -- egal wie viel Pause
  // dazwischen liegt (2ms hat nicht gereicht, das Muster trat dann einfach beim naechsten
  // Geraete-Paar auf). Deshalb jetzt: pro Runde nur EIN sendReport()-Aufruf, reihum -- dann
  // treffen sich nie zwei verschiedene Geraete in derselben Runde. Jedes Geraet wird dadurch
  // seltener aktualisiert (bei ~2 Geraeten alle 2 Runden), das faellt bei HID-Eingaben nicht auf.
  static uint8_t hidTurn=0;
  watchdog_hw->scratch[2]=8;
  if(usbSperren()){   // s. usbSperren() -- ohne die Sperre blieb die Armlehne hier stehen (Watchdog-Neustart, Phase 8)
#if USB_SEC_ENABLED
  switch(hidTurn){
    case 0: if(usb_main.ready()) usb_main.sendReport(0,&rMain,sizeof(rMain)); break;
    case 1: if(usb_kbd.ready())  usb_kbd.sendReport(0,&rKbd,sizeof(rKbd));   break;
    case 2: if(usb_sec.ready())  usb_sec.sendReport(0,&rSec,sizeof(rSec));   break;
  }
  hidTurn=(hidTurn+1)%3;
#else
  switch(hidTurn){
    case 0: if(usb_main.ready()) usb_main.sendReport(0,&rMain,sizeof(rMain)); break;
    case 1: if(usb_kbd.ready())  usb_kbd.sendReport(0,&rKbd,sizeof(rKbd));   break;
  }
  hidTurn=(hidTurn+1)%2;
#endif
  usbFreigeben();
  }
#ifdef USE_TINYUSB
  TinyUSB_Device_Task();   // was der USB-Interrupt waehrend der Sperre ausgelassen hat, gleich hier nachholen
#endif
  watchdog_hw->scratch[2]=11;

  if(monEnabled && millis()-lastMon>150){ lastMon=millis(); Serial.print("Btn:");
    for(int g=1;g<=MAIN_BTNS;g++) if(rMain.btn[(g-1)>>3]&(1<<((g-1)&7))){Serial.print(' ');Serial.print(g);}
    Serial.print("  Ax1 x=");Serial.print(rMain.x);Serial.print(" y=");Serial.print(rMain.y);Serial.print(" Poti=");Serial.println(rSec.x); }
}

void loop(){
  rp2040.wdt_reset();   // Watchdog fuettern (laeuft die loop normal, kein Reboot)
  watchdog_hw->scratch[2]=1;
  unsigned long t0=micros();
  handleSerial();
  watchdog_hw->scratch[2]=2;
  unsigned long t1=micros();
  if(t1-t0>maxSerialUs) maxSerialUs=t1-t0;
  if(mode==MODE_CAL){ calLoop(); delay(8); return; }
  runController();
  unsigned long t2=micros();
  if(t2-t1>maxCtrlUs) maxCtrlUs=t2-t1;
  if(t2-t0>maxLoopUs) maxLoopUs=t2-t0;
  // Letzte Rundenzeit in ein Watchdog-Scratch-Register schreiben (0/1, dazu 2 = Phase; ungenutzt vom SDK -- 4..7
  // sind fuer watchdog_reboot()'s eigene Diagnose reserviert). Ueberlebt einen Reset (anders als
  // normale Variablen) -- damit sehen wir nach einem unerwarteten Neustart, wie lange die letzte
  // Runde VOR dem Reset gedauert hat, statt dass diese Information mit dem RAM verloren geht.
  watchdog_hw->scratch[0]=(uint32_t)(t2-t0);
  watchdog_hw->scratch[1]=SCR_MARKE;    // Marker: Werte in scratch[0]/[2]/[3] stammen von dieser Firmware
  watchdog_hw->scratch[3]=millis()/1000; // Laufzeit, fuer „lief … s vor dem Neustart“
  // Hinweis: aktive I2C-Recovery + Hintergrund-Neuerkennung wurden entfernt (auf dem realen Bus
  // zu aggressiv -> Flutung/Glitches/Lag). Echte Lockups fängt der Watchdog ab; Geräte-Erkennung
  // läuft beim Boot und über den Knopf „Neu prüfen" (STATUS).
  delay(5);
}
