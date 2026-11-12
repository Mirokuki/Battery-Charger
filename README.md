# 6-slot 1S / 2S USB-C LiPo charger

A USB-C powered charger with six independent slots. Every slot takes either a
1S pack (PicoBlade, e.g. Blade Nano 150 mAh) or a 2S pack (XT30, e.g. Mobula8
350 mAh), works out which one is plugged in, and charges it at the voltage and
rate you set on a small OLED with three buttons.

**Status: design files generated, nothing built or powered yet.** See
[Before you order](#before-you-order).

## How it works

```
USB-C ── AP33771C (asks the adapter for 9 V) ── VBUS ─┬─ AP63203 buck ── 3.3 V ── ESP32-C3, OLED, I2C mux
                                                     └─ 6 x BQ25798 buck-boost charger (one per slot)

slot:  BQ25798 BAT ──┬── P-FET pair ── PicoBlade (1S)      switch driven by the charger's D+ pin
                     └── P-FET pair ── XT30 (2S)           switch driven by the charger's D- pin
```

- **Any slot, either battery.** The BQ25798 is a buck-boost charger that is
  switched between 1S and 2S over I2C, so it also works from a plain 5 V port.
- **Detection.** With charging off, the firmware empties the charger's output
  node, closes one connector's switch and reads the pack voltage with the
  charger's own ADC, then does the same for the other connector.
  - PicoBlade 2.5–4.45 V → 1S, starts charging
  - XT30 5.0–8.9 V → 2S, starts charging
  - XT30 2.5–4.45 V → could be a 1S pack or a flat 2S: asks you to confirm
  - both connectors live → fault, nothing is charged
- **The two connectors of a slot can never be joined.** Each has its own
  back-to-back P-FET pair, off by default, and the firmware closes one at a time.
  A reverse-polarity pack cannot turn its switch on.
- **Fail-safe.** Charge-enable is pulled off in hardware, so a reset or crashed
  MCU stops all charging. The chargers' own over-voltage protection and 5 h
  safety timer stay active.
- **Power sharing.** The Samsung 25 W adapter gives about 20 W at 9 V. Six 2S
  350 mAh packs at 1C need about 18 W. If you ask for more than the adapter
  has, the firmware scales every slot down by the same factor, and each charger
  also backs off by itself if the input voltage sags.

### Limits you should know about

- **2S packs are not balanced.** You chose XT30 only, so the charger sees pack
  voltage, not cell voltage. Balance-check 2S packs occasionally on another
  charger. Adding a JST-XH balance port per slot is a board change.
- **Small packs stop a little early.** The charger chip's lowest termination
  current is 40 mA. For 150 mAh packs the firmware adds a 15 minute top-off,
  but expect roughly 95 % rather than a full 100 %.
- **LiPo vs LiHV cannot be detected**, so it is a per-slot setting (default
  4.20 V, which is safe for both).
- "Storage" only charges up to 3.85 V/cell. It cannot discharge a fuller pack.
- Charge rate is 50–1000 mA per slot in 10 mA steps. Current accuracy near the
  bottom of that range is modest (the chip is a 5 A design).

## Using it

| Button | Main screen | Slot menu |
|---|---|---|
| UP / DN | pick a slot | move, or change a value while editing |
| OK | open the slot menu | edit / confirm / run the item |

Slot menu: 1S voltage, 1S rate, 2S voltage, 2S rate, Start/Stop, Back. Settings
are remembered per slot. Defaults are 150 mA for 1S and 350 mA for 2S. The green
LED under each slot is on while that slot charges.

Hold **OK** while plugging in USB to put the ESP32 into download mode.

## Files

| Path | What |
|---|---|
| `hardware/design.py` | The circuit: every part, value, order code and net. Edit this, then regenerate. |
| `hardware/gen_schematic.py` | Writes the KiCad schematic (`lipo-charger.kicad_sch`, `slot1..6.kicad_sch`). |
| `hardware/gen_pcb.py` | Places and routes the 108 x 68 mm 4-layer board (`lipo-charger.kicad_pcb`). |
| `hardware/make_outputs.py` | Gerbers, drill, BOM, pick-and-place, schematic PDF, DRC/ERC reports. |
| `fab/` | What you upload to JLCPCB. |
| `firmware/lipo_charger/` | Arduino sketch for the ESP32-C3. |
| `case/make_case.py` | Generates `case_base.stl` and `case_lid.stl`. |
| `docs/` | Schematic PDF and board render. |

Regenerate everything:

```
python hardware/gen_schematic.py
"C:\Program Files\KiCad\10.0\bin\python.exe" hardware/gen_pcb.py
python hardware/make_outputs.py
python case/make_case.py
```

The schematic uses a net label on every pin instead of drawn wires. It is
generated from `design.py` and checked against it, so it is exact, but it reads
like a netlist rather than a hand-drawn schematic.

## Ordering the board (JLCPCB)

- Upload `fab/lipo-charger-gerbers.zip`: 4 layers, 1.6 mm, 1 oz.
- SMT assembly, top side: `fab/BOM_JLCPCB.csv` and `fab/CPL_JLCPCB.csv`.
- Rows in the BOM with an empty LCSC number need to be matched by MPN in
  JLCPCB's parts search. The LCSC numbers that are filled in were entered from
  memory and **must be checked** against the part description when you upload.
- Check part rotations in JLCPCB's placement preview, especially the BQ25798,
  the SOT-23 FETs, the LEDs and the ESP32 module.

Hand-soldered after delivery: 6 x XT30UPB-M, 6 x PicoBlade 53047-0210, the OLED
module on J2. J3 (UART) is optional.

The OLED is a 0.96" SSD1306 I2C module with pins in the order **GND VCC SCL
SDA**. Some modules swap GND and VCC: check yours against the silkscreen. Solder
it on a plain pin header (not a socket) so the glass sits about 5.5 mm above the
board, hanging towards the front edge.

Buttons need the tall version (PTS645, 9.5 mm) to reach through the lid.

## Firmware

Arduino IDE, board **ESP32C3 Dev Module**, *USB CDC On Boot: Enabled*.
Libraries: Adafruit GFX, Adafruit SSD1306. Flash over the USB-C port.

## Case

`case_base.stl` prints as is, `case_lid.stl` is already flipped to print
top-face down. No supports. Outside size 112.8 x 72.8 x 14.8 mm. Four
M2.5 x 14 self-tapping screws go through the lid posts and the board into the
base.

The opening sizes are parameters at the top of `case/make_case.py`. The OLED
window position (`OLED_WIN`) depends on the module, so measure yours first.

## Before you order

Nothing here has been built. Do these first:

1. **Open the board in KiCad and look it over.** It was placed and routed by
   script. Rule-check results are in the section below.
2. **PicoBlade polarity.** Pin 1 (the pad nearer the XT30) is positive, pin 2 is ground. Check
   with a meter that this matches your Blade packs. A reversed pack is simply
   not detected, so a wrong guess shows up as "nothing happens", not damage.
3. **The D+/D- switch drive is unusual.** The BQ25798 documents its D+/D-
   pins as host-programmable 0–3.3 V outputs (register 0x47), and this design
   uses them to drive the FET gate drivers. The datasheet does not state how
   much current they source; the load here is 49 µA. Worth proving on one slot
   before populating all six.
4. **First power-up without batteries**, on a current-limited supply if you
   have one. All six slots should report `--`. Then try one pack with a meter
   on it and compare voltage and current with the display.
5. **Compile the firmware.** It has not been compiled or run.
6. Never leave charging packs unattended, especially on an unproven charger.

## Rule-check results

From the last run on 2026-11-11 (KiCad 10.0, `hardware/make_outputs.py`):

| Check | Result |
|---|---|
| Schematic netlist vs `design.py` | 289 of 289 parts, 0 net mismatches |
| Board vs schematic (parity) | 0 issues |
| Board DRC errors | 0 |
| Unconnected nets | 0 |
| Board DRC warnings | 7: five unused escape vias on I2C lines, two silkscreen lines of the ESP32 module at the board edge |
| Schematic ERC | 49 "output pin connected to output pin" entries in which KiCad pairs a pin with itself, 3 "power pin not driven" (no power-flag symbols), 2 notes about the generated AP33771C symbol. None is a wiring fault. |
| Case meshes | both STLs closed (every edge shared by exactly two faces) |

Board rules used: 0.127 mm clearance, 0.15 mm minimum track, 0.6/0.3 mm vias,
0.2 mm minimum hole (the ESP32 footprint's thermal vias), 0.18 mm hole clearance
(the USB-C footprint's own locating holes sit 0.194 mm from its pads).

DRC proves the copper matches the netlist and meets spacing rules. It does not
prove the circuit works: see "Before you order".
