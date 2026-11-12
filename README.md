# 6-slot 1S / 2S USB-C LiPo balance charger

A USB-C powered charger with six independent slots. Every slot takes either a
1S pack (PicoBlade, e.g. Blade Nano 150 mAh) or a 2S pack (XT30 plus its JST-XH
balance lead, e.g. Mobula8 350 mAh). It works out which one is plugged in and
charges it at the voltage and rate you set on a small OLED with three buttons.
2S packs are balanced while they charge.

**Status: design files generated, nothing built or powered yet.** See
[Before you order](#before-you-order).

## How it works

```
USB-C ── AP33771C (asks the adapter for 9 V) ── VBUS ─┬─ AP63203 buck ── 3.3 V ── ESP32-C3, OLED, I2C mux
                                                     └─ 6 x BQ25798 buck-boost charger (one per slot)

slot:  BQ25798 BAT ──┬── P-FET pair ── PicoBlade (1S)      switch driven by the charger's D+ pin
                     └── P-FET pair ── XT30 (2S)           switch driven by the charger's D- pin
       JST-XH balance port ── BQ29209 balancer ── over-voltage flag ── charger TS pin
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
- **The two power connectors of a slot can never be joined.** Each has its own
  back-to-back P-FET pair, off by default, and the firmware closes one at a time.
  A reverse-polarity pack cannot turn its switch on.
- **Balancing.** Each slot's JST-XH port feeds a BQ29209, which compares the
  two cells and bleeds the higher one (about 10 mA) until they match. It starts
  when the cells differ by about 30 mV and needs no firmware. It keeps working
  after the charge finishes for as long as the balance lead is plugged in.
- **Per-cell over-voltage cut-off.** If either cell passes 4.30 V the balancer
  pulls the charger's TS pin low, which stops charging in hardware; the display
  shows `cell>4.3`. The cut-off is ignored in LiHV mode, where 4.35 V per cell
  is the target.
- **Fail-safe.** Charge-enable is pulled off in hardware, so a reset or crashed
  MCU stops all charging. The chargers' own over-voltage protection and 5 h
  safety timer stay active.
- **Power sharing.** The Samsung 25 W adapter gives about 20 W at 9 V. Six 2S
  350 mAh packs at 1C need about 18 W. If you ask for more than the adapter
  has, the firmware scales every slot down by the same factor, and each charger
  also backs off by itself if the input voltage sags.

### Limits you should know about

- **Balancing is gentle and invisible to the firmware.** At about 10 mA it
  corrects roughly 10 mAh per hour, which suits packs that are slightly out,
  not badly unbalanced ones. The display shows pack voltage only, not cell
  voltages, and nothing detects a forgotten balance lead: a 2S pack on the
  XT30 alone still charges, just unbalanced and without the per-cell cut-off.
- **Small packs stop a little early.** The charger chip's lowest termination
  current is 40 mA. For 150 mAh packs the firmware adds a 15 minute top-off,
  but expect roughly 95 % rather than a full 100 %.
- **LiPo vs LiHV cannot be detected**, so it is a per-slot setting (default
  4.20 V, which is safe for both).
- "Storage" only charges up to 3.85 V/cell. It cannot discharge a fuller pack.
- Charge rate is 50–1000 mA per slot in 10 mA steps. Current accuracy near the
  bottom of that range is modest (the chip is a 5 A design).

## Using it

2S: plug in the balance lead and the XT30. 1S: plug into the PicoBlade.

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
| `hardware/gen_pcb.py` | Places and routes the 108 x 80 mm 4-layer board (`lipo-charger.kicad_pcb`). |
| `hardware/make_outputs.py` | Gerbers, drill, BOM, pick-and-place, schematic PDF, DRC/ERC reports. |
| `hardware/render_copper.py` | Quick picture of the routed copper (`docs/copper.png`). |
| `fab/` | What you upload to JLCPCB. |
| `firmware/lipo_charger/` | Arduino sketch for the ESP32-C3. |
| `case/make_case.py` | Generates `case_base.stl` and `case_lid.stl`. |
| `docs/` | Schematic PDF and board renders. |

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
- SMT assembly, top side only: `fab/BOM_JLCPCB.csv` and `fab/CPL_JLCPCB.csv`.
  Every surface-mount part is on the top; the bottom carries only copper.
- Rows in the BOM with an empty LCSC number need to be matched by MPN in
  JLCPCB's parts search. The LCSC numbers that are filled in were entered from
  memory and **must be checked** against the part description when you upload.
- Check part rotations in JLCPCB's placement preview, especially the BQ25798,
  the BQ29209, the SOT-23 FETs, the LEDs and the ESP32 module.

Hand-soldered after delivery: 6 x XT30UPB-M, 6 x JST B3B-XH-A, 6 x PicoBlade
53047-0210, the OLED module on J2. J3 (UART) is optional.

The OLED is a 0.96" SSD1306 I2C module with pins in the order **GND VCC SCL
SDA**. Some modules swap GND and VCC: check yours against the silkscreen. Solder
it on a plain pin header (not a socket) so the glass sits about 5.5 mm above the
board, hanging towards the front edge.

Buttons need the tall version (PTS645, 9.5 mm) to reach through the lid.

## Firmware

Arduino IDE, board **ESP32C3 Dev Module**, *USB CDC On Boot: Enabled*.
Libraries: Adafruit GFX, Adafruit SSD1306. Flash over the USB-C port.

## Case

Outside size 112.8 x 84.8 x 14.8 mm. `case_base.stl` prints as is,
`case_lid.stl` is already flipped to print top-face down. No supports.

Hardware: 4 x M3 heat-set insert (5.0 mm knurl, 4.2 mm pilot, 4.0 mm long) and
4 x M3 x 8 socket-head screw.

1. Melt an insert into the open end of each of the four lid posts, flush with
   the end of the post.
2. Drop the board into the base. It rests on the four bosses and seven pegs.
3. Fit the lid. The posts land on the board around its M3 holes.
4. Screw in from underneath. The heads sit in 6.2 x 3.2 mm recesses, so the
   case sits flat, and the top of the lid has no screw holes.

The opening sizes, insert hole (`INSERT_HOLE`, 4.2 mm) and screw recess are
parameters at the top of `case/make_case.py`. Print one post as a test before
committing to the insert hole size for your filament. The OLED window position
(`OLED_WIN`) depends on the module, so measure yours first.

## Before you order

Nothing here has been built. Do these first:

1. **Open the board in KiCad and look it over.** It was placed and routed by
   script. Rule-check results are in the section below.
2. **PicoBlade polarity.** Pin 1 (the pad nearer the balance port) is positive,
   pin 2 is ground. Check with a meter that this matches your Blade packs. A
   reversed pack is simply not detected, so a wrong guess shows up as "nothing
   happens", not damage.
3. **Balance port pinout.** Pin 1 (square pad, at the left) is pack negative,
   pin 2 the cell junction, pin 3 pack positive, which is the usual JST-XH
   balance wiring. Check one of your packs with a meter.
4. **The D+/D- switch drive is unusual.** The BQ25798 documents its D+/D-
   pins as host-programmable 0–3.3 V outputs (register 0x47), and this design
   uses them to drive the FET gate drivers. The datasheet does not state how
   much current they source; the load here is 49 µA. Worth proving on one slot
   before populating all six.
5. **Order the BQ29209, not the BQ29200.** The two share a datasheet; the
   BQ29209 trips at 4.30 V per cell, the BQ29200 at 4.35 V.
6. **First power-up without batteries**, on a current-limited supply if you
   have one. All six slots should report `--`. Then try one pack with a meter
   on it and compare voltage and current with the display.
7. **Compile the firmware.** It has not been compiled or run.
8. Never leave charging packs unattended, especially on an unproven charger.

## Rule-check results

From the last run on 2026-11-11 (KiCad 10.0, `hardware/make_outputs.py`):

| Check | Result |
|---|---|
| Schematic netlist vs `design.py` | 349 of 349 parts, 0 net mismatches |
| Board vs schematic (parity) | 0 issues |
| Board DRC errors | 0 |
| Unconnected nets | 0 |
| Board DRC warnings | 7: five unused escape vias on I2C lines, two silkscreen lines of the ESP32 module at the board edge |
| Schematic ERC | "output pin connected to output pin" entries in which KiCad pairs a pin with itself, "power pin not driven" (no power-flag symbols), and notes about the two generated symbols. None is a wiring fault. |
| Case meshes | both STLs closed (every edge shared by exactly two faces) |

Board rules used: 0.127 mm clearance, 0.15 mm minimum track, 0.6/0.3 mm vias,
0.2 mm minimum hole (the ESP32 footprint's thermal vias), 0.18 mm hole clearance
(the USB-C footprint's own locating holes sit 0.194 mm from its pads).

DRC proves the copper matches the netlist and meets spacing rules. It does not
prove the circuit works: see "Before you order".
