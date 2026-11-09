# 6-Slot 1S LiPo / LiHV Smart USB-C PD Charger

A 6-slot intelligent 1S LiPo / LiHV battery charger powered via USB-C Power Delivery (PD) and controlled by an ESP32-C3 microcontroller.

---

## Features

- **6 Independent Charging Channels**:
  - Powered by 6x TI BQ25188 I2C linear charger ICs.
  - Per-slot programmable charging current (50 mA to 500 mA in 10 mA steps).
  - Per-slot selectable termination voltage:
    - **LiPo**: 4.20 V
    - **LiHV**: 4.35 V
    - **Storage**: 3.80 V
- **USB-C Power Delivery**:
  - Diodes Inc. AP33771C USB-PD sink controller negotiates optimal PD profiles.
  - Reversible USB-C input with native USB Serial/JTAG for programming and telemetry.
- **Microcontroller & Telemetry**:
  - **ESP32-C3-WROOM-02-N4** (RISC-V single-core @ 160 MHz, 4 MB Flash).
  - Integrated BLE GATT Server for remote monitoring and parameter configuration.
  - TI TCA9548A I2C switch for multiplexing communication across all 6 chargers.
  - CD74HC4051 ADC multiplexer with 16x oversampling & rolling median filter for battery detection and voltage sensing.
- **User Interface**:
  - 0.96" 128x64 SSD1306 OLED display for per-slot status, voltages, and currents.
  - AW9523B constant-current LED driver controlling bi-color status LEDs for each slot.
  - Tactile UP / DOWN buttons for manual slot selection and configuration.

---

## Repository Structure

```
├── kicad/                        # KiCad 8 Schematic, PCB, and Footprints
│   ├── lipo-6slot-charger.kicad_sch
│   ├── lipo-6slot-charger.kicad_pcb
│   ├── charger_slot.kicad_sch    # Hierarchical sheet for charger channels
│   ├── lipo_charger_custom.kicad_sym
│   ├── lipo_charger_custom.pretty/
│   └── 3dmodels/
├── firmware/                     # Firmware source code
│   └── lipo_6slot_charger/
│       └── lipo_6slot_charger.ino
├── fab_4layer_gerbers/           # Production-ready Gerber and drill files (4-Layer)
├── fab_4layer_gerbers.zip        # Pre-packaged archive for PCB fab (JLCPCB / PCBWay)
├── BOM_JLCPCB.csv                # Bill of Materials formatted for JLCPCB SMT assembly
├── BOM_DigiKey.csv               # Bill of Materials formatted for DigiKey
└── BOM_Mouser.csv                # Bill of Materials formatted for Mouser
```

---

## Hardware Specifications

- **Layer Count**: 4-Layer PCB (Signal / GND / PWR / Signal)
- **Dimensions**: Optimized for 1S battery charging bays
- **Input Power**: USB Type-C (PD 5V - 12V negotiation via AP33771C)
- **Supported Chemistry**: 1S LiPo (3.7 V / 4.2 V) and 1S LiHV (3.8 V / 4.35 V)

---

## Firmware Setup

1. Open `firmware/lipo_6slot_charger/lipo_6slot_charger.ino` in Arduino IDE or PlatformIO.
2. Ensure the **ESP32 by Espressif Systems** board package is installed.
3. Configure the board:
   - **Board**: `ESP32C3 Dev Module`
   - **USB CDC On Boot**: `Enabled`
   - **Flash Mode**: `QIO 80MHz`
   - **Flash Size**: `4MB (32Mb)`
   - **Partition Scheme**: `Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)`
4. Required Libraries:
   - `Adafruit GFX Library`
   - `Adafruit SSD1306`
5. Connect via USB-C and flash the board.
