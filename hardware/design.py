"""
Single source of truth for the 6-slot 1S/2S USB-C LiPo charger.

Everything electrical lives here: parts, values, footprints, order codes and
the net each pin sits on.  gen_schematic.py and gen_pcb.py both read it, so the
schematic and the board cannot drift apart.

Architecture
  USB-C -> AP33771C PD sink (asks for 9 V) -> VBUS_PD rail
  VBUS_PD -> AP63203 buck -> +3V3 (ESP32-C3, OLED, I2C mux)
  VBUS_PD -> 6 x BQ25798 buck-boost charger (1S..2S, I2C)
  Each slot has two connectors (PicoBlade for 1S, XT30 for 2S).  Each connector
  reaches the charger's BAT node through its own back-to-back P-FET pair, so a
  1S and a 2S pack in the same slot can never be shorted together.  The pairs
  are switched by the charger's own D+ / D- output drivers (I2C register 0x47),
  and the pack voltage is read by the charger's ADC, so a slot needs only SDA
  and SCL from the MCU.
  2S packs also plug their balance lead into a JST-XH port with its own autonomous
  balancer (BQ29209), so balancing needs no MCU lines either.
"""

SLOTS = 6

PARTS = {}          # ref -> dict
NC_PINS = set()     # (ref, pin) intentionally unconnected


def part(ref, sym, fp, value, pins, mpn="", lcsc="", group="core", tht=False, dnp=False):
    assert ref not in PARTS, ref
    PARTS[ref] = dict(ref=ref, sym=sym, fp=fp, value=value, mpn=mpn, lcsc=lcsc,
                      group=group, tht=tht, dnp=dnp,
                      pins={str(k): v for k, v in pins.items() if v is not None})
    for k, v in pins.items():
        if v is None:
            NC_PINS.add((ref, str(k)))


R0402 = "Resistor_SMD:R_0402_1005Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
SOT23 = "Package_TO_SOT_SMD:SOT-23"

RES = {  # value -> (mpn, lcsc)
    "5.1k": ("0402WGF5101TCE", "C25905"),
    "82k": ("0402WGF8202TCE", "C25958"),
    "100k": ("0402WGF1003TCE", "C25741"),
    "2.2k": ("0402WGF2201TCE", "C25879"),
    "6.2k": ("0402WGF6201TCE", ""),
    "200R": ("0402WGF2000TCE", "C25087"),
    "10k": ("0402WGF1002TCE", "C25744"),
    "4.7k": ("0402WGF4701TCE", "C25900"),
    "1k": ("0402WGF1001TCE", "C11702"),
    "100R": ("0402WGF1000TCE", "C25076"),
    "47k": ("0402WGF4702TCE", "C25792"),
    "68k": ("0402WGF6802TCE", "C36871"),
    "120R": ("0402WGF1200TCE", ""),
}
CAPS = {  # value -> (footprint, mpn, lcsc)
    "10uF 25V": (C0805, "CL21A106KAYNNNE", "C15850"),
    "22uF 25V": (C0805, "CL21A226MAQNNNE", "C45783"),
    "4.7uF 16V": (C0603, "CL10A475KO8NNNC", "C19666"),
    "1uF 50V": (C0603, "CL10A105KB8NNNC", "C15849"),
    "1uF 16V": (C0402, "CL05A105KA5NQNC", "C52923"),
    "100nF 50V": (C0402, "CL05B104KB54PNC", "C307331"),
    "47nF 50V": (C0402, "CL05B473KB5NNNC", ""),
    "1nF 50V": (C0402, "CL05B102KB5NNNC", "C1523"),
}


def R(ref, value, a, b, group="core"):
    mpn, lcsc = RES[value]
    part(ref, "Device:R", R0402, value, {1: a, 2: b}, mpn, lcsc, group)


def C(ref, value, a, b, group="core"):
    fp, mpn, lcsc = CAPS[value]
    part(ref, "Device:C", fp, value, {1: a, 2: b}, mpn, lcsc, group)


GND, V33, VBUS = "GND", "+3V3", "VBUS_PD"

# ---------------------------------------------------------------- USB-C + PD
part("J1", "Connector:USB_C_Receptacle_USB2.0_16P",
     "Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal",
     "USB4105-GF-A",
     {"A1": GND, "A12": GND, "B1": GND, "B12": GND, "SH": GND,
      "A4": "VBUS_RAW", "A9": "VBUS_RAW", "B4": "VBUS_RAW", "B9": "VBUS_RAW",
      "A5": "USB_CC1", "B5": "USB_CC2",
      "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DN", "B7": "USB_DN",
      "A8": None, "B8": None},
     "USB4105-GF-A", "C3020560")
R("R1", "5.1k", "USB_CC1", GND)
R("R2", "5.1k", "USB_CC2", GND)
part("U1", "charger:AP33771C", "Package_DFN_QFN:DFN-14-1EP_3x3mm_P0.4mm_EP1.78x2.35mm",
     "AP33771C",
     {1: VBUS, 2: "PD_PWR_EN", 3: "VBUS_SNS", 4: "VBUS_RAW", 5: GND, 6: None,
      7: "PD_LED", 8: "PD_VSEL", 9: "PD_ISEL", 10: "USB_CC2", 11: "USB_CC1",
      12: None, 13: None, 14: "PD_V5V", 15: GND},
     "AP33771CFBZ-13-FA02", "")
NC_PINS.discard(("U1", "1"))
PARTS["U1"]["pins"]["1"] = "PD_VOUT"
part("R3", "Device:R", "Resistor_SMD:R_1206_3216Metric", "5mR 1%",
     {1: "VBUS_RAW", 2: "VBUS_SNS"}, "PE1206FKF470R005Z", "")
part("Q1", "Transistor_FET:AO3400A", SOT23, "AO3400A",
     {1: "PD_GATE", 2: VBUS, 3: "VBUS_SNS"}, "AO3400A", "C20917")
R("R4", "6.2k", "PD_PWR_EN", "PD_GATE")
C("C1", "100nF 50V", "PD_GATE", VBUS)
R("R5", "200R", "PD_VOUT", VBUS)
R("R6", "82k", "PD_VSEL", GND)          # 82k = request 9 V
R("R7", "100k", "PD_ISEL", GND)         # 100k = current "auto"
R("R8", "2.2k", "PD_LED", "PD_LED_A")
part("D1", "Device:LED", "LED_SMD:LED_0603_1608Metric", "PD OK (blue)",
     {1: GND, 2: "PD_LED_A"}, "19-217/BHC-ZL1M2RY/3T", "C72041")
C("C2", "1uF 50V", "VBUS_SNS", GND)
C("C3", "1uF 16V", "PD_V5V", GND)
C("C4", "10uF 25V", "VBUS_RAW", GND)
C("C5", "10uF 25V", VBUS, GND)
C("C6", "10uF 25V", VBUS, GND)
part("U4", "Power_Protection:USBLC6-2SC6", "Package_TO_SOT_SMD:SOT-23-6", "USBLC6-2SC6",
     {1: "USB_DN", 6: "USB_DN", 3: "USB_DP", 4: "USB_DP", 2: GND, 5: V33},
     "USBLC6-2SC6", "C7519")

# ---------------------------------------------------------------- 3.3 V buck
part("U2", "Regulator_Switching:AP63203WU", "Package_TO_SOT_SMD:TSOT-23-6", "AP63203WU",
     {1: V33, 2: VBUS, 3: VBUS, 4: GND, 5: "BUCK_SW", 6: "BUCK_BST"},
     "AP63203WU-7", "C780769")
C("C7", "10uF 25V", VBUS, GND)
C("C8", "100nF 50V", "BUCK_BST", "BUCK_SW")
part("L1", "Device:L", "Inductor_SMD:L_Changjiang_FNR4030S", "4.7uH 2A",
     {1: "BUCK_SW", 2: V33}, "FNR4030S4R7MT", "")
C("C9", "22uF 25V", V33, GND)
C("C10", "22uF 25V", V33, GND)

# ---------------------------------------------------------------- MCU
part("U3", "RF_Module:ESP32-C3-WROOM-02", "RF_Module:ESP32-C3-WROOM-02",
     "ESP32-C3-WROOM-02-N4",
     {1: V33, 2: "MCU_EN", 3: "I2C_SDA", 4: "I2C_SCL", 5: "MUX_RST_N", 6: None,
      7: "BTN_UP", 8: "BTN_OK", 9: GND, 10: "BTN_DN", 11: "UART_RX", 12: "UART_TX",
      13: "USB_DN", 14: "USB_DP", 15: "BUZZ", 16: "IO2_STRAP", 17: "CE_ALL_N",
      18: None, 19: GND},
     "ESP32-C3-WROOM-02-N4", "C2934560")
C("C11", "10uF 25V", V33, GND)
C("C12", "100nF 50V", V33, GND)
R("R9", "10k", V33, "MCU_EN")
C("C13", "1uF 16V", "MCU_EN", GND)
R("R10", "10k", V33, "IO2_STRAP")
R("R11", "10k", V33, "CE_ALL_N")        # chargers stay disabled while the MCU is in reset
R("R12", "4.7k", V33, "I2C_SDA")
R("R13", "4.7k", V33, "I2C_SCL")
part("J3", "Connector_Generic:Conn_01x03", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
     "UART (TX RX GND)", {1: "UART_TX", 2: "UART_RX", 3: GND}, "", "", tht=True, dnp=True)

# ---------------------------------------------------------------- UI
part("J2", "Connector_Generic:Conn_01x04", "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
     "OLED 0.96in SSD1306 (GND VCC SCL SDA)",
     {1: GND, 2: V33, 3: "I2C_SCL", 4: "I2C_SDA"}, "", "", tht=True)
for i, (ref, net) in enumerate((("SW1", "BTN_UP"), ("SW2", "BTN_OK"), ("SW3", "BTN_DN"))):
    part(ref, "Switch:SW_Push", "Button_Switch_SMD:SW_SPST_PTS645Sx43SMTR92", net,
         {1: net, 2: GND}, "PTS645SM95SMTR92 LFS", "")
    R(f"R{14 + i}", "10k", V33, net)
part("BZ1", "Device:Buzzer", "Buzzer_Beeper:MagneticBuzzer_CUI_CMT-8504-100-SMT", "CMT-8504-100-SMT",
     {1: V33, 2: "BUZZ_D", 3: None, 4: None}, "CMT-8504-100-SMT-TR", "")
part("D2", "Device:D", "Diode_SMD:D_SOD-323", "1N4148WS",
     {1: V33, 2: "BUZZ_D"}, "1N4148WS", "C2128")
part("Q2", "Transistor_FET:2N7002", SOT23, "2N7002", {1: "BUZZ", 2: GND, 3: "BUZZ_D"}, "2N7002", "C8545")
R("R17", "100k", "BUZZ", GND)

# ---------------------------------------------------------------- I2C mux
mux_pins = {1: GND, 2: GND, 21: GND, 3: "MUX_RST_N", 12: GND, 24: V33, 22: "I2C_SCL", 23: "I2C_SDA",
            17: None, 18: None, 19: None, 20: None}
for s in range(1, SLOTS + 1):
    mux_pins[4 + 2 * (s - 1) if s <= 4 else 13 + 2 * (s - 5)] = f"S{s}_SDA"
    mux_pins[5 + 2 * (s - 1) if s <= 4 else 14 + 2 * (s - 5)] = f"S{s}_SCL"
part("U5", "Interface_Expansion:TCA9548APWR", "Package_SO:TSSOP-24_4.4x7.8mm_P0.65mm", "TCA9548APWR",
     mux_pins, "TCA9548APWR", "C130026")
C("C14", "100nF 50V", V33, GND)
R("R18", "10k", V33, "MUX_RST_N")

# ---------------------------------------------------------------- charger slots
for s in range(1, SLOTS + 1):
    g = f"slot{s}"
    n = lambda name: f"S{s}_{name}"
    r = lambda kind, i: f"{kind}{s}{i:02d}"

    part(r("U", 1), "Battery_Management:BQ25798", "Package_DFN_QFN:Texas_RQM0029A_VQFN-29_4x4mm_P0.4mm",
         "BQ25798",
         {1: n("STAT"), 2: VBUS, 3: VBUS, 4: n("BTST1"), 5: n("REGN"), 6: n("EN_PB"), 7: n("EN_XT"),
          8: VBUS, 9: VBUS, 10: GND, 11: GND, 12: None, 13: "CE_ALL_N", 14: n("SCL"), 15: n("SDA"),
          16: n("TS"), 17: n("REGN"), 18: n("BATP"), 19: n("BTST2"), 20: n("PROG"), 21: None,
          22: n("BAT"), 23: n("BAT"), 24: n("SDRV"), 25: n("SYS"), 26: n("SW2"), 27: GND,
          28: n("SW1"), 29: n("PMID")},
         "BQ25798RQMR", "C2876768", g)
    part(r("L", 1), "Device:L", "Inductor_SMD:L_Changjiang_FNR4030S", "2.2uH 3A",
         {1: n("SW1"), 2: n("SW2")}, "FNR4030S2R2MT", "", g)
    C(r("C", 1), "47nF 50V", n("BTST1"), n("SW1"), g)
    C(r("C", 2), "4.7uF 16V", n("REGN"), GND, g)
    C(r("C", 3), "47nF 50V", n("BTST2"), n("SW2"), g)
    C(r("C", 4), "1nF 50V", n("SDRV"), GND, g)
    C(r("C", 5), "10uF 25V", VBUS, GND, g)
    C(r("C", 6), "100nF 50V", VBUS, GND, g)
    C(r("C", 7), "10uF 25V", n("PMID"), GND, g)
    C(r("C", 8), "10uF 25V", n("PMID"), GND, g)
    C(r("C", 9), "100nF 50V", n("PMID"), GND, g)
    C(r("C", 10), "22uF 25V", n("SYS"), GND, g)
    C(r("C", 11), "22uF 25V", n("SYS"), GND, g)
    C(r("C", 13), "100nF 50V", n("SYS"), GND, g)
    C(r("C", 14), "10uF 25V", n("BAT"), GND, g)
    C(r("C", 15), "10uF 25V", n("BAT"), GND, g)
    R(r("R", 1), "1k", n("REGN"), n("LED_A"), g)
    part(r("D", 1), "Device:LED", "LED_SMD:LED_0603_1608Metric", "CHARGING (green)",
         {1: n("STAT"), 2: n("LED_A")}, "19-217/GHC-YR1S2/3T", "C72043", g)
    R(r("R", 2), "10k", n("REGN"), n("TS"), g)      # TS held mid-window: no thermistor fitted
    R(r("R", 3), "10k", n("TS"), GND, g)
    R(r("R", 4), "100R", n("BATP"), n("BAT"), g)
    R(r("R", 5), "4.7k", n("PROG"), GND, g)         # 4.7k = 750 kHz, 1S power-up default

    # PicoBlade (1S) path: back-to-back P-FETs, common source
    part(r("J", 2), "Connector_Generic:Conn_01x02",
         "Connector_Molex:Molex_PicoBlade_53047-0210_1x02_P1.25mm_Vertical", "PicoBlade 1S",
         {1: n("PB_P"), 2: GND}, "0530470210", "", g, tht=True)
    part(r("Q", 1), "Transistor_FET:AO3401A", SOT23, "AO3401A", {1: n("PB_G"), 2: n("PB_S"), 3: n("PB_P")},
         "AO3401A", "C15127", g)
    part(r("Q", 2), "Transistor_FET:AO3401A", SOT23, "AO3401A", {1: n("PB_G"), 2: n("PB_S"), 3: n("BAT")},
         "AO3401A", "C15127", g)
    R(r("R", 6), "47k", n("PB_G"), n("PB_S"), g)
    C(r("C", 16), "100nF 50V", n("PB_G"), n("PB_S"), g)
    R(r("R", 7), "10k", n("PB_G"), n("PB_GD"), g)
    part(r("Q", 5), "Transistor_FET:2N7002", SOT23, "2N7002", {1: n("EN_PB"), 2: GND, 3: n("PB_GD")},
         "2N7002", "C8545", g)
    R(r("R", 8), "68k", n("EN_PB"), GND, g)

    # XT30 (2S) path
    part(r("J", 1), "Connector_Generic:Conn_01x02",
         "Connector_AMASS:AMASS_XT30UPB-M_1x02_P5.0mm_Vertical", "XT30 2S",
         {1: GND, 2: n("XT_P")}, "XT30UPB-M", "C99101", g, tht=True)
    part(r("Q", 3), "Transistor_FET:AO3401A", SOT23, "AO3401A", {1: n("XT_G"), 2: n("XT_S"), 3: n("XT_P")},
         "AO3401A", "C15127", g)
    part(r("Q", 4), "Transistor_FET:AO3401A", SOT23, "AO3401A", {1: n("XT_G"), 2: n("XT_S"), 3: n("BAT")},
         "AO3401A", "C15127", g)
    R(r("R", 9), "47k", n("XT_G"), n("XT_S"), g)
    C(r("C", 17), "100nF 50V", n("XT_G"), n("XT_S"), g)
    R(r("R", 10), "10k", n("XT_G"), n("XT_GD"), g)
    part(r("Q", 6), "Transistor_FET:2N7002", SOT23, "2N7002", {1: n("EN_XT"), 2: GND, 3: n("XT_GD")},
         "2N7002", "C8545", g)
    R(r("R", 11), "68k", n("EN_XT"), GND, g)

    # 2S balance port: BQ29209 bleeds the higher cell (about 10 mA through R14) until the two
    # cells match, and raises OUT if either cell passes 4.30 V.  OUT pulls the charger's TS pin
    # low, which suspends charging in hardware (the firmware only ignores TS in LiHV mode).
    part(r("J", 3), "Connector_Generic:Conn_01x03",
         "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", "XH balance 2S",
         {1: GND, 2: n("MID"), 3: n("BALP")}, "B3B-XH-A", "C144394", g, tht=True)
    part(r("U", 2), "charger:BQ29209", "Package_SON:VSON-8-1EP_3x3mm_P0.65mm_EP1.65x2.4mm", "BQ29209",
         {1: n("BALP"), 2: n("MID"), 3: n("CB"), 4: n("CD"), 5: GND, 6: GND, 7: n("BVDD"), 8: n("OV"), 9: GND},
         "BQ29209DRBR", "", g)
    R(r("R", 14), "120R", n("MID"), n("CB"), g)
    R(r("R", 15), "100R", n("BALP"), n("BVDD"), g)
    C(r("C", 18), "100nF 50V", n("BVDD"), GND, g)
    C(r("C", 19), "100nF 50V", n("BALP"), n("MID"), g)
    C(r("C", 20), "100nF 50V", n("MID"), GND, g)
    C(r("C", 21), "100nF 50V", n("CD"), GND, g)      # about 0.9 s over-voltage delay
    part(r("Q", 7), "Transistor_FET:2N7002", SOT23, "2N7002", {1: n("OV"), 2: GND, 3: n("TS")},
         "2N7002", "C8545", g)
    R(r("R", 16), "100k", n("OV"), GND, g)

    # I2C pull-ups for this mux channel (placed beside the mux)
    R(r("R", 12), "10k", V33, n("SDA"), g)
    R(r("R", 13), "10k", V33, n("SCL"), g)


def nets():
    """net name -> sorted list of (ref, pin)."""
    out = {}
    for ref, p in PARTS.items():
        for pin, net in p["pins"].items():
            out.setdefault(net, []).append((ref, pin))
    return {k: sorted(v) for k, v in sorted(out.items())}


if __name__ == "__main__":
    ns = nets()
    print(len(PARTS), "parts,", len(ns), "nets")
    for name, nodes in ns.items():
        if len(nodes) < 2:
            print("SINGLE-PIN NET:", name, nodes)
