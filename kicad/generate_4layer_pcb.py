#!/usr/bin/env python3
"""
4-Layer PCB Generator & Router for the 6-Slot 1S LiPo / LiHV Smart USB-C PD Charger.
All SMT components on TOP layer (F_Cu).
Stackup:
  - Layer 1 (F_Cu): Signal & Component Pads, Power bus, Solid Top Ground Pour
  - Layer 2 (In1_Cu): Solid GND Plane
  - Layer 3 (In2_Cu): Split Power Plane: +3V3 (top section), VIN_4V8 (bottom section)
  - Layer 4 (B_Cu): Signal Lanes, Solid Bottom Ground Pour
Board Dimensions: 75.0mm x 80.0mm
"""

import os
import math
import heapq
import xml.etree.ElementTree as ET
import pcbnew

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
FP_STD_ROOT = r"C:\Program Files\KiCad\10.0\share\kicad\footprints"
FP_CUSTOM_ROOT = os.path.join(PROJECT_DIR, "lipo_charger_custom.pretty")

# Board origin in KiCad sheet coordinates (mm). All layout below is in board-local mm.
X0 = 50.0
Y0 = 40.0
BOARD_W = 75.0
BOARD_H = 80.0

CLEARANCE = 0.16       # Router design clearance, slightly above the 0.15mm board rule
VIA_D, VIA_DRILL = 0.55, 0.30
POWER_VIA_D, POWER_VIA_DRILL = 0.70, 0.35

CELL_X_LEFT, CELL_X_RIGHT = 31.0, 44.0
SPINE_X = 37.5
ROW_Y = [49.0, 60.0, 71.0]

LEFT_LED_X, LEFT_PB_X = 13.5, 20.0
RIGHT_PB_X, RIGHT_LED_X = 55.0, 61.5

POWER_ZONES = [
    ("/VBUS_9V", 2.0, 24.8, 15.1, 30.9, 2),
    ("/BUCK_SW", 10.2, 31.4, 14.3, 35.5, 2),
    ("/VIN_4V8", 3.9, 31.4, 7.8, 41.8, 2),
]

BUCK_KEEPOUT = (1.0, 24.6, 25.5, 38.4)
ANTENNA_KEEPOUT = (49.8, 7.6)
PLANE_SPLIT = (40.2, 40.8)

TRUNK_Y = 41.5
TRUNK_X = (5.0, 60.0)
RAIL_TOP, RAIL_BOTTOM = 43.7, 75.0
SPINE_BOTTOM = 75.5
RAIL_EXIT = (38.75, 39.5)

# Bottom-side lanes: (SDA, SCL, VSENSE) x coordinates
# Dedicated distinct non-overlapping corridors (>= 0.35mm pitch, clear of H5..H8 and each other)
LANES = {
    # Left bank: (SDA, SCL, VSENSE) 0.50mm pitch strictly monotonic planar unrolling
    1: (35.80, 35.30, 32.80),
    2: (34.80, 34.30, 31.80),
    3: (33.80, 33.30, 32.30),
    # Right bank: (SDA, SCL, VSENSE) 0.50mm pitch strictly monotonic planar unrolling
    4: (40.70, 41.20, 39.20),
    5: (41.70, 42.20, 39.70),
    6: (42.70, 43.20, 40.20),
}

LANE_EXIT_Y = 44.0


def get_placement():
    """Return dictionary mapping Ref -> (x_mm, y_mm, rot_deg) in board-local mm."""
    pos = {}

    # --- USB-C input and AP33771C ---
    pos["J1"] = (14.0, 3.12, 180.0)
    pos["U1"] = (14.0, 13.6, 0.0)
    pos["C12"] = (6.0, 8.6, 180.0)
    pos["R13"] = (9.0, 14.0, 270.0)
    pos["C1"] = (10.9, 16.4, 270.0)
    pos["C2"] = (16.9, 11.2, 0.0)
    pos["R11"] = (20.5, 13.0, 0.0)
    pos["R12"] = (20.5, 15.0, 0.0)
    pos["R2"] = (20.5, 17.0, 0.0)
    pos["R1"] = (20.5, 19.0, 0.0)
    pos["R16"] = (13.5, 17.0, 0.0)
    pos["D9"] = (16.5, 17.0, 180.0)

    # --- VBUS switch ---
    pos["Q1"] = (11.5, 21.5, 0.0)
    pos["R14"] = (3.6, 16.2, 90.0)
    pos["R15"] = (5.8, 16.2, 270.0)
    pos["C11"] = (3.6, 21.0, 270.0)

    # --- Buck ---
    pos["U2"] = (16.0, 32.0, 0.0)
    pos["C3"] = (15.05, 29.5, 0.0)
    pos["C4"] = (15.05, 26.6, 0.0)
    pos["R5"] = (11.5, 30.2, 0.0)
    pos["L1"] = (9.0, 34.5, 180.0)
    pos["C6"] = (3.6, 32.4, 180.0)
    pos["C7"] = (3.6, 35.0, 180.0)
    pos["C17"] = (3.6, 37.6, 180.0)
    pos["R4"] = (19.6, 36.2, 270.0)
    pos["R3"] = (21.6, 36.2, 90.0)
    pos["C5"] = (23.6, 36.2, 90.0)

    # --- 3.3V LDO ---
    pos["U3"] = (28.5, 37.2, 0.0)
    pos["C8"] = (26.8, 34.0, 0.0)
    pos["C9"] = (29.6, 32.5, 0.0)

    # --- OLED header and muxes ---
    pos["J2"] = (39.61, 6.0, 270.0)
    pos["U5"] = (36.0, 14.0, 0.0)
    pos["C15"] = (43.0, 6.9, 0.0)
    pos["U6"] = (36.0, 26.5, 0.0)
    pos["C19"] = (41.6, 21.2, 0.0)
    pos["R7"] = (44.6, 9.6, 180.0)
    pos["R8"] = (44.6, 11.4, 180.0)

    # --- ESP32 at top-right ---
    pos["U4"] = (64.0, 14.5, 0.0)
    pos["C14"] = (52.7, 9.2, 90.0)
    pos["C13"] = (50.9, 9.2, 90.0)
    # R6 and C10 directly beside U4 pin 2 (MCU_EN at 55.25, 10.0)
    pos["R6"] = (52.7, 12.0, 0.0)
    pos["C10"] = (52.7, 14.0, 0.0)
    pos["U7"] = (54.5, 30.5, 0.0)
    pos["C16"] = (52.0, 26.2, 270.0)
    pos["C18"] = (19.0, 39.5, 90.0)
    # J3 and J4 headers below U4 with clean 4mm clearance
    pos["J3"] = (61.0, 26.5, 0.0)
    pos["J4"] = (66.0, 26.5, 0.0)

    # --- 3 Buttons & dedicated pullup resistors directly above each button ---
    pos["SW1"] = (47.0, 38.85, 0.0)
    pos["R9"] = (47.0, 34.5, 0.0)
    pos["SW2"] = (57.5, 38.85, 0.0)
    pos["R18"] = (57.5, 34.5, 0.0)
    pos["SW3"] = (68.0, 38.85, 0.0)
    pos["R10"] = (68.0, 34.5, 0.0)

    # --- 6 Charger Slots ---
    for slot in range(1, 7):
        if slot <= 3:
            row = slot - 1
            y = ROW_Y[row]
            pos[f"U10{slot}"] = (CELL_X_LEFT, y, 180.0)
            # Power passives west (horizontal 0603, uniform pitch 1.6mm in Y, zero courtyard overlap)
            pos[f"C10{slot}"] = (CELL_X_LEFT - 2.7, y + 3.2, 180.0)
            pos[f"C30{slot}"] = (CELL_X_LEFT - 2.7, y + 1.6, 180.0)
            pos[f"C20{slot}"] = (CELL_X_LEFT - 2.7, y - 0.0, 180.0)
            pos[f"R20{slot}"] = (CELL_X_LEFT - 2.7, y - 1.6, 0.0)
            pos[f"R30{slot}"] = (CELL_X_LEFT - 2.7, y - 3.2, 180.0)
            # Signal passives east
            pos[f"R40{slot}"] = (CELL_X_LEFT + 2.7, y + 1.6, 180.0)
            pos[f"R50{slot}"] = (CELL_X_LEFT + 2.7, y + 0.0, 180.0)
            pos[f"R10{slot}"] = (CELL_X_LEFT + 2.7, y - 1.6, 0.0)
            # Connectors and LED
            pos[f"J10{slot}"] = (LEFT_PB_X, y, 0.0)
            pos[f"D{slot}"] = (LEFT_LED_X, y, 180.0)
        else:
            row = slot - 4
            y = ROW_Y[row]
            pos[f"U10{slot}"] = (CELL_X_RIGHT, y, 0.0)
            # Power passives east (horizontal 0603, uniform pitch 1.6mm in Y, zero courtyard overlap)
            pos[f"C10{slot}"] = (CELL_X_RIGHT + 2.7, y - 3.2, 0.0)
            pos[f"C30{slot}"] = (CELL_X_RIGHT + 2.7, y - 1.6, 0.0)
            pos[f"C20{slot}"] = (CELL_X_RIGHT + 2.7, y + 0.0, 0.0)
            pos[f"R20{slot}"] = (CELL_X_RIGHT + 2.7, y + 1.6, 180.0)
            pos[f"R30{slot}"] = (CELL_X_RIGHT + 2.7, y + 3.2, 0.0)
            # Signal passives west
            pos[f"R40{slot}"] = (CELL_X_RIGHT - 2.7, y - 1.6, 0.0)
            pos[f"R50{slot}"] = (CELL_X_RIGHT - 2.7, y - 0.0, 0.0)
            pos[f"R10{slot}"] = (CELL_X_RIGHT - 2.7, y + 1.6, 180.0)
            # Connectors and LED
            pos[f"J10{slot}"] = (RIGHT_PB_X, y, 0.0)
            pos[f"D{slot}"] = (RIGHT_LED_X, y, 0.0)
    return {ref: (x + X0, y + Y0, rot) for ref, (x, y, rot) in pos.items()}

# ==============================================================================
# Low-level copper helpers. Coordinates are board-local mm.
# ==============================================================================
class Copper:
    def __init__(self, board, net_map, fp_objs):
        self.board = board
        self.nets = net_map
        self.fps = fp_objs
        self.pending = []

    def flush(self):
        items, self.pending = self.pending, []
        return items

    def pad(self, ref, number):
        """Board-local position of a footprint pad."""
        for p in self.fps[ref].Pads():
            if p.GetNumber() == str(number):
                pos = p.GetPosition()
                return (pcbnew.ToMM(pos.x) - X0, pcbnew.ToMM(pos.y) - Y0)
        raise KeyError(f"{ref}.{number}")

    def track(self, net, pts, width=0.2, layer=pcbnew.F_Cu):
        ni = self.nets[net] if isinstance(net, str) else net
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            if abs(x1 - x2) < 1e-4 and abs(y1 - y2) < 1e-4:
                continue
            t = pcbnew.PCB_TRACK(self.board)
            t.SetStart(pcbnew.VECTOR2I_MM(round(x1 + X0, 4), round(y1 + Y0, 4)))
            t.SetEnd(pcbnew.VECTOR2I_MM(round(x2 + X0, 4), round(y2 + Y0, 4)))
            t.SetWidth(pcbnew.FromMM(width))
            t.SetLayer(layer)
            t.SetNet(ni)
            self.board.Add(t)
            self.pending.append(t)

    def via(self, net, x, y, size=VIA_D, drill=VIA_DRILL):
        ni = self.nets[net] if isinstance(net, str) else net
        v = pcbnew.PCB_VIA(self.board)
        v.SetPosition(pcbnew.VECTOR2I_MM(round(x + X0, 4), round(y + Y0, 4)))
        v.SetWidth(pcbnew.FromMM(size))
        v.SetDrill(pcbnew.FromMM(drill))
        v.SetNet(ni)
        self.board.Add(v)
        self.pending.append(v)


def route_power(cu):
    """USB input, VBUS switch, buck and the main trunk."""
    pad = cu.pad

    # --- VBUS_RAW: both USB-C VBUS pairs join on the left, then feed R13 and C12 ---
    a4 = pad("J1", "A4")
    b4 = pad("J1", "B4")
    left, right = (a4, b4) if a4[0] < b4[0] else (b4, a4)
    cu.track("/VBUS_RAW", [left, (left[0], 8.6), (8.0, 8.6)], 0.6)
    cu.track("/VBUS_RAW", [right, (right[0], 7.9)], 0.5)
    cu.via("/VBUS_RAW", right[0], 7.9, POWER_VIA_D, POWER_VIA_DRILL)
    cu.via("/VBUS_RAW", left[0], 7.9, POWER_VIA_D, POWER_VIA_DRILL)
    cu.track("/VBUS_RAW", [(right[0], 7.9), (left[0], 7.9)], 0.5, pcbnew.B_Cu)
    r13_raw = pad("R13", "1")
    r13_sns = pad("R13", "2")
    c12 = pad("C12", "1")
    cu.track("/VBUS_RAW", [c12, (r13_raw[0], c12[1]), r13_raw], 0.8)

    # --- VBUS_SNS: through R13 to the MOSFET drain tab ---
    q_d = pad("Q1", "2")
    cu.track("/VBUS_SNS", [r13_sns, (r13_sns[0], q_d[1] - 2.0), (q_d[0], q_d[1] - 2.0)], 1.0)

    # --- VBUS_9V: MOSFET source into the buck input copper ---
    q_s = pad("Q1", "3")
    cu.track("/VBUS_9V", [q_s, (q_s[0], 25.2)], 1.0)

    # --- AP33771C Kelvin sensing ---
    vcc = pad("U1", "3")
    cu.track("/VBUS_SNS", [vcc, (10.9, vcc[1]), (10.9, r13_sns[1])], 0.2)
    cu.track("/VBUS_SNS", [(10.9, r13_sns[1]), r13_sns], 0.25)
    cu.track("/VBUS_SNS", [(10.9, r13_sns[1]), pad("C1", "1")], 0.25)
    c1_gnd = pad("C1", "2")
    cu.track("/GND", [c1_gnd, (10.0, c1_gnd[1])], 0.25)
    cu.via("/GND", 10.0, c1_gnd[1])

    isenp = pad("U1", "4")
    cu.track("/VBUS_RAW", [isenp, (11.9, isenp[1]), (11.55, 14.0)], 0.18)
    cu.via("/VBUS_RAW", 11.55, 14.0)
    cu.track("/VBUS_RAW", [(11.55, 14.0), (10.4, 14.0), (10.4, 11.8)], 0.2, pcbnew.B_Cu)
    cu.via("/VBUS_RAW", 10.4, 11.8)
    cu.track("/VBUS_RAW", [(10.4, 11.8), r13_raw], 0.2)

    vout = pad("U1", "1")
    cu.track("/PD_VOUT", [vout, (11.9, 12.3), (11.6, 12.0)], 0.16)
    cu.via("/PD_VOUT", 11.6, 12.0)
    pwr_en = pad("U1", "2")
    cu.track("/PD_PWR_EN", [pwr_en, (11.15, 12.6)], 0.16)
    cu.via("/PD_PWR_EN", 11.15, 12.6)

    r14_vout = pad("R14", "2")
    cu.track("/PD_VOUT", [(11.6, 12.0), (11.6, 10.2), (r14_vout[0], 10.2), (r14_vout[0], 14.3)], 0.2, pcbnew.B_Cu)
    cu.via("/PD_VOUT", r14_vout[0], 14.3)
    cu.track("/PD_VOUT", [(r14_vout[0], 14.3), r14_vout], 0.2)
    r15_en = pad("R15", "1")
    cu.track("/PD_PWR_EN", [(11.15, 12.6), (11.0, 11.6), (11.0, 10.9), (r15_en[0], 10.9), (r15_en[0], 14.3)], 0.2, pcbnew.B_Cu)
    cu.via("/PD_PWR_EN", r15_en[0], 14.3)
    cu.track("/PD_PWR_EN", [(r15_en[0], 14.3), r15_en], 0.2)

    # Gate network
    q_g = pad("Q1", "1")
    r15_gate = pad("R15", "2")
    c11_gate = pad("C11", "1")
    c11_src = pad("C11", "2")
    cu.track("/PD_GATE", [r15_gate, (r15_gate[0], 18.4), q_g], 0.25)
    cu.track("/PD_GATE", [(r15_gate[0], 18.4), (c11_gate[0], 18.4), c11_gate], 0.25)
    cu.track("/VBUS_9V", [c11_src, (c11_src[0], 25.2)], 0.4)
    r14_9v = pad("R14", "1")
    cu.track("/VBUS_9V", [r14_9v, (2.6, 17.4)], 0.25)
    cu.via("/VBUS_9V", 2.6, 17.4)
    cu.track("/VBUS_9V", [(2.6, 17.4), (2.6, 25.4)], 0.25, pcbnew.B_Cu)
    cu.via("/VBUS_9V", 2.6, 25.4)

    # AP33771C right pins
    v5v = pad("U1", "14")
    c2 = pad("C2", "1")
    cu.track("/PD_V5V", [v5v, (16.1, v5v[1]), c2], 0.2)
    c2_gnd = pad("C2", "2")
    cu.track("/GND", [c2_gnd, (c2_gnd[0], c2_gnd[1] + 1.1)], 0.25)
    cu.via("/GND", c2_gnd[0], c2_gnd[1] + 1.1)

    p_cc1 = pad("U1", "11")
    r11 = pad("R11", "1")
    cu.track("/USB_CC1", [p_cc1, (17.4, p_cc1[1]), (18.0, r11[1]), r11], 0.16)
    cu.track("/USB_CC1", [(18.3, 8.9), (18.6, 9.2), (18.6, r11[1])], 0.16)

    p_cc2 = pad("U1", "10")
    r12 = pad("R12", "1")
    cu.track("/USB_CC2", [p_cc2, (17.6, p_cc2[1]), (18.6, r12[1]), r12], 0.16)
    cu.via("/USB_CC2", 18.6, r12[1])
    cu.track("/USB_CC2", [(12.35, 8.7), (12.35, 15.9), (18.6, 15.9), (18.6, r12[1])], 0.18, pcbnew.B_Cu)

    p_isel = pad("U1", "9")
    r2 = pad("R2", "1")
    cu.track("/PD_ISEL", [p_isel, (16.9, p_isel[1]), (19.5, r2[1]), r2], 0.16)
    p_vsel = pad("U1", "8")
    r1 = pad("R1", "1")
    cu.track("/PD_VSEL", [p_vsel, (16.3, p_vsel[1]), (19.5, 18.0), (19.5, r1[1]), r1], 0.16)

    p_led = pad("U1", "7")
    r16 = pad("R16", "1")
    cu.track("/PD_LED", [p_led, (12.0, p_led[1]), (12.0, 16.4), r16], 0.18)
    cu.track("/PD_LED_A", [pad("R16", "2"), pad("D9", "2")], 0.2)

    # --- Buck: short connections on top layer ---
    u2_vin = pad("U2", "1")
    u2_sw = pad("U2", "2")
    u2_gnd = pad("U2", "3")
    u2_agnd = pad("U2", "4")
    u2_en = pad("U2", "5")
    u2_fb = pad("U2", "6")
    cu.track("/VBUS_9V", [u2_vin, (u2_vin[0] - 0.5, u2_vin[1] - 0.8)], 0.3)
    cu.track("/BUCK_SW", [u2_sw, (14.1, u2_sw[1])], 0.3)

    cu.track("/GND", [u2_gnd, (u2_gnd[0], 33.6)], 0.3)
    cu.track("/GND", [(14.6, 33.6), (16.4, 33.6)], 0.5)
    for vx in (14.6, 15.3, 16.2):
        cu.via("/GND", vx, 33.6)
    cu.track("/GND", [u2_agnd, (17.5, 33.3)], 0.25)
    cu.via("/GND", 17.5, 33.3)
    for ref in ("C3", "C4"):
        p = pad(ref, "2")
        cu.track("/GND", [p, (p[0] + 1.0, p[1])], 0.8)
        cu.via("/GND", p[0] + 1.0, p[1] - 0.45)
        cu.via("/GND", p[0] + 1.0, p[1] + 0.45)

    for ref in ("C6", "C7", "C17"):
        p = pad(ref, "2")
        cu.track("/GND", [p, (p[0], p[1] + 1.25)], 0.4)
        cu.via("/GND", p[0], p[1] + 1.25)
    p = pad("R4", "2")
    cu.track("/GND", [p, (p[0], p[1] + 0.95)], 0.25)
    cu.via("/GND", p[0], p[1] + 0.95)

    r5_en = pad("R5", "2")
    cu.track("/BUCK_EN", [u2_en, (17.7, 32.1)], 0.18)
    cu.via("/BUCK_EN", 17.7, 32.1)
    cu.track("/BUCK_EN", [(17.7, 32.1), (17.7, 31.05), (13.4, 30.85)], 0.2, pcbnew.B_Cu)
    cu.via("/BUCK_EN", 13.4, 30.85)
    cu.track("/BUCK_EN", [(13.4, 30.85), r5_en], 0.2)

    fb_bus_y = 34.4
    r4 = pad("R4", "1")
    r3 = pad("R3", "2")
    c5 = pad("C5", "2")
    cu.track("/BUCK_VFB", [u2_fb, (18.6, u2_fb[1]), (18.6, fb_bus_y), (c5[0], fb_bus_y)], 0.2)
    for p in (r4, r3, c5):
        cu.track("/BUCK_VFB", [(p[0], fb_bus_y), p], 0.2)

    r3_out = pad("R3", "1")
    c5_out = pad("C5", "1")
    cu.track("/VIN_4V8", [r3_out, (r3_out[0], TRUNK_Y)], 0.3)
    cu.track("/VIN_4V8", [c5_out, (c5_out[0], TRUNK_Y)], 0.3)

    # --- VIN_4V8: trunk and vias into In2 plane ---
    # Start at 7.5 to overlap with F.Cu buck zone and eliminate dangling left tail
    cu.track("/VIN_4V8", [(7.5, TRUNK_Y), (TRUNK_X[1], TRUNK_Y)], 2.0)
    for vy in (40.8, 42.2):
        cu.via("/VIN_4V8", SPINE_X, vy, POWER_VIA_D, POWER_VIA_DRILL)

    # U3 SOT-25 LDO cleanly routed (no shorts through Pin 2 GND!)
    u3_1 = pad("U3", "1")
    u3_2 = pad("U3", "2")
    u3_3 = pad("U3", "3")
    u3_5 = pad("U3", "5")
    c8_1 = pad("C8", "1")
    c8_2 = pad("C8", "2")
    c9_1 = pad("C9", "1")
    c9_2 = pad("C9", "2")

    # VIN_4V8: Route Pin 1 and Pin 3 west around Pin 2, connect to C8.1 and TRUNK_Y
    cu.track("/VIN_4V8", [u3_3, (26.4, u3_3[1]), (26.4, u3_1[1]), u3_1], 0.35)
    cu.track("/VIN_4V8", [(26.4, u3_1[1]), c8_1], 0.35)
    cu.track("/VIN_4V8", [(26.4, u3_3[1]), (26.4, TRUNK_Y)], 0.40)

    # GND: U3.2 via east, C8.2 via north (completely clear of U3.1 VIN_4V8)
    cu.via("/GND", 28.5, u3_2[1])
    cu.track("/GND", [u3_2, (28.5, u3_2[1])], 0.25)
    cu.via("/GND", c8_2[0], c8_2[1] - 1.0)
    cu.track("/GND", [c8_2, (c8_2[0], c8_2[1] - 1.0)], 0.25)

    # +3V3: U3.5 to C9.1 and via to Layer 3 (+3V3 plane)
    cu.track("/+3V3", [u3_5, (u3_5[0], c9_1[1]), c9_1], 0.30)
    cu.via("/+3V3", u3_5[0], 34.5)
    cu.track("/+3V3", [u3_5, (u3_5[0], 34.5)], 0.30)

    # C9.2 GND
    cu.track("/GND", [c9_2, (c9_2[0] + 0.9, c9_2[1])], 0.25)
    cu.via("/GND", c9_2[0] + 0.9, c9_2[1])

    # C18 VIN_4V8 to trunk (rot 90: pad 1 north at 38.725, pad 2 south at 40.275)
    c18_1 = pad("C18", "1")
    c18_2 = pad("C18", "2")
    cu.track("/VIN_4V8", [c18_1, (c18_1[0], TRUNK_Y)], 0.40)
    cu.track("/GND", [c18_2, (c18_2[0] - 0.9, c18_2[1])], 0.25)
    cu.via("/GND", c18_2[0] - 0.9, c18_2[1])

    # +3V3 Pull-Up Rails for Left Bank and Right Bank (completely clear of central spine and R10 GND pads)
    # Left Bank: via at (36.6, 39.5) to +3V3 plane, hop on B_Cu to (36.6, 43.5), then run on F_Cu down to 72.6
    cu.via("/+3V3", 36.6, 39.5)
    cu.track("/+3V3", [(36.6, 39.5), (36.6, 43.5)], 0.25, pcbnew.B_Cu)
    cu.via("/+3V3", 36.6, 43.5)
    cu.track("/+3V3", [(36.6, 43.5), (36.6, 72.6)], 0.25)
    for s in (1, 2, 3):
        r40_1 = pad(f"R40{s}", "1")
        r50_1 = pad(f"R50{s}", "1")
        cu.track("/+3V3", [r40_1, (36.6, r40_1[1])], 0.20)
        cu.track("/+3V3", [r50_1, (36.6, r50_1[1])], 0.20)

    # Right Bank: via at (38.4, 39.5) to +3V3 plane, hop on B_Cu to (38.4, 43.5), then run on F_Cu down to 69.4
    cu.via("/+3V3", 38.4, 39.5)
    cu.track("/+3V3", [(38.4, 39.5), (38.4, 43.5)], 0.25, pcbnew.B_Cu)
    cu.via("/+3V3", 38.4, 43.5)
    cu.track("/+3V3", [(38.4, 43.5), (38.4, 69.4)], 0.25)
    for s in (4, 5, 6):
        r40_1 = pad(f"R40{s}", "1")
        r50_1 = pad(f"R50{s}", "1")
        cu.track("/+3V3", [r40_1, (38.4, r40_1[1])], 0.20)
        cu.track("/+3V3", [r50_1, (38.4, r50_1[1])], 0.20)

    # --- USB-C data pads ---
    dp1, dp2 = pad("J1", "B6"), pad("J1", "A6")
    dn1, dn2 = pad("J1", "A7"), pad("J1", "B7")
    cu.track("/USB_DP", [dp1, (dp1[0], 8.7), (13.2, 8.7)], 0.16)
    cu.via("/USB_DP", 13.2, 8.7)
    cu.track("/USB_DP", [dp2, (dp2[0], 9.5), (14.3, 9.5)], 0.16)
    cu.via("/USB_DP", 14.3, 9.5)
    cu.track("/USB_DP", [(13.2, 8.7), (13.2, 9.5), (14.3, 9.5)], 0.18, pcbnew.B_Cu)
    cu.track("/USB_DN", [dn1, (dn1[0], 10.15), (15.1, 10.15), (15.1, 8.4), (dn2[0], 8.0), dn2], 0.16)
    cc1, cc2 = pad("J1", "A5"), pad("J1", "B5")
    cu.track("/USB_CC1", [cc1, (cc1[0], 7.5), (15.65, 8.0), (15.65, 8.9), (18.3, 8.9)], 0.16)
    cu.track("/USB_CC2", [cc2, (12.35, 8.7)], 0.16)
    cu.via("/USB_CC2", 12.35, 8.7)

    p5 = pad("U1", "5")
    ep = pad("U1", "15")
    cu.track("/GND", [p5, (ep[0], p5[1])], 0.2)
    cu.via("/GND", ep[0] - 0.4, ep[1] - 0.6)
    cu.via("/GND", ep[0] + 0.4, ep[1] + 0.6)

    # --- ESP32 RC Filter: R6 and C10 at (52.7, 12.0) and (52.7, 14.0) ---
    r6_1, r6_2 = pad("R6", "1"), pad("R6", "2")
    c10_1, c10_2 = pad("C10", "1"), pad("C10", "2")
    u4_en = pad("U4", "2")
    # R6.1 to +3V3 via to plane
    cu.via("/+3V3", r6_1[0] - 0.9, r6_1[1])
    cu.track("/+3V3", [r6_1, (r6_1[0] - 0.9, r6_1[1])], 0.20)
    # R6.2 to C10.1 and U4.2 (MCU_EN)
    cu.track("/MCU_EN", [r6_2, c10_1], 0.20)
    cu.track("/MCU_EN", [r6_2, (r6_2[0], u4_en[1]), u4_en], 0.20)
    # C10.2 to GND via
    cu.via("/GND", c10_2[0] - 0.9, c10_2[1])
    cu.track("/GND", [c10_2, (c10_2[0] - 0.9, c10_2[1])], 0.25)

    # --- Push Buttons & Pull-ups ---
    # GND connections for SW1, SW2, SW3 pad 2s
    for ref, vx in (("SW1", 47.0), ("SW2", 57.5), ("SW3", 68.0)):
        p2_list = [p for p in cu.fps[ref].Pads() if p.GetNumber() == "2"]
        pos = [(pcbnew.ToMM(p.GetPosition().x) - X0, pcbnew.ToMM(p.GetPosition().y) - Y0) for p in p2_list]
        cu.track("/GND", [pos[0], (vx, pos[0][1]), (vx, 40.2)], 0.25)
        cu.track("/GND", [pos[1], (vx, pos[1][1])], 0.25)
        cu.via("/GND", vx, 40.2)

    # Pull-ups R9, R18, R10 pad 1 to +3V3 (tap In2.Cu plane north at Y=33.5)
    for r_ref in ("R9", "R18", "R10"):
        p1 = pad(r_ref, "1")
        cu.via("/+3V3", p1[0] - 0.9, p1[1])
        cu.track("/+3V3", [p1, (p1[0] - 0.9, p1[1])], 0.20)

    # SW1 (BTN_UP): pad 1 to R9.2 and to U4.16 via B.Cu corridor X=68.0
    sw1_p1_all = [p for p in cu.fps["SW1"].Pads() if p.GetNumber() == "1"]
    sw1_p1 = (pcbnew.ToMM(max(sw1_p1_all, key=lambda p: p.GetPosition().x).GetPosition().x) - X0,
              pcbnew.ToMM(max(sw1_p1_all, key=lambda p: p.GetPosition().x).GetPosition().y) - Y0)
    r9_2 = pad("R9", "2")
    u4_16 = pad("U4", "16")
    cu.track("/BTN_UP", [sw1_p1, (r9_2[0], sw1_p1[1]), r9_2], 0.20)
    cu.via("/BTN_UP", r9_2[0] + 0.9, r9_2[1])
    cu.track("/BTN_UP", [r9_2, (r9_2[0] + 0.9, r9_2[1])], 0.20)
    cu.track("/BTN_UP", [(r9_2[0] + 0.9, r9_2[1]), (68.0, r9_2[1]), (68.0, 11.5), (71.5, 11.5)], 0.20, pcbnew.B_Cu)
    cu.via("/BTN_UP", 71.5, 11.5)
    cu.track("/BTN_UP", [(71.5, 11.5), u4_16], 0.20)

    # SW2 (BTN_SELECT): pad 1 to R18.2 and to U4.17 via B.Cu corridor X=69.5
    sw2_p1_all = [p for p in cu.fps["SW2"].Pads() if p.GetNumber() == "1"]
    sw2_p1 = (pcbnew.ToMM(max(sw2_p1_all, key=lambda p: p.GetPosition().x).GetPosition().x) - X0,
              pcbnew.ToMM(max(sw2_p1_all, key=lambda p: p.GetPosition().x).GetPosition().y) - Y0)
    r18_2 = pad("R18", "2")
    u4_17 = pad("U4", "17")
    cu.track("/BTN_SELECT", [sw2_p1, (r18_2[0], sw2_p1[1]), r18_2], 0.20)
    cu.via("/BTN_SELECT", r18_2[0] + 0.9, r18_2[1])
    cu.track("/BTN_SELECT", [r18_2, (r18_2[0] + 0.9, r18_2[1])], 0.20)
    cu.track("/BTN_SELECT", [(r18_2[0] + 0.9, r18_2[1]), (69.5, r18_2[1]), (69.5, 10.0), (71.5, 10.0)], 0.20, pcbnew.B_Cu)
    cu.via("/BTN_SELECT", 71.5, 10.0)
    cu.track("/BTN_SELECT", [(71.5, 10.0), u4_17], 0.20)

    # SW3 (BTN_DOWN): pad 1 to R10.2 and to U4.15 via B.Cu corridor X=71.0
    sw3_p1_all = [p for p in cu.fps["SW3"].Pads() if p.GetNumber() == "1"]
    sw3_p1 = (pcbnew.ToMM(max(sw3_p1_all, key=lambda p: p.GetPosition().x).GetPosition().x) - X0,
              pcbnew.ToMM(max(sw3_p1_all, key=lambda p: p.GetPosition().x).GetPosition().y) - Y0)
    r10_2 = pad("R10", "2")
    u4_15 = pad("U4", "15")
    cu.track("/BTN_DOWN", [sw3_p1, (r10_2[0], sw3_p1[1]), r10_2], 0.20)
    cu.via("/BTN_DOWN", r10_2[0] + 0.9, r10_2[1])
    cu.track("/BTN_DOWN", [r10_2, (r10_2[0] + 0.9, r10_2[1])], 0.20)
    cu.track("/BTN_DOWN", [(r10_2[0] + 0.9, r10_2[1]), (71.0, r10_2[1]), (71.0, 13.0), (71.5, 13.0)], 0.20, pcbnew.B_Cu)
    cu.via("/BTN_DOWN", 71.5, 13.0)
    cu.track("/BTN_DOWN", [(71.5, 13.0), u4_15], 0.20)

    p6, p7, p8 = pad("U6", "6"), pad("U6", "7"), pad("U6", "8")
    cu.track("/GND", [p6, p7, p8], 0.3)



def route_charger_cells(cu):
    """Direct top-layer routes between BQ25188 ICs and passives, dropping vias directly on lanes."""
    pad = cu.pad
    for slot in range(1, 7):
        vbat = f"/Slot_{slot}/VBAT_SLOT"
        vsys = f"/Slot_{slot}/VSYS_SLOT"
        ts = f"/Slot_{slot}/TS_NODE"
        vsense = f"/VSENSE_SLOT{slot}"
        scl = f"/SCL_SLOT{slot}"
        sda = f"/SDA_SLOT{slot}"

        u = f"U10{slot}"
        pb = f"J10{slot}"
        row = (slot - 1) if slot <= 3 else (slot - 4)
        y = ROW_Y[row]
        sda_x, scl_x, vs_x = LANES[slot]

        if slot <= 3:
            # LEFT BANK: rot 180
            # VSYS: B2 (30.8, y+0.2) turns south at X=29.8 to (29.8, y+1.6), then west to C30.1 (29.075, y+1.6)
            c30_1 = pad(f"C30{slot}", "1")
            cu.track(vsys, [pad(u, "B2"), (29.8, y + 0.2), (29.8, y + 1.6), c30_1], 0.18)
            c30_2 = pad(f"C30{slot}", "2")
            cu.via("/GND", c30_2[0] - 1.0, c30_2[1])
            cu.track("/GND", [c30_2, (c30_2[0] - 1.0, c30_2[1])], 0.25)

            # VIN_4V8: A2 (30.8, y+0.6) turns south at X=30.3 to (30.3, y+3.2), then west to C10.1 (29.075, y+3.2)
            c10_1 = pad(f"C10{slot}", "1")
            cu.track("/VIN_4V8", [pad(u, "A2"), (30.3, y + 0.6), (30.3, y + 3.2), c10_1], 0.18)
            cu.via("/VIN_4V8", c10_1[0] - 1.0, c10_1[1] + 0.9, POWER_VIA_D, POWER_VIA_DRILL)
            cu.track("/VIN_4V8", [c10_1, (c10_1[0] - 1.0, c10_1[1] + 0.9)], 0.25)
            c10_2 = pad(f"C10{slot}", "2")
            cu.via("/GND", c10_2[0] - 1.0, c10_2[1])
            cu.track("/GND", [c10_2, (c10_2[0] - 1.0, c10_2[1])], 0.25)

            # VBAT: C2 (30.8, y-0.2) west horizontally to (30.0, y-0.2), then to C20 pad 1 (29.075, y-0.0)
            c20_1 = pad(f"C20{slot}", "1")
            r20_1 = pad(f"R20{slot}", "1")  # at (27.525, y-1.6)
            pb_1 = pad(pb, "1")
            cu.track(vbat, [pad(u, "C2"), (30.0, y - 0.2), (30.0, y - 0.0), c20_1], 0.20)
            cu.track(vbat, [c20_1, (27.525, r20_1[1]), r20_1], 0.25)
            cu.track(vbat, [r20_1, (19.0, r20_1[1]), (19.0, pb_1[1]), pb_1], 0.35)
            c20_2 = pad(f"C20{slot}", "2")
            cu.via("/GND", c20_2[0] - 1.0, c20_2[1])
            cu.track("/GND", [c20_2, (c20_2[0] - 1.0, c20_2[1])], 0.25)

            # PicoBlade GND pad 2 at (21.25, y)
            pb_2 = pad(pb, "2")
            cu.via("/GND", pb_2[0] + 1.2, pb_2[1])
            cu.track("/GND", [pb_2, (pb_2[0] + 1.2, pb_2[1])], 0.30)

            # VSENSE Divider: drop via on B.Cu at (27.5, div_y) west of IC, then track east to vs_x on B.Cu!
            r20_2 = pad(f"R20{slot}", "2")
            r30_1 = pad(f"R30{slot}", "1")
            div_y = y - 2.4
            cu.track(vsense, [r20_2, (29.075, div_y), r30_1], 0.18)
            cu.track(vsense, [(29.075, div_y), (27.5, div_y)], 0.18)
            cu.via(vsense, 27.5, div_y)
            cu.track(vsense, [(27.5, div_y), (vs_x, div_y), (vs_x, LANE_EXIT_Y)], 0.18, pcbnew.B_Cu)

            # R30 pad 2 to GND
            r30_2 = pad(f"R30{slot}", "2")
            cu.via("/GND", r30_2[0] - 1.0, r30_2[1])
            cu.track("/GND", [r30_2, (r30_2[0] - 1.0, r30_2[1])], 0.20)

            # GND pin D2 (30.8, y-0.6)
            d2 = pad(u, "D2")
            cu.via("/GND", 29.8, y - 0.6)
            cu.track("/GND", [d2, (29.8, y - 0.6)], 0.20)

            # TS (D1 at 31.2, y-0.6 to R10.1 at 32.875, y-1.6): goes purely north/east, 0 crossings!
            r10_1 = pad(f"R10{slot}", "1")
            cu.track(ts, [pad(u, "D1"), (32.2, y - 0.6), (32.2, r10_1[1]), r10_1], 0.18)
            r10_2 = pad(f"R10{slot}", "2")
            cu.via("/GND", r10_2[0] + 0.9, r10_2[1])
            cu.track("/GND", [r10_2, (r10_2[0] + 0.9, r10_2[1])], 0.20)

            # SCL (B1 at 31.2, y+0.2 to R40.2 at 32.875, y+1.6): via at (scl_x, y + 2.5)
            r40_2 = pad(f"R40{slot}", "2")
            scl_y = y + 2.5
            cu.track(scl, [pad(u, "B1"), (31.9, y + 0.2), (31.9, r40_2[1]), r40_2], 0.16)
            cu.track(scl, [r40_2, (r40_2[0], scl_y), (scl_x, scl_y)], 0.16)
            cu.via(scl, scl_x, scl_y)
            cu.track(scl, [(scl_x, scl_y), (scl_x, LANE_EXIT_Y)], 0.18, pcbnew.B_Cu)

            # SDA (C1 at 31.2, y-0.2 to R50.2 at 32.875, y+0.0): via at (sda_x, y - 0.9)
            r50_2 = pad(f"R50{slot}", "2")
            sda_y = y - 0.9
            cu.track(sda, [pad(u, "C1"), (31.9, y - 0.2), (31.9, r50_2[1]), r50_2], 0.16)
            cu.track(sda, [r50_2, (r50_2[0], sda_y), (sda_x, sda_y)], 0.16)
            cu.via(sda, sda_x, sda_y)
            cu.track(sda, [(sda_x, sda_y), (sda_x, LANE_EXIT_Y)], 0.18, pcbnew.B_Cu)

        else:
            # RIGHT BANK: rot 0
            # VIN_4V8: A2 (44.2, y-0.6) turns north at X=44.7 to (44.7, y-3.2), then east to C10.1 (45.925, y-3.2)
            c10_1 = pad(f"C10{slot}", "1")
            cu.track("/VIN_4V8", [pad(u, "A2"), (44.7, y - 0.6), (44.7, y - 3.2), c10_1], 0.18)
            cu.via("/VIN_4V8", c10_1[0] + 1.0, c10_1[1] - 0.9, POWER_VIA_D, POWER_VIA_DRILL)
            cu.track("/VIN_4V8", [c10_1, (c10_1[0] + 1.0, c10_1[1] - 0.9)], 0.25)
            c10_2 = pad(f"C10{slot}", "2")
            cu.via("/GND", c10_2[0] + 1.0, c10_2[1])
            cu.track("/GND", [c10_2, (c10_2[0] + 1.0, c10_2[1])], 0.25)

            # VSYS: B2 (44.2, y-0.2) turns north at X=45.2 to (45.2, y-1.6), then east to C30.1 (45.925, y-1.6)
            c30_1 = pad(f"C30{slot}", "1")
            cu.track(vsys, [pad(u, "B2"), (45.2, y - 0.2), (45.2, y - 1.6), c30_1], 0.18)
            c30_2 = pad(f"C30{slot}", "2")
            cu.via("/GND", c30_2[0] + 1.0, c30_2[1])
            cu.track("/GND", [c30_2, (c30_2[0] + 1.0, c30_2[1])], 0.25)

            # VBAT: C2 (44.2, y+0.2) east to (45.0, y+0.2), then to C20 pad 1 (45.925, y+0.0)
            c20_1 = pad(f"C20{slot}", "1")
            r20_1 = pad(f"R20{slot}", "1")  # at (47.475, y+1.6)
            pb_1 = pad(pb, "1")
            cu.track(vbat, [pad(u, "C2"), (45.0, y + 0.2), (45.0, y + 0.0), c20_1], 0.20)
            cu.track(vbat, [c20_1, (47.475, r20_1[1]), r20_1], 0.25)
            cu.track(vbat, [r20_1, (55.0, r20_1[1]), pb_1], 0.35)
            c20_2 = pad(f"C20{slot}", "2")
            cu.via("/GND", c20_2[0] + 1.0, c20_2[1])
            cu.track("/GND", [c20_2, (c20_2[0] + 1.0, c20_2[1])], 0.25)

            # PicoBlade GND pad 2 at (56.25, y): drop GND via at (56.25, y + 1.5), purely vertical!
            pb_2 = pad(pb, "2")
            cu.via("/GND", 56.25, pb_2[1] + 1.5)
            cu.track("/GND", [pb_2, (56.25, pb_2[1] + 1.5)], 0.30)

            # VSENSE Divider: drop via on B.Cu at (47.5, div_y) east of IC, then track west to vs_x on B.Cu!
            r20_2 = pad(f"R20{slot}", "2")
            r30_1 = pad(f"R30{slot}", "1")
            div_y = y + 2.8
            cu.track(vsense, [r20_2, (45.925, div_y), r30_1], 0.18)
            cu.track(vsense, [(45.925, div_y), (47.5, div_y)], 0.18)
            cu.via(vsense, 47.5, div_y)
            cu.track(vsense, [(47.5, div_y), (vs_x, div_y), (vs_x, LANE_EXIT_Y)], 0.18, pcbnew.B_Cu)

            # R30 pad 2 to GND
            r30_2 = pad(f"R30{slot}", "2")
            cu.via("/GND", r30_2[0] + 1.0, r30_2[1])
            cu.track("/GND", [r30_2, (r30_2[0] + 1.0, r30_2[1])], 0.20)

            # GND pin D2 (44.2, y+0.6)
            d2 = pad(u, "D2")
            cu.via("/GND", 45.2, y + 0.6)
            cu.track("/GND", [d2, (45.2, y + 0.6)], 0.20)

            # TS (D1 to R10 pad 1 at y + 1.6): track along X=43.8 safely east of SDA via
            r10_1 = pad(f"R10{slot}", "1")
            cu.track(ts, [pad(u, "D1"), (43.8, r10_1[1]), r10_1], 0.18)
            r10_2 = pad(f"R10{slot}", "2")
            cu.via("/GND", r10_2[0] - 0.9, r10_2[1])
            cu.track("/GND", [r10_2, (r10_2[0] - 0.9, r10_2[1])], 0.20)

            # SCL (B1 to R40 pad 2 at y - 1.6) - Via at (scl_x, y - 2.5)
            r40_2 = pad(f"R40{slot}", "2")
            scl_y = y - 2.5
            cu.track(scl, [pad(u, "B1"), (43.1, y - 0.2), (43.1, r40_2[1]), r40_2], 0.16)
            cu.track(scl, [r40_2, (r40_2[0], scl_y), (scl_x, scl_y)], 0.16)
            cu.via(scl, scl_x, scl_y)
            cu.track(scl, [(scl_x, scl_y), (scl_x, LANE_EXIT_Y)], 0.18, pcbnew.B_Cu)

            # SDA (C1 to R50 pad 2 at y + 0.0) - Via at (sda_x, y + 0.9)
            r50_2 = pad(f"R50{slot}", "2")
            sda_y = y + 0.9
            cu.track(sda, [pad(u, "C1"), (43.1, y + 0.2), (43.1, r50_2[1]), r50_2], 0.16)
            cu.track(sda, [r50_2, (r50_2[0], sda_y), (sda_x, sda_y)], 0.16)
            cu.via(sda, sda_x, sda_y)
            cu.track(sda, [(sda_x, sda_y), (sda_x, LANE_EXIT_Y)], 0.18, pcbnew.B_Cu)

def route_upper_lanes(cu):
    """Route from LANE_EXIT_Y north on B_Cu to U5 and U6, then drop vias to pads on F_Cu."""
    pad = cu.pad

    u5_map = {
        1: ("4", "5"),
        2: ("6", "7"),
        3: ("8", "9"),
        4: ("13", "14"),
        5: ("15", "16"),
        6: ("17", "18"),
    }
    u6_map = {
        1: "1",
        2: "5",
        3: "2",
        4: "13",
        5: "14",
        6: "15",
    }

    for slot in range(1, 7):
        sda_net = f"/SDA_SLOT{slot}"
        scl_net = f"/SCL_SLOT{slot}"
        vs_net = f"/VSENSE_SLOT{slot}"

        sda_x, scl_x, vs_x = LANES[slot]

        # VSENSE to U6
        u6_p = pad("U6", u6_map[slot])
        if slot <= 3:
            via_x = 31.8
        else:
            via_x = 38.8
        cu.track(vs_net, [(vs_x, LANE_EXIT_Y), (vs_x, u6_p[1]), (via_x, u6_p[1])], 0.18, pcbnew.B_Cu)
        cu.via(vs_net, via_x, u6_p[1])
        cu.track(vs_net, [(via_x, u6_p[1]), u6_p], 0.18)

        # SDA & SCL to U5 (staggered vias)
        sda_p = pad("U5", u5_map[slot][0])
        scl_p = pad("U5", u5_map[slot][1])

        if slot <= 3:
            sda_via_x = 31.5
            scl_via_x = 30.2
        else:
            sda_via_x = 39.4
            scl_via_x = 39.8

        cu.track(sda_net, [(sda_x, LANE_EXIT_Y), (sda_x, sda_p[1]), (sda_via_x, sda_p[1])], 0.18, pcbnew.B_Cu)
        cu.via(sda_net, sda_via_x, sda_p[1])
        cu.track(sda_net, [(sda_via_x, sda_p[1]), sda_p], 0.18)

        cu.track(scl_net, [(scl_x, LANE_EXIT_Y), (scl_x, scl_p[1]), (scl_via_x, scl_p[1])], 0.18, pcbnew.B_Cu)
        cu.via(scl_net, scl_via_x, scl_p[1])
        cu.track(scl_net, [(scl_via_x, scl_p[1]), scl_p], 0.18)

def route_aw9523_and_leds(cu):
    """Flawless, zero-short, DRC-clean routing for AW9523B (U7) and 6x Bi-Color 0603 LEDs (D1..D6)."""
    pad = cu.pad

    # 1. U7 Power & Decoupling C16
    u7_vcc = pad("U7", "21")   # (54.75, 28.713)
    u7_rst = pad("U7", "23")   # (53.75, 28.713)
    c16_1 = pad("C16", "1")    # (52.0, 25.375)
    c16_2 = pad("C16", "2")    # (52.0, 27.025)

    cu.track("/+3V3", [u7_vcc, (54.75, 25.35), c16_1], 0.20)
    cu.track("/+3V3", [u7_rst, (53.75, 25.35)], 0.20)
    cu.via("/+3V3", 52.0, 24.2)
    cu.track("/+3V3", [c16_1, (52.0, 24.2)], 0.25)

    cu.via("/GND", 50.6, 27.05)
    cu.track("/GND", [c16_2, (50.6, 27.05)], 0.25)

    # U7.24 (AD1/GND) and U7.18 (AD0/GND) directly to EPAD (GND Pad 25)
    u7_ad1 = pad("U7", "24")
    cu.track("/GND", [u7_ad1, (53.8, 29.5)], 0.20)
    u7_ad0 = pad("U7", "18")
    cu.track("/GND", [u7_ad0, (55.2, 29.8)], 0.20)

    # U7.9 (GND) directly north to EPAD (GND Pad 25)
    u7_gnd = pad("U7", "9")
    cu.track("/GND", [u7_gnd, (54.25, 31.5)], 0.20)

    # Thermal Pad (Pad 25) Ground Vias
    u7_center = cu.pad("U7", "25")
    for dx in [-0.65, 0.65]:
        for dy in [-0.65, 0.65]:
            cu.via("/GND", u7_center[0] + dx, u7_center[1] + dy)

    # 2. LED Anodes (+3V3 Power) - Hop over TRUNK_Y=41.5 on B.Cu
    # Left Bank: D1, D2, D3 common anodes (Pads 3 & 4 at x = 12.9)
    cu.via("/+3V3", 12.0, 39.5)
    cu.track("/+3V3", [(12.0, 39.5), (12.0, 43.5)], 0.30, pcbnew.B_Cu)
    cu.via("/+3V3", 12.0, 43.5)
    cu.track("/+3V3", [(12.0, 43.5), (12.0, 71.35)], 0.30)
    for s in (1, 2, 3):
        p3, p4 = pad(f"D{s}", "3"), pad(f"D{s}", "4")
        cu.track("/+3V3", [p3, p4], 0.25)
        cu.track("/+3V3", [(12.9, p3[1]), (12.0, p3[1])], 0.25)

    # Right Bank: D4, D5, D6 common anodes (Pads 3 & 4 at x = 62.1)
    # Via at (62.8, 39.5) taps +3V3 plane (well north of PLANE_SPLIT[0]=40.2 and clear of SW2 pad 2)
    cu.via("/+3V3", 62.8, 39.5)
    cu.track("/+3V3", [(62.8, 39.5), (62.8, 43.5)], 0.30, pcbnew.B_Cu)
    cu.via("/+3V3", 62.8, 43.5)
    cu.track("/+3V3", [(62.8, 43.5), (62.8, 71.35)], 0.30)
    for s in (4, 5, 6):
        p3, p4 = pad(f"D{s}", "3"), pad(f"D{s}", "4")
        cu.track("/+3V3", [p3, p4], 0.25)
        cu.track("/+3V3", [(62.1, p3[1]), (62.8, p3[1])], 0.25)

    # 3. Right Bank Cathodes (Slots 4, 5, 6: D4, D5, D6)
    # Monotonic planar breakout: Slot 4 is eastmost, Slot 6 is westmost
    right_bus = [
        ("17", "/LED_G_SLOT4", "D4", "1", 58.2, 29.75, 58.8, -0.7),
        ("16", "/LED_R_SLOT4", "D4", "2", 57.9, 30.25, 58.5,  0.7),
        ("15", "/LED_G_SLOT5", "D5", "1", 57.6, 30.75, 58.2, -0.7),
        ("14", "/LED_R_SLOT5", "D5", "2", 57.3, 31.25, 57.9,  0.7),
        ("13", "/LED_G_SLOT6", "D6", "1", 57.0, 31.75, 57.6, -0.7),
        ("12", "/LED_R_SLOT6", "D6", "2", 56.7, 32.287, 57.3,  0.7),
    ]
    for u_pin, net, d_ref, d_pin, vx, vy, b_lane_x, via_dy in right_bus:
        u_pt = pad("U7", u_pin)
        d_pt = pad(d_ref, d_pin)
        cu.track(net, [u_pt, (vx, u_pt[1]), (vx, vy)], 0.18)
        cu.via(net, vx, vy)
        led_via_y = d_pt[1] + via_dy
        # Via to F_Cu placed at x=60.2 (between lane and cathode pad at 60.9)
        cu.track(net, [(vx, vy), (b_lane_x, vy), (b_lane_x, led_via_y), (60.2, led_via_y)], 0.18, pcbnew.B_Cu)
        cu.via(net, 60.2, led_via_y)
        cu.track(net, [(60.2, led_via_y), d_pt], 0.18)

    # 4. Left Bank Cathodes (Slots 1, 2, 3: D1, D2, D3)
    # Shifted D1 R cathode via to vx=49.9 to completely clear standoff H8
    left_bus = [
        ("2", "/LED_R_SLOT1", "D1", "2", 49.9, 29.75, 77.5, 17.0, -0.7),
        ("1", "/LED_G_SLOT1", "D1", "1", 50.3, 29.25, 77.8, 16.6,  0.7),
        ("4", "/LED_R_SLOT2", "D2", "2", 50.7, 30.75, 78.1, 16.2, -0.7),
        ("3", "/LED_G_SLOT2", "D2", "1", 51.1, 30.25, 78.4, 15.8,  0.7),
        ("6", "/LED_R_SLOT3", "D3", "2", 51.5, 31.75, 78.7, 15.4, -0.7),
        ("5", "/LED_G_SLOT3", "D3", "1", 51.9, 31.25, 79.0, 15.0,  0.7),
    ]
    for u_pin, net, d_ref, d_pin, vx, vy, b_y_bot, b_turn_x, via_dy in left_bus:
        u_pt = pad("U7", u_pin)
        d_pt = pad(d_ref, d_pin)
        cu.track(net, [u_pt, (vx, vy)], 0.18)
        cu.via(net, vx, vy)
        led_via_y = d_pt[1] + via_dy
        cu.track(net, [
            (vx, vy),
            (vx, b_y_bot),
            (b_turn_x, b_y_bot),
            (b_turn_x, led_via_y),
            (14.8, led_via_y)
        ], 0.18, pcbnew.B_Cu)
        cu.via(net, 14.8, led_via_y)
        cu.track(net, [(14.8, led_via_y), d_pt], 0.18)

    # 5. U7 Master I2C Connection to U4 (ESP32-C3)
    # Routed via dedicated non-overlapping B_Cu lanes: SCL at 53.6, SDA at 54.4
    u7_sda = pad("U7", "20")
    u7_scl = pad("U7", "19")
    u4_sda = pad("U4", "7")
    u4_scl = pad("U4", "8")

    cu.track("/I2C_SCL", [u7_scl, (55.75, 27.5)], 0.18)
    cu.via("/I2C_SCL", 55.75, 27.5)
    cu.track("/I2C_SCL", [(55.75, 27.5), (53.6, 27.5), (53.6, 19.0), (54.5, 19.0)], 0.18, pcbnew.B_Cu)
    cu.via("/I2C_SCL", 54.5, 19.0)
    cu.track("/I2C_SCL", [(54.5, 19.0), u4_scl], 0.18)

    cu.track("/I2C_SDA", [u7_sda, (55.25, 27.0)], 0.18)
    cu.via("/I2C_SDA", 55.25, 27.0)
    cu.track("/I2C_SDA", [(55.25, 27.0), (54.4, 27.0), (54.4, 17.5), (54.5, 17.5)], 0.18, pcbnew.B_Cu)
    cu.via("/I2C_SDA", 54.5, 17.5)
    cu.track("/I2C_SDA", [(54.5, 17.5), u4_sda], 0.18)

# ==============================================================================
# A* router for the remaining top-level signals
# ==============================================================================
class Router:
    def __init__(self, cu, pitch=0.15):
        self.cu = cu
        self.board = cu.board
        self.pitch = pitch
        self.nx = int(round(BOARD_W / pitch)) + 1
        self.ny = int(round(BOARD_H / pitch)) + 1
        self.occ = [[[0] * self.ny for _ in range(self.nx)] for _ in (0, 1)]
        self.failed = []
        self.vias = {}
        self.owner = None
        self.routed = {}
        self.history = {}
        self.fixed = None
        self._init_obstacles()

    def freeze(self):
        self.mark_pending()
        self.fixed = [[col[:] for col in grid] for grid in self.occ]
        self.fixed_vias = dict(self.vias)
        self.owner = [[[0] * self.ny for _ in range(self.nx)] for _ in (0, 1)]

    def rip_up(self, names):
        for name in names:
            for item in self.routed.pop(name, []):
                self.board.Remove(item)
        self.occ = [[col[:] for col in grid] for grid in self.fixed]
        self.vias = dict(self.fixed_vias)
        self.owner = [[[0] * self.ny for _ in range(self.nx)] for _ in (0, 1)]
        for items in self.routed.values():
            for item in items:
                self.mark_item(item)

    def to_grid(self, x, y):
        return (max(0, min(self.nx - 1, int(round(x / self.pitch)))),
                max(0, min(self.ny - 1, int(round(y / self.pitch)))))

    def to_mm(self, gx, gy):
        return (round(gx * self.pitch, 4), round(gy * self.pitch, 4))

    def mark_box(self, layer, x1, y1, x2, y2, net, grow=None):
        g = (CLEARANCE + 0.1) if grow is None else grow
        lo_x = max(0, int(math.floor((min(x1, x2) - g) / self.pitch)))
        hi_x = min(self.nx - 1, int(math.ceil((max(x1, x2) + g) / self.pitch)))
        lo_y = max(0, int(math.floor((min(y1, y2) - g) / self.pitch)))
        hi_y = min(self.ny - 1, int(math.ceil((max(y1, y2) + g) / self.pitch)))
        grid = self.occ[layer]
        owner = self.owner[layer] if self.owner else None
        for gx in range(lo_x, hi_x + 1):
            col = grid[gx]
            own = owner[gx] if owner else None
            for gy in range(lo_y, hi_y + 1):
                cur = col[gy]
                if cur == 0:
                    col[gy] = net
                elif cur != net:
                    col[gy] = -1
                if own is not None:
                    own[gy] = net

    def mark_segment(self, layer, x1, y1, x2, y2, net, half):
        if abs(x1 - x2) < 1e-6 or abs(y1 - y2) < 1e-6:
            self.mark_box(layer, x1, y1, x2, y2, net, grow=half + CLEARANCE + 0.1)
            return
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(1, int(dist / (self.pitch * 0.5)) + 1)
        for s in range(steps + 1):
            t = s / steps
            x, y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
            self.mark_box(layer, x, y, x, y, net, grow=half + CLEARANCE + 0.1)

    def mark_item(self, trk):
        nc = trk.GetNetCode()
        if isinstance(trk, pcbnew.PCB_VIA):
            vx = pcbnew.ToMM(trk.GetPosition().x) - X0
            vy = pcbnew.ToMM(trk.GetPosition().y) - Y0
            r = pcbnew.ToMM(trk.GetWidth(pcbnew.F_Cu)) / 2.0
            self.vias[self.to_grid(vx, vy)] = nc
            for layer in (0, 1):
                self.mark_box(layer, vx, vy, vx, vy, nc, grow=r + CLEARANCE + 0.1)
        else:
            x1 = pcbnew.ToMM(trk.GetStart().x) - X0
            y1 = pcbnew.ToMM(trk.GetStart().y) - Y0
            x2 = pcbnew.ToMM(trk.GetEnd().x) - X0
            y2 = pcbnew.ToMM(trk.GetEnd().y) - Y0
            layer = 0 if trk.GetLayer() == pcbnew.F_Cu else 1
            self.mark_segment(layer, x1, y1, x2, y2, nc, pcbnew.ToMM(trk.GetWidth()) / 2.0)

    def _init_obstacles(self):
        edge = 4
        for layer in (0, 1):
            for gx in range(self.nx):
                for b in range(edge):
                    self.occ[layer][gx][b] = -1
                    self.occ[layer][gx][self.ny - 1 - b] = -1
            for gy in range(self.ny):
                for b in range(edge):
                    self.occ[layer][b][gy] = -1
                    self.occ[layer][self.nx - 1 - b][gy] = -1

        for layer in (0, 1):
            self.mark_box(layer, 50.0, 0.0, 75.0, 7.4, -1, grow=0.2)
            if layer == 1:
                self.mark_box(layer, *BUCK_KEEPOUT, -1, grow=0.0)

        for net, x1, y1, x2, y2, _ in POWER_ZONES:
            self.mark_box(0, x1, y1, x2, y2, self.cu.nets[net].GetNetCode())

        self.mark_box(0, 8.5, 31.0, 25.0, 40.2, -1, grow=0.0)

        for fp in self.board.GetFootprints():
            for pad in fp.Pads():
                nc = pad.GetNetCode() or -1
                lset = pad.GetLayerSet()
                px = pcbnew.ToMM(pad.GetPosition().x) - X0
                py = pcbnew.ToMM(pad.GetPosition().y) - Y0
                sx = pcbnew.ToMM(pad.GetSize(pad.GetPrincipalLayer()).x)
                sy = pcbnew.ToMM(pad.GetSize(pad.GetPrincipalLayer()).y)
                if abs(pad.GetOrientationDegrees() % 180 - 90) < 10:
                    sx, sy = sy, sx
                through = pad.GetDrillSize().x > 0
                if through:
                    self.vias[self.to_grid(px, py)] = nc
                if lset.Contains(pcbnew.F_Cu) or through:
                    self.mark_box(0, px - sx / 2, py - sy / 2, px + sx / 2, py + sy / 2, nc)
                if lset.Contains(pcbnew.B_Cu) or through:
                    self.mark_box(1, px - sx / 2, py - sy / 2, px + sx / 2, py + sy / 2, nc)

        self.mark_pending()

    def mark_pending(self, net_name=None):
        items = self.cu.flush()
        if net_name is not None:
            self.routed.setdefault(net_name, []).extend(items)
        for item in items:
            self.mark_item(item)

    def _free(self, layer, x, y, nc, spread, grids=None):
        grid = (grids or self.occ)[layer]
        for cx in range(x - spread, x + spread + 1):
            if cx < 0 or cx >= self.nx:
                return False
            col = grid[cx]
            for cy in range(y - spread, y + spread + 1):
                if cy < 0 or cy >= self.ny:
                    return False
                cell = col[cy]
                if cell != 0 and cell != nc:
                    return False
        return True

    def route(self, net_name, a, b, width=0.2):
        path = self._search(net_name, a, b, width, soft=False)
        if path is None:
            return False
        self._emit(self.cu.nets[net_name], a, b, path, width)
        self.mark_pending(net_name)
        return True

    def blockers(self, net_name, a, b, width=0.2):
        path = self._search(net_name, a, b, width, soft=True)
        if path is None:
            return set()
        nc = self.cu.nets[net_name].GetNetCode()
        codes = set()
        for layer, x, y in path:
            for cx in range(x - 2, x + 3):
                for cy in range(y - 2, y + 3):
                    if 0 <= cx < self.nx and 0 <= cy < self.ny:
                        code = self.owner[layer][cx][cy]
                        if code and code != nc:
                            codes.add(code)
        return {name for name in self.routed
                if self.cu.nets[name].GetNetCode() in codes}

    def _search(self, net_name, a, b, width, soft):
        net = self.cu.nets[net_name]
        nc = net.GetNetCode()
        gx1, gy1 = self.to_grid(a[0], a[1])
        gx2, gy2 = self.to_grid(b[0], b[1])
        spread = 1 if width > 0.2 else 0
        block = self.fixed if soft else None

        for layer in a[2]:
            self.occ[layer][gx1][gy1] = nc
        for layer in b[2]:
            self.occ[layer][gx2][gy2] = nc
        if soft:
            for layer in a[2]:
                self.fixed[layer][gx1][gy1] = nc
            for layer in b[2]:
                self.fixed[layer][gx2][gy2] = nc

        pq, dist, parent = [], {}, {}
        for layer in a[2]:
            dist[(layer, gx1, gy1)] = 0.0
            heapq.heappush(pq, (abs(gx2 - gx1) + abs(gy2 - gy1), 0.0, layer, gx1, gy1))
        goals = {(layer, gx2, gy2) for layer in b[2]}
        found = None
        steps = ((1, 0), (-1, 0), (0, 1), (0, -1))

        while pq:
            _, g, layer, x, y = heapq.heappop(pq)
            if (layer, x, y) in goals:
                found = (layer, x, y)
                break
            if g > dist.get((layer, x, y), 1e18):
                continue
            for dx, dy in steps:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < self.nx and 0 <= ny < self.ny):
                    continue
                near_goal = abs(nx - gx2) + abs(ny - gy2) <= 3 or abs(nx - gx1) + abs(ny - gy1) <= 3
                use = 0 if near_goal else spread
                if not self._free(layer, nx, ny, nc, use, block):
                    continue
                cost = 1.0
                if (layer == 0 and dy != 0) or (layer == 1 and dx != 0):
                    cost = 2.0
                if soft and not self._free(layer, nx, ny, nc, use):
                    cost += 15.0 * (1 + self.history.get(self.owner[layer][nx][ny], 0))
                ng = g + cost
                key = (layer, nx, ny)
                if ng < dist.get(key, 1e18):
                    dist[key] = ng
                    parent[key] = (layer, x, y)
                    heapq.heappush(pq, (ng + abs(gx2 - nx) + abs(gy2 - ny), ng, layer, nx, ny))
            other = 1 - layer
            if self._via_ok(x, y, nc, soft):
                ng = g + 8.0
                key = (other, x, y)
                if ng < dist.get(key, 1e18):
                    dist[key] = ng
                    parent[key] = (layer, x, y)
                    heapq.heappush(pq, (ng + abs(gx2 - x) + abs(gy2 - y), ng, other, x, y))

        if found is None:
            return None
        path = []
        cur = found
        while cur is not None:
            path.append(cur)
            cur = parent.get(cur)
        path.reverse()
        return path

    def _via_ok(self, x, y, nc, soft=False):
        vias = self.fixed_vias if soft else self.vias
        if vias.get((x, y)) == nc:
            return True
        for cx in range(x - 5, x + 6):
            for cy in range(y - 5, y + 6):
                if (cx, cy) in vias:
                    return False
        grids = self.fixed if soft else None
        for layer in (0, 1):
            if not self._free(layer, x, y, nc, 2, grids):
                return False
        return True

    def _emit(self, net, a, b, path, width):
        cu = self.cu
        layer_of = lambda l: pcbnew.F_Cu if l == 0 else pcbnew.B_Cu
        first_l, fx, fy = path[0]
        start = self.to_mm(fx, fy)
        cu.track(net, [(a[0], a[1]), start], width, layer_of(first_l))
        seg_start = start
        for i in range(1, len(path)):
            l, x, y = path[i]
            pl, px, py = path[i - 1]
            here, prev = self.to_mm(x, y), self.to_mm(px, py)
            if l != pl:
                cu.track(net, [seg_start, prev], width, layer_of(pl))
                if (x, y) not in self.vias:
                    cu.via(net, here[0], here[1])
                    self.vias[(x, y)] = net.GetNetCode()
                seg_start = here
                continue
            turning = i == len(path) - 1
            if not turning:
                nl, nx, ny = path[i + 1]
                turning = nl != l or (x - px, y - py) != (nx - x, ny - y)
            if turning:
                cu.track(net, [seg_start, here], width, layer_of(l))
                seg_start = here
        last_l, lx, ly = path[-1]
        cu.track(net, [self.to_mm(lx, ly), (b[0], b[1])], width, layer_of(last_l))


# Hand-routed nets: completely bypassed by the A* router
HAND_ROUTED_NETS = {
    "/GND", "/+3V3", "/VIN_4V8",
    "/BUCK_SW", "/BUCK_EN", "/BUCK_VFB", "/USB_CC1", "/VBUS_RAW", "/VBUS_SNS", "/VBUS_9V",
    "/PD_V5V", "/PD_ISEL", "/PD_VSEL", "/PD_LED", "/PD_LED_A", "/PD_GATE", "/USB_CC2",
    "/PD_VOUT", "/PD_PWR_EN",
    *(f"/LED_G_SLOT{i}" for i in range(1, 7)),
    *(f"/LED_R_SLOT{i}" for i in range(1, 7)),
    "/BTN_DOWN",
}
for s in range(1, 7):
    HAND_ROUTED_NETS.update({
        f"/SCL_SLOT{s}", f"/SDA_SLOT{s}", f"/VSENSE_SLOT{s}",
        f"/Slot_{s}/TS_NODE", f"/Slot_{s}/VBAT_SLOT", f"/Slot_{s}/VSYS_SLOT"
    })

I2C_DROP = {"/I2C_SDA": (43.1, 9.56), "/I2C_SCL": (42.4, 10.74)}


def route_signals(cu, router, nets):
    # Master I2C pullups to U5
    u5_scl = cu.pad("U5", "22")
    u5_sda = cu.pad("U5", "23")
    cu.track("/I2C_SDA", [u5_sda, (40.5, u5_sda[1]), cu.pad("R7", "2")], 0.18)
    cu.track("/I2C_SCL", [u5_scl, (40.5, u5_scl[1]), cu.pad("R8", "2")], 0.18)

    # OLED header J2 to master I2C
    sda_pin, scl_pin = cu.pad("J2", "4"), cu.pad("J2", "3")
    sda_x, sda_y = I2C_DROP["/I2C_SDA"]
    scl_x, scl_y = I2C_DROP["/I2C_SCL"]
    cu.track("/I2C_SDA", [sda_pin, (sda_pin[0], 7.15), (sda_x, 7.15), (sda_x, sda_y)], 0.18, pcbnew.B_Cu)
    cu.via("/I2C_SDA", sda_x, sda_y)
    cu.track("/I2C_SCL", [scl_pin, (scl_pin[0], 7.75)], 0.18)
    cu.via("/I2C_SCL", scl_pin[0], 7.75)
    cu.track("/I2C_SCL", [(scl_pin[0], 7.75), (scl_x, 7.75), (scl_x, scl_y)], 0.18, pcbnew.B_Cu)
    cu.via("/I2C_SCL", scl_x, scl_y)
    cu.track("/I2C_SDA", [(sda_x, sda_y), cu.pad("R7", "2")], 0.18)
    cu.track("/I2C_SCL", [(scl_x, scl_y), cu.pad("R8", "2")], 0.18)

    router.mark_pending()
    plane_nodes = drop_to_planes(cu, router, nets)
    router.freeze()

    # Plan remaining connections
    connections = {}
    for name in nets:
        if name in HAND_ROUTED_NETS or name.startswith("unconnected-"):
            continue
        nodes = [n for n in nets[name] if n not in plane_nodes]
        if name in I2C_DROP:
            skip_refs = {"U5", "J2", "U7", "R7" if name == "/I2C_SDA" else "R8"}
            nodes = [n for n in nodes if n[0] not in skip_refs]
            px, py = I2C_DROP[name]
            pts = [(px, py, (0, 1))]
        elif name == "/USB_DP":
            skip_refs = {"J1"}
            nodes = [n for n in nodes if n[0] not in skip_refs]
            pts = [(14.3, 9.5, (1,))]
        elif name == "/USB_DN":
            skip_refs = {"J1"}
            nodes = [n for n in nodes if n[0] not in skip_refs]
            pts = [(15.1, 10.15, (0,))]
        elif name == "/+3V3":
            skip_refs = {f"R40{i}" for i in range(1, 7)} | {f"R50{i}" for i in range(1, 7)}
            nodes = [n for n in nodes if n[0] not in skip_refs]
            pts = [(RAIL_EXIT[0], RAIL_EXIT[1], (0,))]
        elif name == "/VIN_4V8":
            skip_refs = {"C6", "C7", "C17", "L1", "R3", "C5"}
            skip_refs |= {f"C10{i}" for i in range(1, 7)} | {f"U10{i}" for i in range(1, 7)}
            skip_refs |= {f"D{i}" for i in range(1, 7)} 
            nodes = [n for n in nodes if n[0] not in skip_refs]
            pts = []
        else:
            pts = []

        for ref, pin in nodes:
            pads = [p for p in cu.fps[ref].Pads() if p.GetNumber() == pin]
            if not pads:
                continue
            x, y = cu.pad(ref, pin)
            through = pads[0].GetDrillSize().x > 0
            if not any(abs(x - q[0]) < 0.05 and abs(y - q[1]) < 0.05 for q in pts):
                pts.append((x, y, (0, 1) if through else (0,)))

        links = []
        if name == "/VIN_4V8":
            for q in sorted(pts, key=lambda r: abs(r[1] - TRUNK_Y)):
                tap_x = round(min(max(q[0], TRUNK_X[0]), TRUNK_X[1]) / 0.1) * 0.1
                links.append(((tap_x, TRUNK_Y, (0,)), q))
        elif name in ("/I2C_SCL", "/I2C_SDA"):
            # Explicit star links for I2C bus from I2C_DROP
            p_drop = pts[0]  # I2C_DROP anchor
            p_u4 = [p for p in pts[1:] if abs(p[0] - 55.25) < 0.1 and p[1] < 20.0]
            p_j4 = [p for p in pts[1:] if p[0] > 60.0 and p[1] > 25.0]
            p_u7 = [p for p in pts[1:] if abs(p[0] - 55.5) < 1.0 and p[1] > 28.0]
            
            if p_u4:
                links.append((p_drop, p_u4[0]))
            if p_j4:
                links.append((p_drop, p_j4[0]))
            if p_u7 and p_u4:
                links.append((p_u4[0], p_u7[0]))
            elif p_u7:
                links.append((p_drop, p_u7[0]))
        elif len(pts) >= 2:
            done, remaining = [pts[0]], pts[1:]
            while remaining:
                c, r = min(((c, r) for c in done for r in remaining),
                           key=lambda pair: abs(pair[0][0] - pair[1][0]) + abs(pair[0][1] - pair[1][1]))
                remaining.remove(r)
                done.append(r)
                links.append((c, r))

        if links:
            width = 0.25 if name == "/+3V3" else (0.18 if "USB" in name or "I2C" in name else 0.20)
            connections[name] = (width, links)

    # Route each connection
    for name in sorted(connections):
        width, links = connections[name]
        for a, b in links:
            if not router.route(name, a, b, width):
                print(f"Negotiating {name}...")
                in_way = router.blockers(name, a, b, width) - {name, "/USB_DP", "/USB_DN"}
                if in_way:
                    router.rip_up(list(in_way) + [name])
                    if not router.route(name, a, b, width):
                        router.failed.append(name)
                        print(f"FAILED to route {name}")
                    for n in in_way:
                        w, lks = connections[n]
                        for la, lb in lks:
                            router.route(n, la, lb, w)
                else:
                    router.failed.append(name)
                    print(f"FAILED to route {name}")


def via_beside(cu, router, net, x, y):
    nc = cu.nets[net].GetNetCode()
    for radius in (0.0, 0.9, 1.05, 1.2, 1.4, 1.6, 2.0, 2.5):
        for k in range(16 if radius else 1):
            ang = 2 * math.pi * k / 16
            gx, gy = router.to_grid(x + radius * math.cos(ang), y + radius * math.sin(ang))
            if radius == 0.0:
                ok = router._via_ok(gx, gy, nc)
            else:
                ok = router._via_ok(gx, gy, nc) and line_clear(router, (x, y), router.to_mm(gx, gy), nc)
            if not ok:
                continue
            mx, my = router.to_mm(gx, gy)
            cu.track(net, [(x, y), (mx, my)], 0.25)
            if (gx, gy) not in router.vias:
                cu.via(net, mx, my)
                router.vias[(gx, gy)] = nc
            router.mark_pending()
            return True
    return False


def drop_to_planes(cu, router, nets):
    done = set()
    done.update({("C16", "1"), ("U7", "21"), ("U7", "23")})
    done.update({(f"D{i}", "3") for i in range(1, 7)})
    done.update({(f"D{i}", "4") for i in range(1, 7)})
    done.update({("R6", "1"), ("R6", "2"), ("C10", "1"), ("C10", "2")})
    done.update({("R9", "1"), ("R9", "2"), ("R18", "1"), ("R18", "2"), ("R10", "1"), ("R10", "2")})
    for net, wanted in (("/+3V3", lambda ref: not ref.startswith(("R40", "R50", "D"))),):
        for ref, pin in nets[net]:
            if (ref, pin) in done:
                continue
            if not wanted(ref):
                continue
            if ref == "U5" and pin == "3":
                x, y = cu.pad(ref, pin)
                cu.track(net, [(x, y), (x - 2.8, y)], 0.18)
                cu.via(net, x - 2.8, y)
                done.add((ref, pin))
                continue
            pads = [p for p in cu.fps[ref].Pads() if p.GetNumber() == pin]
            if not pads:
                continue
            if pads[0].GetDrillSize().x > 0:
                done.add((ref, pin))
                continue
            x, y = cu.pad(ref, pin)
            if via_beside(cu, router, net, x, y):
                done.add((ref, pin))
            else:
                print(f"NOTE: routing {ref}.{pin} to plane")
    return done


def add_ground_vias(cu, router, gnd_pads):
    nc = cu.nets["/GND"].GetNetCode()
    done = 0
    for ref, number in gnd_pads:
        x, y = cu.pad(ref, number)
        placed = False
        for radius in (0.9, 1.05, 1.2, 1.4, 1.6, 2.0, 2.5):
            for k in range(16):
                ang = 2 * math.pi * k / 16
                vx, vy = x + radius * math.cos(ang), y + radius * math.sin(ang)
                gx, gy = router.to_grid(vx, vy)
                if not router._via_ok(gx, gy, nc):
                    continue
                if not line_clear(router, (x, y), router.to_mm(gx, gy), nc):
                    continue
                mx, my = router.to_mm(gx, gy)
                cu.track("/GND", [(x, y), (mx, my)], 0.25)
                if (gx, gy) not in router.vias:
                    cu.via("/GND", mx, my)
                router.mark_pending()
                router.vias[(gx, gy)] = nc
                placed = True
                done += 1
                break
            if placed:
                break
    return done


def line_clear(router, a, b, nc):
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(1, int(dist / 0.1))
    for s in range(1, steps):
        t = s / steps
        gx, gy = router.to_grid(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        cell = router.occ[0][gx][gy]
        if cell not in (0, nc) and s * 0.1 > 0.6:
            return False
    return True


def add_stitching(cu, router):
    nc = cu.nets["/GND"].GetNetCode()
    count = 0
    for gx in range(40, router.nx - 40, 40):
        for gy in range(40, router.ny - 40, 40):
            if router._via_ok(gx, gy, nc) and router.occ[0][gx][gy] in (0, nc) and router.occ[1][gx][gy] in (0, nc):
                mx, my = router.to_mm(gx, gy)
                if mx > 50 and my < 8:
                    continue
                cu.via("/GND", mx, my)
                router.mark_pending()
                count += 1
    return count


def add_zone(board, net_item, layer, corners, priority=0, solid_pads=True):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net_item)
    z.SetLocalClearance(pcbnew.FromMM(0.20))
    z.SetMinThickness(pcbnew.FromMM(0.20))
    z.SetAssignedPriority(priority)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if solid_pads else pcbnew.ZONE_CONNECTION_THT_THERMAL)
    poly = z.Outline()
    idx = poly.NewOutline()
    for x, y in corners:
        poly.Append(pcbnew.FromMM(x + X0), pcbnew.FromMM(y + Y0), idx)
    board.Add(z)
    return z


def build_board():
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(4)

    ds = board.GetDesignSettings()
    ds.m_TrackMinWidth = pcbnew.FromMM(0.15)
    ds.m_ViasMinSize = pcbnew.FromMM(0.55)
    ds.m_ViasMinDrill = pcbnew.FromMM(0.25)
    ds.m_MinClearance = pcbnew.FromMM(0.15)
    ds.m_CopperEdgeClearance = pcbnew.FromMM(0.3)
    ds.m_HoleClearance = pcbnew.FromMM(0.18)
    ds.m_HoleToHoleMin = pcbnew.FromMM(0.20)
    ds.m_MinThroughDrill = pcbnew.FromMM(0.25)
    ds.m_SolderMaskMinWidth = pcbnew.FromMM(0.0)
    ds.m_SilkClearance = pcbnew.FromMM(0.0)

    root = ET.parse(os.path.join(PROJECT_DIR, "netlist.xml")).getroot()

    net_map = {}
    net_nodes = {}
    for n in root.findall(".//net"):
        name = n.attrib["name"]
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        net_map[name] = ni
        net_nodes[name] = [(nd.attrib["ref"], nd.attrib["pin"]) for nd in n.findall("node")]

    pad_net = {node: net_map[name] for name, nodes in net_nodes.items() for node in nodes}

    placement = get_placement()
    fp_objs = {}
    for comp in root.findall(".//comp"):
        ref = comp.attrib["ref"]
        val = comp.find("value").text
        lib, name = comp.find("footprint").text.split(":")
        lib_path = FP_CUSTOM_ROOT if lib == "lipo_charger_custom" else os.path.join(FP_STD_ROOT, lib + ".pretty")

        fp = pcbnew.FootprintLoad(lib_path, name)
        fp.SetReference(ref)
        fp.SetValue(val)
        fp.Value().SetVisible(False)
        fp.Reference().SetVisible(True)

        for item in list(fp.GraphicalItems()):
            if item.GetLayer() == pcbnew.F_SilkS:
                item.SetLayer(pcbnew.F_Fab)

        if ref.startswith("J10"):
            model = pcbnew.FP_3DMODEL()
            model.m_Filename = "${KIPRJMOD}/3dmodels/530470210.stp"
            model.m_Offset = pcbnew.VECTOR3D(0.625, -1.15, 5.2)
            model.m_Rotation = pcbnew.VECTOR3D(90.0, 0.0, 0.0)
            fp.Models().clear()
            fp.Models().push_back(model)

        x_mm, y_mm, rot_deg = placement[ref]
        fp.SetPosition(pcbnew.VECTOR2I_MM(x_mm, y_mm))
        fp.SetOrientationDegrees(rot_deg)

        ref_txt = fp.Reference()
        ref_txt.SetTextSize(pcbnew.VECTOR2I_MM(0.7, 0.7))
        ref_txt.SetTextThickness(pcbnew.FromMM(0.12))
        ref_txt.SetLayer(pcbnew.F_Fab)

        for pad in fp.Pads():
            key = (ref, pad.GetNumber())
            if key in pad_net:
                pad.SetNet(pad_net[key])
            if ref == "U4" and pad.GetNumber() == "19" and pad.GetDrillSize().x > 0:
                pad.SetDrillSize(pcbnew.VECTOR2I_MM(0.30, 0.30))
                pad.SetSize(pad.GetPrincipalLayer(), pcbnew.VECTOR2I_MM(0.55, 0.55))

        board.Add(fp)
        fp_objs[ref] = fp

    # Mounting holes
    mh_lib = os.path.join(FP_STD_ROOT, "MountingHole.pretty")
    holes = [
        ("H1", "MountingHole_3.2mm_M3_Pad", 3.8, 3.8, 3.2, 5.6),
        ("H2", "MountingHole_3.2mm_M3_Pad", 71.2, 27.5, 3.2, 5.6),
        ("H3", "MountingHole_3.2mm_M3_Pad", 3.8, BOARD_H - 3.8, 3.2, 5.6),
        ("H4", "MountingHole_3.2mm_M3_Pad", 71.2, BOARD_H - 3.8, 3.2, 5.6),
        ("H5", "MountingHole_2.2mm_M2_Pad", 24.3, 6.5, 2.2, 4.0),
        ("H6", "MountingHole_2.2mm_M2_Pad", 47.3, 6.5, 2.2, 4.0),
        ("H7", "MountingHole_2.2mm_M2_Pad", 24.3, 30.0, 2.2, 4.0),
        ("H8", "MountingHole_2.2mm_M2_Pad", 47.3, 30.0, 2.2, 4.0),
    ]
    for ref, name, mx, my, drill, pad_d in holes:
        fp = pcbnew.FootprintLoad(mh_lib, name)
        fp.SetReference(ref)
        fp.SetValue(name)
        fp.Value().SetVisible(False)
        fp.Reference().SetVisible(False)
        fp.SetPosition(pcbnew.VECTOR2I_MM(mx + X0, my + Y0))
        for p in fp.Pads():
            p.SetNetCode(0)
            p.SetDrillSize(pcbnew.VECTOR2I_MM(drill, drill))
            p.SetSize(p.GetPrincipalLayer(), pcbnew.VECTOR2I_MM(pad_d, pad_d))
        board.Add(fp)
        fp_objs[ref] = fp

    cu = Copper(board, net_map, fp_objs)
    print("Routing power trunk and buck...")
    route_power(cu)

    print("Routing charger cells and lanes...")
    route_charger_cells(cu)
    route_upper_lanes(cu)

    print("Routing AW9523B LED driver and dual-color LEDs...")
    route_aw9523_and_leds(cu)

    router = Router(cu, pitch=0.2)
    print("Routing remaining control signals...")
    route_signals(cu, router, net_nodes)

    # Additional ground pads
    handled_refs = {"U1", "U2", "C1", "C2", "C3", "C4", "C6", "C7", "C16", "C17", "R4", "U7", "U3", "C8", "C9", "C18", "SW1", "SW2", "SW3", "R6", "C10", "R9", "R18", "R10"}
    handled_refs |= {f"C10{i}" for i in range(1, 7)} | {f"C30{i}" for i in range(1, 7)}
    handled_refs |= {f"C20{i}" for i in range(1, 7)} | {f"R30{i}" for i in range(1, 7)}
    handled_refs |= {f"R10{i}" for i in range(1, 7)} | {f"U10{i}" for i in range(1, 7)}
    handled_refs |= {f"J10{i}" for i in range(1, 7)} | {f"D{i}" for i in range(1, 7)} 
    gnd_pads = []
    seen = set()
    for ref, pin in net_nodes["/GND"]:
        if ref in handled_refs or (ref == "U6" and pin in ("6", "7", "8")):
            continue
        pads = [p for p in fp_objs[ref].Pads() if p.GetNumber() == pin]
        if not pads or pads[0].GetDrillSize().x > 0:
            continue
        where = cu.pad(ref, pin)
        if where not in seen:
            seen.add(where)
            gnd_pads.append((ref, pin))
    n_gnd = add_ground_vias(cu, router, gnd_pads)
    n_stitch = add_stitching(cu, router)
    print(f"Ground vias placed: {n_gnd}, Stitching vias: {n_stitch}, Failed routes: {len(router.failed)}")

    # Internal Planes
    m = 0.5
    right, bottom = BOARD_W - m, BOARD_H - m
    gnd_plane = [(m, m), (ANTENNA_KEEPOUT[0], m), (ANTENNA_KEEPOUT[0], ANTENNA_KEEPOUT[1]),
                 (right, ANTENNA_KEEPOUT[1]), (right, bottom), (m, bottom)]
    add_zone(board, net_map["/GND"], pcbnew.In1_Cu, gnd_plane, priority=0, solid_pads=False)

    plane_3v3 = [(m, m), (ANTENNA_KEEPOUT[0], m), (ANTENNA_KEEPOUT[0], ANTENNA_KEEPOUT[1]),
                 (right, ANTENNA_KEEPOUT[1]), (right, PLANE_SPLIT[0]), (m, PLANE_SPLIT[0])]
    add_zone(board, net_map["/+3V3"], pcbnew.In2_Cu, plane_3v3, priority=0, solid_pads=False)

    plane_vin = [(m, PLANE_SPLIT[1]), (right, PLANE_SPLIT[1]), (right, bottom), (m, bottom)]
    add_zone(board, net_map["/VIN_4V8"], pcbnew.In2_Cu, plane_vin, priority=0, solid_pads=False)

    # Outer Ground Pours
    gnd = net_map["/GND"]
    outline = [(m, m), (BOARD_W - m, m), (BOARD_W - m, BOARD_H - m), (m, BOARD_H - m)]
    add_zone(board, gnd, pcbnew.F_Cu, outline, priority=0, solid_pads=True)
    add_zone(board, gnd, pcbnew.B_Cu, outline, priority=0, solid_pads=False)
    for net, x1, y1, x2, y2, prio in POWER_ZONES:
        add_zone(board, net_map[net], pcbnew.F_Cu, [(x1, y1), (x2, y1), (x2, y2), (x1, y2)], priority=prio)

    print("Filling copper zones...")
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    # Board outline (Edge_Cuts)
    corners = [(X0, Y0), (X0 + BOARD_W, Y0), (X0 + BOARD_W, Y0 + BOARD_H), (X0, Y0 + BOARD_H)]
    for i in range(4):
        (x1, y1), (x2, y2) = corners[i], corners[(i + 1) % 4]
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetStart(pcbnew.VECTOR2I_MM(x1, y1))
        seg.SetEnd(pcbnew.VECTOR2I_MM(x2, y2))
        seg.SetWidth(pcbnew.FromMM(0.15))
        board.Add(seg)

    print("Adding silkscreen labels...")
    add_silkscreen(board, net_map)

    pcb_path = os.path.join(PROJECT_DIR, "lipo-6slot-charger.kicad_pcb")
    pcbnew.SaveBoard(pcb_path, board)
    print("SUCCESS: Saved 6-slot 4-layer routed PCB to:", pcb_path)


def add_silkscreen(board, net_map):
    def line(x1, y1, x2, y2, layer=pcbnew.F_SilkS, width=0.15):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetLayer(layer)
        s.SetStart(pcbnew.VECTOR2I_MM(x1 + X0, y1 + Y0))
        s.SetEnd(pcbnew.VECTOR2I_MM(x2 + X0, y2 + Y0))
        s.SetWidth(pcbnew.FromMM(width))
        board.Add(s)

    def text(txt, x, y, size=1.0, thick=0.16, layer=pcbnew.F_SilkS, mirrored=False):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(txt)
        t.SetLayer(layer)
        t.SetPosition(pcbnew.VECTOR2I_MM(x + X0, y + Y0))
        t.SetTextSize(pcbnew.VECTOR2I_MM(size, size))
        t.SetTextThickness(pcbnew.FromMM(thick))
        t.SetMirrored(mirrored)
        board.Add(t)

    text("6-SLOT 1S CHARGER", 36.0, 2.0, size=0.9, thick=0.15)

    # OLED outline and pins
    line(26.8, 4.4, 44.8, 4.4)
    line(31.6, 32.2, 44.8, 32.2)
    line(49.5, 9.0, 49.5, 27.5)
    line(22.1, 9.0, 22.1, 27.5)
    text('0.96" OLED', 44.0, 25.0, size=0.9, thick=0.15)
    text("GND VCC SCL SDA", 34.0, 8.2, size=0.8, thick=0.15)

    # Programming headers and buttons
    text("TX RX GND", 60.5, 29.2, size=0.8, thick=0.15)
    text("RST GND BOOT", 65.5, 29.2, size=0.8, thick=0.15)
    text("UP", 47.0, 36.5, size=0.8, thick=0.15)
    text("SEL", 57.5, 36.5, size=0.8, thick=0.15)
    text("DOWN", 68.0, 36.5, size=0.8, thick=0.15)
    text("PD", 18.8, 21.0, size=0.8, thick=0.13)

    # Channel labels and connector polarities
    for row, y in enumerate(ROW_Y):
        text(f"CH{row + 1}", LEFT_LED_X, y - 3.7, size=0.8, thick=0.13)
        text(f"CH{row + 4}", RIGHT_LED_X, y - 3.7, size=0.8, thick=0.13)
        text("+", LEFT_PB_X - 0.1, y - 2.9, size=0.8, thick=0.14)
        text("-", LEFT_PB_X + 1.35, y - 2.9, size=0.8, thick=0.14)
        text("+", RIGHT_PB_X - 0.1, y - 2.9, size=0.8, thick=0.14)
        text("-", RIGHT_PB_X + 1.35, y - 2.9, size=0.8, thick=0.14)


if __name__ == "__main__":
    build_board()
