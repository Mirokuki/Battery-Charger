#!/usr/bin/env python3
"""
Board generator for the 6-slot 1S/2S USB-C LiPo charger.

Run with KiCad's bundled Python:
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" gen_pcb.py [--place-only]

Stack-up (4 layers, 108 x 68 mm):
    F.Cu   parts, local routing, GND pour
    In1.Cu solid GND
    In2.Cu VBUS_PD (slot band + PD/buck corner) and +3V3 (logic area)
    B.Cu   long signal runs, GND pour
"""

import heapq
import math
import os
import sys

import numpy as np
import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FP_ROOT = r"C:\Program Files\KiCad\10.0\share\kicad\footprints"
OUT = os.path.join(HERE, "lipo-charger.kicad_pcb")

X0, Y0 = 50.0, 40.0            # board origin on the KiCad sheet
BOARD_W, BOARD_H = 108.0, 68.0
CELL_W, CELL_X0 = 17.6, 1.2    # slot pitch and left margin
IC_X, IC_Y = 9.0, 25.0         # charger IC centre inside a slot cell
ANT = (0.0, 62.0, 27.2, 68.0)  # ESP32 antenna keep-out (x1, y1, x2, y2), from the module footprint
VBUS_POLY = [(0.5, 0.5), (107.5, 0.5), (107.5, 30.3), (32.0, 30.3), (32.0, 46.3), (0.5, 46.3)]
V33_POLY = [(32.6, 30.9), (107.5, 30.9), (107.5, 67.5), (ANT[2], 67.5), (ANT[2], ANT[1]),
            (0.5, ANT[1]), (0.5, 46.9), (32.6, 46.9)]
GND_POLY = [(0.5, 0.5), (107.5, 0.5), (107.5, 67.5), (ANT[2], 67.5), (ANT[2], ANT[1]), (0.5, ANT[1])]

PITCH = 0.1
CLR = 0.15                     # router clearance; board rule is 0.127
VIA_D, VIA_DRILL = 0.6, 0.3
W_SIG, W_PWR = 0.15, 0.4

PLANE_NETS = {"GND": None, "VBUS_PD": VBUS_POLY, "+3V3": V33_POLY}
NET_WIDTH = {"VBUS_RAW": 0.8, "VBUS_SNS": 0.8, "BUCK_SW": 0.5, "PD_VOUT": 0.2, "PD_GATE": 0.2}
for _s in range(1, design.SLOTS + 1):
    for _n in ("BAT", "PB_P", "PB_S", "XT_P", "XT_S"):
        NET_WIDTH[f"S{_s}_{_n}"] = W_PWR
    for _n in ("SW1", "SW2", "PMID", "SYS", "REGN"):
        NET_WIDTH[f"S{_s}_{_n}"] = 0.2


# ------------------------------------------------------------------ placement
def cell_origin(s):
    return CELL_X0 + CELL_W * (s - 1)


SLOT_REL = {  # ref pattern -> (dx, dy, rot) relative to the charger IC centre
    "U{s}01": (0, 0, 0),
    "L{s}01": (0, -6.8, 0),
    "C{s}09": (-2.2, -3.5, 90), "C{s}07": (-3.75, -4.0, 90), "C{s}08": (-5.85, -4.0, 90),
    "C{s}13": (2.2, -3.5, 90), "C{s}10": (3.75, -4.0, 90), "C{s}11": (5.85, -4.0, 90),
    "C{s}06": (-4.0, -1.3, 90), "C{s}05": (-6.3, -0.82, 180),
    "C{s}02": (-5.0, 1.0, 180),
    "C{s}01": (-3.35, -6.6, 0), "C{s}03": (3.35, -6.6, 180),
    "C{s}04": (3.9, -1.75, 0),
    "C{s}14": (6.65, -0.6, 0), "C{s}15": (6.65, 1.5, 0),
    "R{s}05": (5.4, 3.55, 90), "R{s}04": (3.9, 1.0, 0),
    "R{s}03": (3.9, 2.6, 0), "R{s}02": (3.9, 3.65, 180),
    "R{s}01": (-3.9, 3.0, 180), "D{s}01": (-6.4, 3.0, 0),
    # XT30 path (left), PicoBlade path (right)
    "Q{s}03": (-6.6, -15.85, 90), "Q{s}04": (-3.1, -15.85, 270),
    "Q{s}01": (3.1, -15.85, 90), "Q{s}02": (6.6, -15.85, 270),
    "R{s}09": (-6.5, -13.2, 0), "C{s}17": (-4.4, -13.2, 180), "R{s}10": (-2.3, -13.2, 0),
    "R{s}07": (2.3, -13.2, 0), "C{s}16": (4.4, -13.2, 0), "R{s}06": (6.5, -13.2, 180),
    "Q{s}06": (-6.0, -10.9, 0), "R{s}11": (-3.0, -10.9, 0),
    "Q{s}05": (6.0, -10.9, 180), "R{s}08": (3.0, -10.9, 180),
    "J{s}01": (-0.4, -21.0, 180), "J{s}02": (5.2, -20.5, 0),
}

CORE = {
    "J1": (3.4, 38.0, 270),
    "R1": (10.0, 36.2, 0), "R2": (10.0, 40.4, 0),
    "U1": (14.5, 38.0, 180),
    "R6": (13.4, 34.6, 90), "R7": (12.2, 34.6, 90), "C3": (11.6, 41.6, 0),
    "R8": (15.6, 34.9, 90), "D1": (15.6, 32.3, 90),
    "C2": (18.4, 38.4, 0), "R4": (18.2, 40.0, 0), "R5": (18.2, 41.2, 0),
    "R3": (19.6, 34.0, 0), "Q1": (24.0, 34.0, 180), "C1": (24.0, 36.6, 0),
    "C4": (10.2, 32.6, 0), "C5": (27.4, 33.4, 90), "C6": (29.6, 33.4, 90),
    "U4": (9.6, 45.4, 0),
    "U2": (25.4, 42.6, 0), "C7": (21.5, 43.6, 180), "L1": (30.2, 42.6, 0), "C8": (26.8, 40.0, 0),
    "C9": (34.6, 41.4, 0), "C10": (34.6, 44.0, 0),
    "U3": (13.0, 55.15, 180),
    "C12": (26.2, 60.9, 0), "C11": (25.6, 55.6, 90), "R9": (26.4, 59.65, 180), "C13": (26.4, 58.3, 0),
    "R10": (1.9, 57.3, 90), "R11": (1.9, 60.6, 90),
    "U5": (33.5, 56.0, 0), "C14": (39.2, 51.6, 0), "R12": (41.2, 53.6, 0), "R13": (41.2, 55.6, 0),
    "R18": (27.6, 56.6, 0),
    "J2": (50.19, 36.0, 90),
    "SW1": (76.0, 39.0, 90), "SW2": (76.0, 49.5, 90), "SW3": (76.0, 60.0, 90),
    "R14": (81.0, 39.0, 90), "R15": (81.0, 49.5, 90), "R16": (81.0, 60.0, 90),
    "BZ1": (93.0, 39.6, 0), "Q2": (87.0, 47.0, 0), "D2": (91.0, 47.0, 0), "R17": (84.0, 47.0, 0),
    "J3": (88.0, 56.0, 90),
}
HOLES = {"H1": (37.3, 34.6), "H2": (105.0, 32.6), "H3": (31.0, 65.0), "H4": (105.0, 65.0)}


def placement():
    pos = dict(CORE)
    for s in range(1, design.SLOTS + 1):
        cx, cy = cell_origin(s) + IC_X, IC_Y
        for pat, (dx, dy, rot) in SLOT_REL.items():
            pos[pat.format(s=s)] = (round(cx + dx, 4), round(cy + dy, 4), rot)
        for k, ref in enumerate((f"R{s}12", f"R{s}13")):
            pos[ref] = (27.6 + 1.5 * (2 * (s - 1) + k), 47.4, 90)
    return pos


# ------------------------------------------------------------------ helpers
def mm(v):
    return pcbnew.FromMM(float(v))


def pt(x, y):
    return pcbnew.VECTOR2I(mm(x + X0), mm(y + Y0))


class Board:
    def __init__(self):
        self.b = pcbnew.CreateEmptyBoard()
        self.b.SetCopperLayerCount(4)
        ds = self.b.GetDesignSettings()
        ds.m_TrackMinWidth = mm(0.127)
        ds.m_ViasMinSize = mm(0.5)
        ds.m_MinThroughDrill = mm(0.2)
        ds.m_MinClearance = mm(0.127)
        ds.m_CopperEdgeClearance = mm(0.3)
        ds.m_HoleClearance = mm(0.18)
        ds.m_HoleToHoleMin = mm(0.25)
        self.nets = {}
        self.fps = {}
        self.pad_net = {}
        for name, nodes in design.nets().items():
            ni = pcbnew.NETINFO_ITEM(self.b, name)
            self.b.Add(ni)
            self.nets[name] = ni
            for node in nodes:
                self.pad_net[node] = name
        nl = os.path.join(HERE, "sch_netlist.xml")
        if os.path.exists(nl):
            import xml.etree.ElementTree as ET
            for net in ET.parse(nl).getroot().find("nets"):
                name = net.get("name")
                if name.startswith("unconnected-"):
                    ni = pcbnew.NETINFO_ITEM(self.b, name)
                    self.b.Add(ni)
                    self.nets[name] = ni
                    for node in net:
                        self.pad_net[(node.get("ref"), node.get("pin"))] = name
        self.code_name = {}

    def load(self, pos):
        for ref, p in design.PARTS.items():
            lib, name = p["fp"].split(":")
            fp = pcbnew.FootprintLoad(os.path.join(FP_ROOT, lib + ".pretty"), name)
            fp.SetFPIDAsString(p["fp"])
            fp.SetReference(ref)
            fp.SetValue(p["value"])
            fp.Value().SetVisible(False)
            fp.Reference().SetLayer(pcbnew.F_Fab)
            fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.5), mm(0.5)))
            fp.Reference().SetTextThickness(mm(0.08))
            if p["dnp"]:
                fp.SetDNP(True)
            for item in fp.GraphicalItems():        # library polarity texts collide with the next connector
                if item.GetLayer() == pcbnew.F_SilkS and isinstance(item, pcbnew.PCB_TEXT):
                    item.SetLayer(pcbnew.F_Fab)
            if ref == "U3":
                for item in list(fp.GraphicalItems()):
                    if item.GetLayer() == pcbnew.F_CrtYd:
                        fp.Remove(item)
                box = pcbnew.PCB_SHAPE(fp)
                box.SetShape(pcbnew.SHAPE_T_RECT)
                box.SetLayer(pcbnew.F_CrtYd)
                box.SetStart(pcbnew.VECTOR2I(mm(-9.25), mm(-12.8)))
                box.SetEnd(pcbnew.VECTOR2I(mm(9.25), mm(7.4)))
                box.SetWidth(mm(0.05))
                fp.Add(box)
            x, y, rot = pos[ref]
            fp.SetPosition(pt(x, y))
            fp.SetOrientationDegrees(rot)
            for pad in fp.Pads():
                net = self.pad_net.get((ref, pad.GetNumber()))
                if net:
                    pad.SetNet(self.nets[net])
            self.b.Add(fp)
            self.fps[ref] = fp
        lib = os.path.join(FP_ROOT, "MountingHole.pretty")
        for ref, (x, y) in HOLES.items():
            fp = pcbnew.FootprintLoad(lib, "MountingHole_2.7mm_M2.5")
            fp.SetReference(ref)
            fp.SetBoardOnly(True)
            fp.Reference().SetVisible(False)
            fp.Value().SetVisible(False)
            fp.SetPosition(pt(x, y))
            self.b.Add(fp)
            self.fps[ref] = fp
        self.code_name = {ni.GetNetCode(): n for n, ni in self.nets.items()}

    def pad_xy(self, ref, num):
        for p in self.fps[ref].Pads():
            if p.GetNumber() == str(num):
                c = p.GetPosition()
                return (round(pcbnew.ToMM(c.x) - X0, 4), round(pcbnew.ToMM(c.y) - Y0, 4))
        raise KeyError((ref, num))

    def track(self, net, a, b, w, layer):
        if abs(a[0] - b[0]) < 1e-5 and abs(a[1] - b[1]) < 1e-5:
            return None
        t = pcbnew.PCB_TRACK(self.b)
        t.SetStart(pt(*a))
        t.SetEnd(pt(*b))
        t.SetWidth(mm(w))
        t.SetLayer(pcbnew.F_Cu if layer == 0 else pcbnew.B_Cu)
        t.SetNet(self.nets[net])
        self.b.Add(t)
        return t

    def via(self, net, x, y, d=VIA_D, drill=VIA_DRILL):
        v = pcbnew.PCB_VIA(self.b)
        v.SetPosition(pt(x, y))
        v.SetWidth(pcbnew.F_Cu, mm(d)) if _via_width_takes_layer() else v.SetWidth(mm(d))
        v.SetDrill(mm(drill))
        v.SetNet(self.nets[net])
        self.b.Add(v)
        return v

    def zone(self, net, layer, corners, priority=0, thermal=False, clearance=0.2):
        z = pcbnew.ZONE(self.b)
        z.SetLayer(layer)
        z.SetNet(self.nets[net])
        z.SetLocalClearance(mm(clearance))
        z.SetMinThickness(mm(0.2))
        z.SetAssignedPriority(priority)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL if thermal else pcbnew.ZONE_CONNECTION_FULL)
        z.SetThermalReliefGap(mm(0.25))
        z.SetThermalReliefSpokeWidth(mm(0.4))
        poly = z.Outline()
        i = poly.NewOutline()
        for x, y in corners:
            poly.Append(mm(x + X0), mm(y + Y0), i)
        self.b.Add(z)
        return z


_VW = None


def _via_width_takes_layer():
    global _VW
    if _VW is None:
        try:
            v = pcbnew.PCB_VIA(pcbnew.CreateEmptyBoard())
            v.SetWidth(pcbnew.F_Cu, 1000)
            _VW = True
        except Exception:
            _VW = False
    return _VW


def courtyard_boxes(board):
    out = {}
    for ref, fp in board.fps.items():
        fp.BuildCourtyardCaches()
        cy = fp.GetCourtyard(pcbnew.F_CrtYd)
        if cy.OutlineCount() == 0:
            continue
        bb = cy.BBox()
        out[ref] = (pcbnew.ToMM(bb.GetLeft()) - X0, pcbnew.ToMM(bb.GetTop()) - Y0,
                    pcbnew.ToMM(bb.GetRight()) - X0, pcbnew.ToMM(bb.GetBottom()) - Y0)
    return out


def check_overlaps(board):
    boxes = courtyard_boxes(board)
    refs = sorted(boxes)
    bad = 0
    for i, a in enumerate(refs):
        ax1, ay1, ax2, ay2 = boxes[a]
        if ax1 < 0.0 or ay1 < 0.0 or ax2 > BOARD_W or ay2 > BOARD_H:
            if a not in ("J1",):
                print(f"  off-board courtyard: {a} {boxes[a]}")
        for b in refs[i + 1:]:
            bx1, by1, bx2, by2 = boxes[b]
            ox = min(ax2, bx2) - max(ax1, bx1)
            oy = min(ay2, by2) - max(ay1, by1)
            if ox > 0.01 and oy > 0.01:
                bad += 1
                print(f"  courtyard overlap {a} / {b}: {ox:.2f} x {oy:.2f}")
    print(f"courtyard overlaps: {bad}")
    return bad


# ------------------------------------------------------------------ router
def dilate(mask, r):
    if r <= 0:
        return mask
    out = mask.copy()
    for k in range(1, r + 1):
        out[k:, :] |= mask[:-k, :]
        out[:-k, :] |= mask[k:, :]
    m2 = out.copy()
    for k in range(1, r + 1):
        out[:, k:] |= m2[:, :-k]
        out[:, :-k] |= m2[:, k:]
    return out


def in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


class Router:
    def __init__(self, board):
        self.bd = board
        self.nx = int(round(BOARD_W / PITCH)) + 1
        self.ny = int(round(BOARD_H / PITCH)) + 1
        self.occ = np.zeros((2, self.nx, self.ny), np.int32)
        self.novia = np.zeros((self.nx, self.ny), bool)
        self.tree = {}      # net -> list of sets (connected copper groups, as grid cells)
        self.failed = []
        self.items = []     # every track / via created, in order (for rip-up)
        self.via_at = {}
        self._edges()
        self._pads()

    # --- rasterising
    def _rng(self, lo, hi, n):
        a = max(0, int(math.ceil(lo / PITCH - 1e-6)))
        b = min(n - 1, int(math.floor(hi / PITCH + 1e-6)))
        return a, b

    def mark_rect(self, layer, x1, y1, x2, y2, code, grow=CLR):
        a, b = self._rng(min(x1, x2) - grow, max(x1, x2) + grow, self.nx)
        c, d = self._rng(min(y1, y2) - grow, max(y1, y2) + grow, self.ny)
        if a > b or c > d:
            return
        reg = self.occ[layer, a:b + 1, c:d + 1]
        clash = (reg != 0) & (reg != code)
        reg[reg == 0] = code
        reg[clash] = -1

    def mark_seg(self, layer, a, b, w, code):
        h = w / 2.0
        if abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6:
            self.mark_rect(layer, a[0] - h, a[1] - h, b[0] + h if b[0] >= a[0] else b[0] - h,
                           b[1] + h if b[1] >= a[1] else b[1] - h, code)
            self.mark_rect(layer, min(a[0], b[0]) - h, min(a[1], b[1]) - h,
                           max(a[0], b[0]) + h, max(a[1], b[1]) + h, code)
            return
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (PITCH / 2)) + 1)
        for i in range(n + 1):
            t = i / n
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            self.mark_rect(layer, x - h, y - h, x + h, y + h, code)

    def _edges(self):
        e = int(round(0.45 / PITCH))
        self.occ[:, :e + 1, :] = -1
        self.occ[:, -e - 1:, :] = -1
        self.occ[:, :, :e + 1] = -1
        self.occ[:, :, -e - 1:] = -1
        for layer in (0, 1):
            self.mark_rect(layer, ANT[0], ANT[1], ANT[2], ANT[3], -1, grow=0.2)

    def cells_in(self, x1, y1, x2, y2, layers, shrink=0.03):
        a, b = self._rng(x1 + shrink, x2 - shrink, self.nx)
        c, d = self._rng(y1 + shrink, y2 - shrink, self.ny)
        return {(l, i, j) for l in layers for i in range(a, b + 1) for j in range(c, d + 1)}

    def _pads(self):
        self.pad_cells = {}
        for ref, fp in self.bd.fps.items():
            for pad in fp.Pads():
                ls = pad.GetLayerSet()
                on_f, on_b = ls.Contains(pcbnew.F_Cu), ls.Contains(pcbnew.B_Cu)
                hole = pcbnew.ToMM(pad.GetDrillSize().x)
                if not (on_f or on_b or hole > 0):
                    continue
                c = pad.GetPosition()
                x, y = pcbnew.ToMM(c.x) - X0, pcbnew.ToMM(c.y) - Y0
                sz = pad.GetSize(pcbnew.F_Cu)
                sx, sy = pcbnew.ToMM(sz.x), pcbnew.ToMM(sz.y)
                if abs(pad.GetOrientationDegrees() % 180 - 90) < 10:
                    sx, sy = sy, sx
                code = pad.GetNetCode() or -1
                if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH or not (on_f or on_b):
                    for layer in (0, 1):
                        self.mark_rect(layer, x - hole / 2, y - hole / 2, x + hole / 2, y + hole / 2, -1, grow=0.3)
                    continue
                layers = [l for l, on in ((0, on_f), (1, on_b)) if on or hole > 0]
                for layer in layers:
                    self.mark_rect(layer, x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2, code)
                if hole == 0:
                    a, b = self._rng(x - sx / 2 - 0.36, x + sx / 2 + 0.36, self.nx)
                    cc, d = self._rng(y - sy / 2 - 0.36, y + sy / 2 + 0.36, self.ny)
                    self.novia[a:b + 1, cc:d + 1] = True
                if code > 0:
                    cells = self.cells_in(x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2, layers)
                    if pad.GetShape(pcbnew.F_Cu) in (pcbnew.PAD_SHAPE_CIRCLE, pcbnew.PAD_SHAPE_OVAL):
                        r = min(sx, sy) / 2 - 0.05          # stay inside the round copper, not its bounding box
                        ex, ey = max(0.0, sx / 2 - min(sx, sy) / 2), max(0.0, sy / 2 - min(sx, sy) / 2)
                        cells = {c for c in cells
                                 if math.hypot(max(0.0, abs(c[1] * PITCH - x) - ex),
                                               max(0.0, abs(c[2] * PITCH - y) - ey)) <= r}
                    if not cells:
                        g = (int(round(x / PITCH)), int(round(y / PITCH)))
                        cells = {(l, g[0], g[1]) for l in layers}
                    key = (ref, pad.GetNumber())
                    if key not in self.bd.pad_net:
                        continue
                    self.pad_cells.setdefault(key, set()).update(cells)
                    self.tree.setdefault(self.bd.pad_net[key], []).append(set(cells))

    # --- copper bookkeeping
    def _merge(self, net):
        groups = self.tree.get(net, [])
        merged = True
        while merged:
            merged = False
            for i in range(len(groups)):
                for j in range(i + 1, len(groups)):
                    if groups[i] & groups[j]:
                        groups[i] |= groups.pop(j)
                        merged = True
                        break
                if merged:
                    break

    def add_track(self, net, a, b, w, layer=0):
        t = self.bd.track(net, a, b, w, layer)
        if t is None:
            return
        self.items.append(t)
        code = self.bd.nets[net].GetNetCode()
        self.mark_seg(layer, a, b, w, code)
        cells = set()
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (PITCH / 2)))
        r = max(0.0, w / 2 - 0.03)
        for i in range(n + 1):
            t = i / n
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            for gx in range(int(math.ceil((x - r) / PITCH - 1e-6)), int(math.floor((x + r) / PITCH + 1e-6)) + 1):
                for gy in range(int(math.ceil((y - r) / PITCH - 1e-6)), int(math.floor((y + r) / PITCH + 1e-6)) + 1):
                    if 0 <= gx < self.nx and 0 <= gy < self.ny:
                        cells.add((layer, gx, gy))
        if not cells:
            g = (int(round(a[0] / PITCH)), int(round(a[1] / PITCH)))
            cells.add((layer, g[0], g[1]))
        self.tree.setdefault(net, []).append(cells)

    def add_via(self, net, x, y):
        g0 = (int(round(x / PITCH)), int(round(y / PITCH)))
        if self.via_at.get(g0) == net:
            return
        self.via_at[g0] = net
        self.items.append(self.bd.via(net, x, y))
        code = self.bd.nets[net].GetNetCode()
        r = VIA_D / 2
        for layer in (0, 1):
            self.mark_rect(layer, x - r, y - r, x + r, y + r, code)
        g = (int(round(x / PITCH)), int(round(y / PITCH)))
        self.tree.setdefault(net, []).append({(0, g[0], g[1]), (1, g[0], g[1])})

    def snapshot(self):
        return (self.occ.copy(), {n: [set(c) for c in gs] for n, gs in self.tree.items()},
                len(self.items), list(self.failed), dict(self.via_at))

    def restore(self, snap):
        occ, tree, n, failed, via_at = snap
        self.via_at = dict(via_at)
        self.occ = occ.copy()
        self.tree = {k: [set(c) for c in gs] for k, gs in tree.items()}
        for item in self.items[n:]:
            self.bd.b.Remove(item)
        del self.items[n:]
        self.failed = list(failed)

    def route_block(self, nets, label, tries=8):
        """Route a list of nets; on failure retry with the failed nets moved to the front."""
        order = list(nets)
        best = None
        for attempt in range(tries):
            snap = self.snapshot()
            before = len(self.failed)
            for net in order:
                self.route_net(net)
            bad = self.failed[before:]
            if not bad:
                print(f"{label}: routed (attempt {attempt + 1})", flush=True)
                return True
            if best is None or len(bad) < best[0]:
                best = (len(bad), list(order))
            if attempt == tries - 1:
                break
            self.restore(snap)
            order = list(dict.fromkeys(bad)) + [n for n in order if n not in bad]
        if best[1] != order:
            self.restore(snap)
            for net in best[1]:
                self.route_net(net)
        print(f"{label}: FAILED nets {sorted(set(self.failed[before:]))}", flush=True)
        return False

    def polyline(self, net, pts, w, layer=0):
        code = self.bd.nets[net].GetNetCode()
        for a, b in zip(pts, pts[1:]):
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
            for i in range(n + 1):
                t = i / n
                x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
                i0, i1 = self._rng(x - w / 2, x + w / 2, self.nx)
                j0, j1 = self._rng(y - w / 2, y + w / 2, self.ny)
                reg = self.occ[layer, i0:i1 + 1, j0:j1 + 1]
                if ((reg != 0) & (reg != code)).any():
                    print(f"  HAND TRACK CLASH {net} near {x:.2f},{y:.2f}")
                    break
            self.add_track(net, a, b, w, layer)
        self._merge(net)

    # --- search
    def _masks(self, code, w, win):
        a, b, c, d = win
        r = int(math.ceil((w / 2 - 1e-4) / PITCH))
        rv = int(math.ceil((VIA_D / 2 - 1e-4) / PITCH))
        m = max(r, rv)
        a2, b2, c2, d2 = max(0, a - m), min(self.nx - 1, b + m), max(0, c - m), min(self.ny - 1, d + m)
        free, viaok = [], None
        for layer in (0, 1):
            sub = self.occ[layer, a2:b2 + 1, c2:d2 + 1]
            blocked = (sub != 0) & (sub != code)
            f = ~dilate(blocked, r)
            v = ~dilate(blocked, rv)
            free.append(f[a - a2:a - a2 + (b - a + 1), c - c2:c - c2 + (d - c + 1)])
            v = v[a - a2:a - a2 + (b - a + 1), c - c2:c - c2 + (d - c + 1)]
            viaok = v if viaok is None else (viaok & v)
        viaok = viaok & ~self.novia[a:b + 1, c:d + 1]
        return [f.tolist() for f in free], viaok.tolist()

    def search(self, code, w, sources, goals, margin, allow_via=True, via_goal=None):
        """A* from any source cell to any goal cell.  Returns list of (layer, gx, gy)."""
        xs = [c[1] for c in sources] + [c[1] for c in goals]
        ys = [c[2] for c in sources] + [c[2] for c in goals]
        m = int(round(margin / PITCH))
        a, b = max(0, min(xs) - m), min(self.nx - 1, max(xs) + m)
        c, d = max(0, min(ys) - m), min(self.ny - 1, max(ys) + m)
        free, viaok = self._masks(code, w, (a, b, c, d))
        W, H = b - a + 1, d - c + 1
        goalset = {(l, x - a, y - c) for l, x, y in goals if a <= x <= b and c <= y <= d}
        if not goalset and via_goal is None:
            return None
        if goalset:
            gx1 = min(g[1] for g in goalset)
            gx2 = max(g[1] for g in goalset)
            gy1 = min(g[2] for g in goalset)
            gy2 = max(g[2] for g in goalset)

            def hfun(x, y):
                dx = gx1 - x if x < gx1 else (x - gx2 if x > gx2 else 0)
                dy = gy1 - y if y < gy1 else (y - gy2 if y > gy2 else 0)
                return 0.7 * (dx + dy)
        else:
            def hfun(x, y):
                return 0.0
        pq, dist, parent = [], {}, {}
        for l, x, y in sources:
            if a <= x <= b and c <= y <= d:
                k = (l, x - a, y - c)
                dist[k] = 0.0
                heapq.heappush(pq, (hfun(k[1], k[2]), 0.0, k))
        step_cost = (1.0, 0.7)
        found = None
        while pq:
            _, g, k = heapq.heappop(pq)
            if g > dist.get(k, 1e18):
                continue
            l, x, y = k
            if k in goalset:
                found = k
                break
            if via_goal is not None and viaok[x][y] and via_goal(x + a, y + c) and g > 0:
                found = k
                break
            fl = free[l]
            sc = step_cost[l]
            for nx_, ny_ in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if 0 <= nx_ < W and 0 <= ny_ < H:
                    nk = (l, nx_, ny_)
                    if not fl[nx_][ny_] and nk not in goalset:
                        continue
                    ng = g + sc
                    if ng < dist.get(nk, 1e18):
                        dist[nk] = ng
                        parent[nk] = k
                        heapq.heappush(pq, (ng + hfun(nx_, ny_), ng, nk))
            if allow_via and viaok[x][y]:
                nk = (1 - l, x, y)
                if free[1 - l][x][y]:
                    ng = g + 12.0
                    if ng < dist.get(nk, 1e18):
                        dist[nk] = ng
                        parent[nk] = k
                        heapq.heappush(pq, (ng + hfun(x, y), ng, nk))
        if found is None:
            return None
        path = [found]
        while path[-1] in parent:
            path.append(parent[path[-1]])
        path.reverse()
        return [(l, x + a, y + c) for l, x, y in path]

    def emit(self, net, path, w):
        mmxy = lambda x, y: (round(x * PITCH, 4), round(y * PITCH, 4))
        start = path[0]
        for i in range(1, len(path)):
            cur, prev = path[i], path[i - 1]
            if cur[0] != prev[0]:
                self.add_track(net, mmxy(start[1], start[2]), mmxy(prev[1], prev[2]), w, prev[0])
                self.add_via(net, *mmxy(cur[1], cur[2]))
                start = cur
                continue
            last = i == len(path) - 1
            turn = last
            if not last:
                nxt = path[i + 1]
                turn = nxt[0] != cur[0] or (cur[1] - prev[1], cur[2] - prev[2]) != (nxt[1] - cur[1], nxt[2] - cur[2])
            if turn:
                self.add_track(net, mmxy(start[1], start[2]), mmxy(cur[1], cur[2]), w, cur[0])
                start = cur

    def connect(self, net, src, dst, w, allow_via=True):
        code = self.bd.nets[net].GetNetCode()
        for margin in (2.5, 6.0, 15.0, 40.0):
            path = self.search(code, w, src, dst, margin, allow_via)
            if path:
                self.emit(net, path, w)
                return True
        return False

    def route_net(self, net, w=None, only_refs=None):
        """Join every copper group of a net (optionally only groups touching given refs)."""
        w = w or NET_WIDTH.get(net, W_SIG)
        self._merge(net)
        groups = self.tree.get(net, [])
        if only_refs is not None:
            keep = set()
            for (ref, num), cells in self.pad_cells.items():
                if ref in only_refs and self.bd.pad_net[(ref, num)] == net:
                    keep |= cells
            groups = [g for g in groups if g & keep]
        if len(groups) < 2:
            return True
        ok = True

        def centre(g):
            return (sum(c[1] for c in g) / len(g), sum(c[2] for c in g) / len(g))

        base = max(groups, key=len)
        rest = [g for g in groups if g is not base]
        while rest:
            bc = centre(base)
            rest.sort(key=lambda g: abs(centre(g)[0] - bc[0]) + abs(centre(g)[1] - bc[1]))
            # nearest group to any already joined copper: approximate by centre distance
            best = min(rest, key=lambda g: min(abs(c[1] - q[1]) + abs(c[2] - q[2])
                                               for c in list(g)[:6] for q in self._sample(base)))
            rest.remove(best)
            if self.connect(net, base, best, w):
                self._merge(net)
                base = next(g for g in self.tree[net] if g & best)
            elif w > W_SIG and self.connect(net, base, best, 0.2):
                self._merge(net)
                base = next(g for g in self.tree[net] if g & best)
            else:
                ok = False
                self.failed.append(net)
        return ok

    @staticmethod
    def _sample(cells, n=60):
        lst = list(cells)
        if len(lst) <= n:
            return lst
        step = len(lst) // n
        return lst[::step]

    def drop_to_plane(self, net, poly):
        """Give every copper group of a plane net a via (unless it already has a hole)."""
        code = self.bd.nets[net].GetNetCode()
        self._merge(net)
        missing = 0
        for grp in list(self.tree.get(net, [])):
            if any((1 - l, x, y) in grp for l, x, y in grp):
                if poly is None or any(in_poly(x * PITCH, y * PITCH, poly) for _, x, y in grp):
                    continue
            ok = (lambda gx, gy: True) if poly is None else (lambda gx, gy: in_poly(gx * PITCH, gy * PITCH, poly))
            done = False
            for margin in (1.5, 3.0, 8.0, 25.0):
                path = self.search(code, 0.25, grp, [], margin, allow_via=False, via_goal=ok)
                if path:
                    self.emit(net, path, 0.25)
                    l, x, y = path[-1]
                    self.add_via(net, round(x * PITCH, 4), round(y * PITCH, 4))
                    done = True
                    break
            if not done:
                missing += 1
                print(f"  no plane via for {net} group near "
                      f"{next(iter(grp))[1] * PITCH:.1f},{next(iter(grp))[2] * PITCH:.1f}")
            self._merge(net)
        return missing


# ------------------------------------------------------------------ routing plan
def hand_routes(rt):
    """Short, deliberate power connections around each charger IC."""
    for s in range(1, design.SLOTS + 1):
        cx, cy = cell_origin(s) + IC_X, IC_Y
        P = lambda dx, dy: (round(cx + dx, 4), round(cy + dy, 4))
        n = lambda name: f"S{s}_{name}"
        # input capacitor bank
        rt.polyline(n("PMID"), [P(-0.9, -1.7), P(-0.9, -2.8)], 0.2)
        rt.polyline(n("PMID"), [P(-0.9, -2.8), P(-5.85, -2.8)], 0.3)
        # output (SYS) capacitor bank
        rt.polyline(n("SYS"), [P(0.9, -1.7), P(0.9, -2.8)], 0.2)
        rt.polyline(n("SYS"), [P(0.9, -2.8), P(5.85, -2.8)], 0.3)
        # switch nodes to the inductor, bootstrap caps
        rt.polyline(n("SW1"), [P(-0.45, -1.7), P(-0.45, -3.6)], 0.2)
        rt.polyline(n("SW1"), [P(-0.45, -3.6), P(-1.5, -4.7), P(-1.5, -5.6)], 0.3)
        rt.polyline(n("SW1"), [P(-2.87, -6.6), P(-1.5, -6.6)], 0.25)
        rt.polyline(n("SW2"), [P(0.45, -1.7), P(0.45, -3.6)], 0.2)
        rt.polyline(n("SW2"), [P(0.45, -3.6), P(1.5, -4.7), P(1.5, -5.6)], 0.3)
        rt.polyline(n("SW2"), [P(2.87, -6.6), P(1.5, -6.6)], 0.25)
        # ground finger under the inductor, stitched to the plane
        rt.polyline("GND", [P(0, -1.7), P(0, -4.9)], 0.2)
        rt.polyline("GND", [P(0, -4.9), P(0, -9.4)], 1.0)
        for dy in (-5.6, -7.2, -8.8):
            rt.add_via("GND", *P(0, dy))
        # battery node: IC pins -> caps -> trunk up the right edge to the PicoBlade switch
        rt.polyline(n("BAT"), [P(1.9, -1.2), P(2.6, -1.2), P(2.6, -0.8), P(1.9, -0.8)], 0.2)
        rt.polyline(n("BAT"), [P(2.6, -1.0), P(6.65, -1.0), P(6.65, -1.85), P(8.5, -1.85),
                               P(8.5, -14.6), P(6.6, -14.6)], W_PWR)
        # VBUS pins to local decoupling
        rt.polyline("VBUS_PD", [P(-1.9, -1.2), P(-2.45, -1.2), P(-2.45, -0.8)], 0.2)
        rt.polyline("VBUS_PD", [P(-1.9, -0.8), P(-3.5, -0.8)], 0.2)
        rt.polyline("VBUS_PD", [P(-3.5, -0.8), P(-5.35, -0.8)], W_PWR)
        # VAC1/VAC2 sense pins get their own plane via so they do not fence in pins 4-7
        rt.polyline("VBUS_PD", [P(-1.9, 1.2), P(-2.15, 1.2), P(-2.15, 2.7), P(-2.6, 2.7), P(-2.6, 3.3)], 0.2)
        rt.polyline("VBUS_PD", [P(-1.4, 1.85), P(-1.4, 2.7), P(-2.15, 2.7)], 0.15)
        rt.add_via("VBUS_PD", *P(-2.6, 3.3))
        # bottom-row pins: fixed escapes to vias so later routing cannot fence them in
        rt.polyline("GND", [P(-1.0, 1.9), P(-1.0, 2.9), P(-0.6, 2.9), P(-0.6, 1.9)], 0.2)
        rt.polyline("GND", [P(-0.8, 2.9), P(-0.8, 3.3)], 0.2)
        rt.add_via("GND", *P(-0.8, 3.3))
        rt.polyline("CE_ALL_N", [P(0.2, 1.9), P(0.2, 3.8)], 0.15)
        rt.add_via("CE_ALL_N", *P(0.2, 3.8))
        rt.polyline(n("SCL"), [P(0.6, 1.9), P(0.6, 2.8), P(1.1, 3.3), P(1.1, 4.8)], 0.15)
        rt.add_via(n("SCL"), *P(1.1, 4.8))
        rt.polyline(n("SDA"), [P(1.0, 1.9), P(1.0, 2.6), P(1.9, 3.5), P(1.9, 3.9)], 0.15)
        rt.add_via(n("SDA"), *P(1.9, 3.9))
    # full adapter current enters the VBUS plane here: wide link across both bulk caps, three vias
    rt.polyline("VBUS_PD", [(24.94, 33.05), (26.2, 33.05), (26.2, 34.4), (30.8, 34.4)], 0.6)
    rt.polyline("VBUS_PD", [(26.2, 34.4), (26.2, 35.5)], 0.6)
    rt.polyline("VBUS_PD", [(28.5, 34.4), (28.5, 35.5)], 0.6)
    for vx, vy in ((26.2, 35.5), (28.5, 35.5), (30.8, 34.4)):
        rt.add_via("VBUS_PD", vx, vy)
    # PD controller ground pin to its exposed pad, one thermal via in the pad
    rt.polyline("GND", [(15.98, 37.6), (15.0, 37.6)], 0.2)
    rt.add_via("GND", 14.5, 38.0)
    # I2C mux address pin A2 to ground
    rt.polyline("GND", [(36.36, 54.375), (37.75, 54.375)], 0.2)
    rt.add_via("GND", 37.75, 54.375)
    # I2C mux address pins A0/A1 to ground, and 3V3 vias for its decoupling cap and bus pull-up
    rt.polyline("GND", [(30.64, 52.425), (29.6, 52.425), (29.6, 53.075), (30.64, 53.075)], 0.2)
    rt.polyline("GND", [(29.6, 52.75), (29.1, 52.75)], 0.2)
    rt.add_via("GND", 29.1, 52.75)
    rt.polyline("+3V3", [(38.72, 51.6), (38.72, 50.6)], 0.25)
    rt.add_via("+3V3", 38.72, 50.6)
    rt.polyline("+3V3", [(40.72, 53.6), (40.72, 52.6)], 0.25)
    rt.add_via("+3V3", 40.72, 52.6)
    for net in ("GND", "VBUS_PD", "CE_ALL_N", "+3V3"):
        rt._merge(net)


def slot_nets(s):
    order = ["REGN", "BTST1", "BTST2", "SDRV", "PROG", "BATP", "TS", "LED_A", "STAT",
             "PB_G", "PB_GD", "XT_G", "XT_GD", "PB_S", "XT_S", "PB_P", "XT_P", "EN_PB", "EN_XT",
             "BAT", "PMID", "SYS", "SW1", "SW2"]
    return [f"S{s}_{n}" for n in order]


def route_all(rt):
    hand_routes(rt)
    debug = "--debug" in sys.argv
    for s in range(1, design.SLOTS + 1):
        rt.route_block(slot_nets(s), f"slot {s}")
        if debug:
            return 0
    core = [n for n in design.nets() if not n.startswith("S") or n.split("_", 1)[1] in ("SDA", "SCL")]
    core = [n for n in core if n not in PLANE_NETS]
    first = ["USB_DP", "USB_DN", "USB_CC1", "USB_CC2", "VBUS_RAW", "VBUS_SNS", "BUCK_SW", "BUCK_BST",
             "PD_GATE", "PD_PWR_EN", "PD_VOUT", "PD_V5V", "PD_VSEL", "PD_ISEL", "PD_LED", "PD_LED_A"]
    rest = sorted(n for n in core if n not in first)
    # reserve plane vias for every power/ground pad before the long signal runs take the space
    for net, poly in (("VBUS_PD", VBUS_POLY), ("+3V3", V33_POLY), ("GND", None)):
        rt.drop_to_plane(net, poly)
    print("plane vias reserved", flush=True)
    rt.route_block(first + rest, "core", tries=8)
    missing = 0
    for net, poly in (("VBUS_PD", VBUS_POLY), ("+3V3", V33_POLY), ("GND", None)):
        missing += rt.drop_to_plane(net, poly)
    print(f"plane drops missing: {missing}", flush=True)
    return missing


# ------------------------------------------------------------------ finishing
def outline(board):
    c = [(0, 0), (BOARD_W, 0), (BOARD_W, BOARD_H), (0, BOARD_H)]
    for i in range(4):
        s = pcbnew.PCB_SHAPE(board.b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetStart(pt(*c[i]))
        s.SetEnd(pt(*c[(i + 1) % 4]))
        s.SetWidth(mm(0.1))
        board.b.Add(s)


def silk(board):
    def text(txt, x, y, size=1.0, thick=0.15):
        t = pcbnew.PCB_TEXT(board.b)
        t.SetText(txt)
        t.SetLayer(pcbnew.F_SilkS)
        t.SetPosition(pt(x, y))
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
        t.SetTextThickness(mm(thick))
        board.b.Add(t)

    for s in range(1, design.SLOTS + 1):
        x = cell_origin(s)
        text(str(s), x + 9.0, 9.4, 1.4, 0.22)
    text("6x 1S/2S LiPo CHARGER  rev A", 60.0, 66.3, 1.0, 0.15)
    text("UP", 70.0, 39.0, 0.8, 0.13)
    text("OK", 70.0, 49.5, 0.8, 0.13)
    text("DN", 70.0, 60.0, 0.8, 0.13)
    text("OLED: GND VCC SCL SDA", 54.0, 33.6, 0.8, 0.13)
    text("TX RX GND", 90.5, 53.6, 0.8, 0.13)
    text("USB-C PD", 6.0, 47.6, 0.8, 0.13)


def zones(board):
    board.zone("GND", pcbnew.In1_Cu, GND_POLY, thermal=True)
    board.zone("VBUS_PD", pcbnew.In2_Cu, VBUS_POLY, thermal=True)
    board.zone("+3V3", pcbnew.In2_Cu, V33_POLY, thermal=True)
    board.zone("GND", pcbnew.F_Cu, GND_POLY)
    board.zone("GND", pcbnew.B_Cu, GND_POLY, thermal=True)
    pcbnew.ZONE_FILLER(board.b).Fill(board.b.Zones())


def main():
    board = Board()
    board.load(placement())
    bad = check_overlaps(board)
    outline(board)
    if "--place-only" not in sys.argv:
        rt = Router(board)
        route_all(rt)
        print("failed nets:", sorted(set(rt.failed)))
        zones(board)
    silk(board)
    pcbnew.SaveBoard(OUT, board.b)
    print("saved", OUT, "| courtyard overlaps:", bad)


if __name__ == "__main__":
    main()
