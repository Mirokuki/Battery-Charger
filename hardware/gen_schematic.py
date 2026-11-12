#!/usr/bin/env python3
"""
Writes the KiCad schematic from design.py:
    lipo-charger.kicad_sch      power input, MCU, UI, I2C mux
    slot1..slot6.kicad_sch      one charger slot each

Style: every pin carries a net label instead of drawn wires, so the files are a
faithful, ERC-checkable picture of design.py rather than a hand-drawn schematic.
    python gen_schematic.py
"""

import os
import uuid

import design
from sexpr import Sym, dump, find, first, parse

HERE = os.path.dirname(os.path.abspath(__file__))
SYM_ROOT = r"C:\Program Files\KiCad\10.0\share\kicad\symbols"
PROJECT = "lipo-charger"
NS = uuid.UUID("6b1f0c1e-5d0a-4b7e-9c55-25798c3a0001")


def uid(*key):
    return str(uuid.uuid5(NS, "/".join(str(k) for k in key)))


# ------------------------------------------------------------------ symbol library access
_libs = {}


def box_symbol(name, left, right):
    """Simple rectangular symbol: lists of (number, name) for each side."""
    rows = max(len(left), len(right))
    h = 2.54 * (rows + 1)
    w = 20.32
    body = [Sym("symbol"), f"{name}_0_1",
            [Sym("rectangle"), [Sym("start"), -w / 2, h / 2], [Sym("end"), w / 2, -h / 2],
             [Sym("stroke"), [Sym("width"), 0.254], [Sym("type"), Sym("default")]],
             [Sym("fill"), [Sym("type"), Sym("background")]]]]
    pins = [Sym("symbol"), f"{name}_1_1"]
    for side, items in ((0, left), (1, right)):
        for i, (num, pname) in enumerate(items):
            y = h / 2 - 2.54 * (i + 1)
            x = -w / 2 - 5.08 if side == 0 else w / 2 + 5.08
            pins.append([Sym("pin"), Sym("passive"), Sym("line"), [Sym("at"), x, y, 0 if side == 0 else 180],
                         [Sym("length"), 5.08],
                         [Sym("name"), pname, [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]],
                         [Sym("number"), str(num), [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]]])
    return [Sym("symbol"), name, [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes")],
            [Sym("on_board"), Sym("yes")],
            [Sym("property"), "Reference", "U", [Sym("at"), 0, h / 2 + 1.27, 0],
             [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]],
            [Sym("property"), "Value", name, [Sym("at"), 0, -h / 2 - 1.27, 0],
             [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]],
            [Sym("property"), "Footprint", "", [Sym("at"), 0, 0, 0],
             [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]], [Sym("hide"), Sym("yes")]]],
            body, pins]


CUSTOM = {
    "charger:BQ29209": box_symbol(
        "BQ29209",
        [(1, "VC2"), (2, "VC1"), (3, "VC1_CB"), (4, "CD")],
        [(7, "VDD"), (8, "OUT"), (6, "CB_EN"), (5, "GND"), (9, "EPAD")]),
    "charger:AP33771C": box_symbol(
        "AP33771C",
        [(4, "ISENP"), (3, "VCC"), (11, "CC1"), (10, "CC2"), (13, "DP"), (12, "DN"), (5, "GND"), (15, "EPAD")],
        [(2, "PWR_EN"), (1, "VOUT"), (14, "V5V"), (8, "VSEL"), (9, "ISEL"), (7, "LED"), (6, "FLIP")]),
}


def lib_symbol(lib_id):
    """Return a flattened (no 'extends') symbol definition named lib_id."""
    if lib_id in CUSTOM:
        sym = [x for x in CUSTOM[lib_id]]
        sym[1] = lib_id
        return sym
    lib, name = lib_id.split(":")
    if lib not in _libs:
        with open(os.path.join(SYM_ROOT, lib + ".kicad_sym"), encoding="utf8") as f:
            _libs[lib] = {s[1]: s for s in find(parse(f.read()), "symbol")}
    sym = _libs[lib][name]
    ext = first(sym, "extends")
    if ext:
        parent = _libs[lib][ext[1]]
        merged = [Sym("symbol"), lib_id]
        own_props = {p[1] for p in find(sym, "property")}
        for item in parent[2:]:
            if isinstance(item, list) and item[0] == "property" and item[1] in own_props:
                continue
            if isinstance(item, list) and item[0] == "symbol":
                item = [item[0], name + item[1][len(ext[1]):]] + item[2:]
            merged.append(item)
        merged[2:2] = [p for p in find(sym, "property")]
        return merged
    return [sym[0], lib_id] + sym[2:]


def symbol_pins(sym):
    out = {}
    for unit in find(sym, "symbol"):
        for p in find(unit, "pin"):
            at = first(p, "at")
            out.setdefault(first(p, "number")[1], (float(at[1]), float(at[2]), int(float(at[3])) if len(at) > 3 else 0))
    return out


# ------------------------------------------------------------------ sheet writer
FONT = [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]


def snap(v):
    return round(round(v / 1.27) * 1.27, 4)


class Sheet:
    def __init__(self, path_uuids, title, page):
        self.path = "/" + "/".join(path_uuids)
        self.uuid = path_uuids[-1]
        self.title = title
        self.page = page
        self.libs = {}
        self.items = []
        self.x, self.y, self.row_h = 30.0, 40.0, 0.0
        self.max_x = 380.0

    def note(self, text, x, y, size=2.5):
        self.items.append([Sym("text"), text, [Sym("at"), x, y, 0],
                           [Sym("effects"), [Sym("font"), [Sym("size"), size, size]],
                            [Sym("justify"), Sym("left"), Sym("bottom")]],
                           [Sym("uuid"), uid(self.uuid, "note", text)]])

    def newline(self, gap=12.0):
        self.x = 30.0
        self.y += self.row_h + gap
        self.row_h = 0.0

    def place(self, ref):
        p = design.PARTS[ref]
        lib_id = p["sym"]
        if lib_id not in self.libs:
            self.libs[lib_id] = lib_symbol(lib_id)
        pins = symbol_pins(self.libs[lib_id])
        xs = [v[0] for v in pins.values()]
        ys = [v[1] for v in pins.values()]
        horizontal = any(v[2] in (0, 180) for v in pins.values())
        vertical = any(v[2] in (90, 270) for v in pins.values())
        left = -min(xs) + (30.0 if horizontal else 4.0)
        right = max(xs) + (30.0 if horizontal else 4.0)
        up = max(ys) + (30.0 if vertical else 6.0)
        down = -min(ys) + (30.0 if vertical else 6.0)
        if self.x + left + right > self.max_x:
            self.newline()
        cx, cy = snap(self.x + left), snap(self.y + up)
        self.x += left + right + 4.0
        self.row_h = max(self.row_h, up + down)
        inst = [Sym("symbol"), [Sym("lib_id"), lib_id], [Sym("at"), cx, cy, 0], [Sym("unit"), 1],
                [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes")], [Sym("on_board"), Sym("yes")],
                [Sym("dnp"), Sym("yes") if p["dnp"] else Sym("no")], [Sym("uuid"), uid(ref)],
                [Sym("property"), "Reference", ref, [Sym("at"), cx, snap(cy - max(ys) - 2.54), 0], FONT],
                [Sym("property"), "Value", p["value"], [Sym("at"), cx, snap(cy - min(ys) + 2.54), 0], FONT],
                [Sym("property"), "Footprint", p["fp"], [Sym("at"), cx, cy, 0],
                 [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]], [Sym("hide"), Sym("yes")]]],
                [Sym("property"), "MPN", p["mpn"], [Sym("at"), cx, cy, 0],
                 [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]], [Sym("hide"), Sym("yes")]]],
                [Sym("property"), "LCSC", p["lcsc"], [Sym("at"), cx, cy, 0],
                 [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]], [Sym("hide"), Sym("yes")]]]]
        for num in pins:
            inst.append([Sym("pin"), num, [Sym("uuid"), uid(ref, "pin", num)]])
        inst.append([Sym("instances"), [Sym("project"), PROJECT,
                                        [Sym("path"), self.path, [Sym("reference"), ref], [Sym("unit"), 1]]]])
        self.items.append(inst)
        for num, (px, py, ang) in pins.items():
            x, y = round(cx + px, 4), round(cy - py, 4)
            net = p["pins"].get(num)
            if net is None:
                self.items.append([Sym("no_connect"), [Sym("at"), x, y], [Sym("uuid"), uid(ref, "nc", num)]])
                continue
            lab_ang = {0: 180, 180: 0, 90: 270, 270: 90}[ang % 360]
            just = Sym("right") if lab_ang in (180, 270) else Sym("left")
            self.items.append([Sym("global_label"), net, [Sym("shape"), Sym("passive")],
                               [Sym("at"), x, y, lab_ang],
                               [Sym("effects"), [Sym("font"), [Sym("size"), 1.0, 1.0]], [Sym("justify"), just]],
                               [Sym("uuid"), uid(ref, "lab", num)]])

    def add_sheet(self, name, filename, sheet_uuid, x, y, page):
        self.items.append([Sym("sheet"), [Sym("at"), x, y], [Sym("size"), 40, 12],
                           [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes")],
                           [Sym("on_board"), Sym("yes")], [Sym("dnp"), Sym("no")],
                           [Sym("stroke"), [Sym("width"), 0.15], [Sym("type"), Sym("solid")]],
                           [Sym("fill"), [Sym("color"), 0, 0, 0, 0.0]],
                           [Sym("uuid"), sheet_uuid],
                           [Sym("property"), "Sheetname", name, [Sym("at"), x, y - 0.7, 0],
                            [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]],
                             [Sym("justify"), Sym("left"), Sym("bottom")]]],
                           [Sym("property"), "Sheetfile", filename, [Sym("at"), x, y + 12.7, 0],
                            [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]],
                             [Sym("justify"), Sym("left"), Sym("top")]]],
                           [Sym("instances"), [Sym("project"), PROJECT,
                                               [Sym("path"), self.path, [Sym("page"), str(page)]]]]])

    def write(self, filename, root=False):
        doc = [Sym("kicad_sch"), [Sym("version"), 20260306], [Sym("generator"), "gen_schematic"],
               [Sym("generator_version"), "10.0"], [Sym("uuid"), self.uuid], [Sym("paper"), "A2"],
               [Sym("title_block"), [Sym("title"), self.title], [Sym("rev"), "A"]],
               [Sym("lib_symbols")] + list(self.libs.values())]
        doc += self.items
        if root:
            doc.append([Sym("sheet_instances"), [Sym("path"), "/", [Sym("page"), "1"]]])
        with open(os.path.join(HERE, filename), "w", encoding="utf8", newline="\n") as f:
            f.write(dump(doc) + "\n")


def order_key(ref):
    kind = "".join(c for c in ref if c.isalpha())
    rank = {"J": 0, "U": 1, "Q": 2, "L": 3, "SW": 4, "BZ": 5, "D": 6, "C": 7, "R": 8}.get(kind, 9)
    return (rank, int("".join(c for c in ref if c.isdigit())))


def main():
    root_uuid = uid("root")
    root = Sheet([root_uuid], "6-slot 1S/2S USB-C LiPo charger - power input, MCU, UI", 1)
    root.max_x = 560.0
    root.note("USB-C PD input (AP33771C asks for 9 V), 3.3 V buck, ESP32-C3, OLED + 3 buttons, I2C mux to the six slots.\n"
              "Label-style schematic generated from design.py: pins with the same label are connected.", 30, 28)
    groups = {}
    for ref, p in design.PARTS.items():
        groups.setdefault(p["group"], []).append(ref)
    big = [r for r in sorted(groups["core"], key=order_key) if order_key(r)[0] <= 5]
    small = [r for r in sorted(groups["core"], key=order_key) if order_key(r)[0] > 5]
    for ref in big:
        root.place(ref)
    root.newline(20)
    for ref in small:
        root.place(ref)
    root.newline(25)
    for s in range(1, design.SLOTS + 1):
        su = uid("sheet", s)
        root.add_sheet(f"Slot {s}", f"slot{s}.kicad_sch", su, 30 + 50 * (s - 1), root.y, s + 1)
        sh = Sheet([root_uuid, su], f"Charger slot {s}: BQ25798 + PicoBlade (1S) / XT30 (2S) switches", s + 1)
        sh.max_x = 560.0
        sh.note("BQ25798 buck-boost charger.  D+ drives the PicoBlade switch, D- the XT30 switch (register 0x47).\n"
                "Each connector has a back-to-back P-FET pair so the two packs can never be connected together.", 30, 28)
        refs = sorted(groups[f"slot{s}"], key=order_key)
        for ref in [r for r in refs if order_key(r)[0] <= 5]:
            sh.place(ref)
        sh.newline(20)
        for ref in [r for r in refs if order_key(r)[0] > 5]:
            sh.place(ref)
        sh.write(f"slot{s}.kicad_sch")
    root.write(f"{PROJECT}.kicad_sch", root=True)
    print("schematic written")
    # KiCad's own netlist of what was just written: gen_pcb.py reads the names KiCad gives
    # unconnected pins from it, and check_netlist() proves the schematic equals design.py
    import subprocess
    kc = os.path.join("C:" + os.sep, "Program Files", "KiCad", "10.0", "bin", "kicad-cli.exe")
    out = os.path.join(HERE, "sch_netlist.xml")
    subprocess.run([kc, "sch", "export", "netlist", "--format", "kicadxml", "-o", out,
                    os.path.join(HERE, f"{PROJECT}.kicad_sch")], capture_output=True)
    check_netlist(out)


def check_netlist(path):
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    sch = {n.get("name").lstrip("/"): frozenset((x.get("ref"), x.get("pin")) for x in n) for n in root.find("nets")}
    want = {k: frozenset(v) for k, v in design.nets().items()}
    bad = [k for k, v in want.items() if sch.get(k) != v]
    extra = [k for k in sch if k not in want and not k.startswith("unconnected-")]
    parts = {c.get("ref") for c in root.find("components")}
    print(f"netlist check: {len(parts)}/{len(design.PARTS)} parts, {len(bad)} net mismatches, {len(extra)} extra nets")


if __name__ == "__main__":
    main()
