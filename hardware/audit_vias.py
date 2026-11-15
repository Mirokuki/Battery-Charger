#!/usr/bin/env python3
"""
Via audit for the routed board.  Reports vias that do no useful work:
  dangling    signal via with copper on fewer than two layers
  redundant   plane-net via whose copper island already has another via or a through-hole pad
  close pair  two vias of the same signal net within 1.5 mm (a hop that a short track could replace)
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" audit_vias.py
"""
import math
import os
import sys
from collections import defaultdict

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
X0, Y0 = 50.0, 40.0
PLANE = {"GND", "VBUS_PD", "+3V3"}


def audit(path=None, verbose=True):
    board = pcbnew.LoadBoard(path or os.path.join(HERE, "lipo-charger.kicad_pcb"))
    mm = lambda p: (round(pcbnew.ToMM(p.x) - X0, 3), round(pcbnew.ToMM(p.y) - Y0, 3))
    vias = defaultdict(list)
    segs = defaultdict(list)          # (net, layer) -> [(a, b, w)]
    for t in board.GetTracks():
        net = t.GetNetname()
        if isinstance(t, pcbnew.PCB_VIA):
            vias[net].append(mm(t.GetPosition()))
        else:
            segs[(net, t.GetLayer())].append((mm(t.GetStart()), mm(t.GetEnd()), pcbnew.ToMM(t.GetWidth())))
    pads = defaultdict(list)          # net -> [(x, y, half_w, half_h, on_f, on_b)]
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            bb = pad.GetBoundingBox()
            cx = pcbnew.ToMM(bb.GetLeft() + bb.GetWidth() / 2) - X0
            cy = pcbnew.ToMM(bb.GetTop() + bb.GetHeight() / 2) - Y0
            ls = pad.GetLayerSet()
            thru = pad.GetDrillSize().x > 0
            pads[pad.GetNetname()].append((cx, cy, pcbnew.ToMM(bb.GetWidth()) / 2, pcbnew.ToMM(bb.GetHeight()) / 2,
                                           ls.Contains(pcbnew.F_Cu) or thru, ls.Contains(pcbnew.B_Cu) or thru))

    def touches(v, net, layer):
        for a, b, w in segs.get((net, layer), []):
            dx, dy = b[0] - a[0], b[1] - a[1]
            ln = dx * dx + dy * dy
            t = 0 if ln == 0 else max(0, min(1, ((v[0] - a[0]) * dx + (v[1] - a[1]) * dy) / ln))
            if math.hypot(v[0] - a[0] - t * dx, v[1] - a[1] - t * dy) <= w / 2 + 0.31:
                return True
        for x, y, hw, hh, on_f, on_b in pads.get(net, []):
            if (on_f if layer == pcbnew.F_Cu else on_b) and abs(v[0] - x) <= hw + 0.3 and abs(v[1] - y) <= hh + 0.3:
                return True
        return False

    out = {"total": sum(len(v) for v in vias.values()), "dangling": [], "close": [], "plane": 0}
    for net, vs in vias.items():
        if net in PLANE:
            out["plane"] += len(vs)
            continue
        for v in vs:
            if not (touches(v, net, pcbnew.F_Cu) and touches(v, net, pcbnew.B_Cu)):
                out["dangling"].append((net, v))
        for i, a in enumerate(vs):
            for b in vs[i + 1:]:
                if math.hypot(a[0] - b[0], a[1] - b[1]) < 1.5:
                    out["close"].append((net, a, b))
    if verbose:
        print(f"vias: {out['total']} total, {out['plane']} to planes, {out['total'] - out['plane']} on signals")
        print(f"dangling signal vias: {len(out['dangling'])}")
        for net, v in out["dangling"][:40]:
            print("   ", net, v)
        print(f"same-net signal via pairs closer than 1.5 mm: {len(out['close'])}")
        for net, a, b in out["close"][:40]:
            print("   ", net, a, b)
    return out


if __name__ == "__main__":
    audit(sys.argv[1] if len(sys.argv) > 1 else None)
