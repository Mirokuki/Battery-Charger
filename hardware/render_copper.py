#!/usr/bin/env python3
"""
Quick 2D picture of the routed copper (front = red, back = blue, vias = green,
pads = yellow) for eyeballing the layout without opening KiCad.
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" render_copper.py [x1 y1 x2 y2] [out.png]
"""
import os
import struct
import sys
import zlib

import numpy as np
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
X0, Y0 = 50.0, 40.0


def save_png(path, img):
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def main():
    args = [a for a in sys.argv[1:] if not a.endswith(".png")]
    out = next((a for a in sys.argv[1:] if a.endswith(".png")), os.path.join(HERE, "..", "docs", "copper.png"))
    x1, y1, x2, y2 = (float(a) for a in args) if len(args) == 4 else (0.0, 0.0, 108.0, 80.0)
    scale = min(40.0, 2400.0 / (x2 - x1))
    w, h = int((x2 - x1) * scale), int((y2 - y1) * scale)
    img = np.full((h, w, 3), 18, np.uint8)
    board = pcbnew.LoadBoard(os.path.join(HERE, "lipo-charger.kicad_pcb"))

    def box(ax, ay, bx, by, colour, blend=False):
        i0, i1 = int((min(ax, bx) - x1) * scale), int((max(ax, bx) - x1) * scale) + 1
        j0, j1 = int((min(ay, by) - y1) * scale), int((max(ay, by) - y1) * scale) + 1
        i0, i1, j0, j1 = max(0, i0), min(w, i1), max(0, j0), min(h, j1)
        if i0 >= i1 or j0 >= j1:
            return
        if blend:
            img[j0:j1, i0:i1] = np.maximum(img[j0:j1, i0:i1], np.array(colour, np.uint8))
        else:
            img[j0:j1, i0:i1] = colour

    def seg(a, b, width, colour):
        n = max(1, int(np.hypot(b[0] - a[0], b[1] - a[1]) * scale / 2))
        for i in range(n + 1):
            t = i / n
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            box(x - width / 2, y - width / 2, x + width / 2, y + width / 2, colour, blend=True)

    mmxy = lambda p: (pcbnew.ToMM(p.x) - X0, pcbnew.ToMM(p.y) - Y0)
    tracks = list(board.GetTracks())
    for layer, colour in ((pcbnew.B_Cu, (40, 90, 230)), (pcbnew.F_Cu, (220, 50, 50))):
        for t in tracks:
            if not isinstance(t, pcbnew.PCB_VIA) and t.GetLayer() == layer:
                seg(mmxy(t.GetStart()), mmxy(t.GetEnd()), pcbnew.ToMM(t.GetWidth()), colour)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if not pad.GetLayerSet().Contains(pcbnew.F_Cu):
                continue
            x, y = mmxy(pad.GetPosition())
            s = pad.GetSize(pcbnew.F_Cu)
            sx, sy = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y)
            if abs(pad.GetOrientationDegrees() % 180 - 90) < 10:
                sx, sy = sy, sx
            box(x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2, (225, 200, 60))
    for t in tracks:
        if isinstance(t, pcbnew.PCB_VIA):
            x, y = mmxy(t.GetPosition())
            box(x - 0.3, y - 0.3, x + 0.3, y + 0.3, (60, 210, 90))
            box(x - 0.12, y - 0.12, x + 0.12, y + 0.12, (18, 18, 18))
    save_png(out, img)
    print("wrote", out, w, "x", h)


if __name__ == "__main__":
    main()
