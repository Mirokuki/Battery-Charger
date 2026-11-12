#!/usr/bin/env python3
"""
3D-printable case for the 6-slot 1S/2S LiPo charger.  No dependencies:
    python make_case.py        -> case_base.stl, case_lid.stl

Two parts, both print flat without supports:
  base  tray with four screw bosses the PCB sits on (print as modelled)
  lid   top plate with connector / display / button openings (print top-face down;
        the STL is already flipped)
Four M2.5 x 14 mm self-tapping screws go through lid posts and the PCB holes into
the base bosses.

Every solid is an extruded 2D outline.  Coordinates match the PCB (x right,
y towards the front edge, origin at the board's rear-left corner).
"""

import math
import os
import struct

# ------------------------------------------------------------------ parameters (mm)
BOARD_W, BOARD_H, BOARD_T = 108.0, 68.0, 1.6
GAP = 0.4            # board-to-wall clearance
WALL = 2.0
FLOOR = 1.6
STANDOFF = 4.0       # board underside above the floor (clears through-hole pins)
HEADROOM = 6.0       # board top to lid underside (tallest enclosed part is the OLED, 5.5)
LID_T = 1.6
LIP_T, LIP_H, LIP_FIT = 1.2, 2.5, 0.2
HOLES = [(37.3, 34.6), (105.0, 32.6), (31.0, 65.0), (105.0, 65.0)]   # PCB mounting holes
BOSS_D, BOSS_HOLE = 6.0, 2.1       # 2.1 mm pilot for M2.5 self-tapping screws
POST_D, POST_HOLE = 5.4, 2.9
SCREW_HEAD_D = 5.2

SLOTS, CELL_W, CELL_X0 = 6, 17.6, 1.2
XT30 = (6.1, 4.0, 11.0, 6.2)       # centre x, centre y in the slot cell, opening w, h
PICO = (14.9, 4.05, 5.4, 6.0)      # PicoBlade opening (0.6 mm ribs separate it from the XT30 openings)
SLOT_LED = (2.6, 28.0, 2.0)        # charge LED window: x, y in the cell, diameter
PD_LED = (15.6, 32.3, 2.0)
OLED_WIN = (54.0, 46.2, 25.0, 14.5)   # centre x, y, w, h  - check against your module
BUTTONS = [(76.0, 39.0), (76.0, 49.5), (76.0, 60.0)]
BUTTON_D = 4.0
BUZZER = (93.0, 39.6, 1.6)
USB = (38.0, 13.0, 7.6)            # centre y on the left wall, opening width, height (fits the plug body)
# extra rests under the board, placed clear of every through-hole solder joint
SUPPORTS = [(13.2, 3.0), (48.4, 3.0), (83.6, 3.0), (3.0, 22.0), (60.0, 31.5), (70.0, 64.5), (3.0, 64.5)]

Z_BOARD = FLOOR + STANDOFF                  # board underside
Z_TOP = Z_BOARD + BOARD_T + HEADROOM        # wall top = lid underside
OX = OY = WALL + GAP                        # board origin inside the case outline
OUT_W, OUT_H = BOARD_W + 2 * OX, BOARD_H + 2 * OY


# ------------------------------------------------------------------ geometry kernel
def rect(x1, y1, x2, y2):
    return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]


def crect(cx, cy, w, h):
    return rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def circle(cx, cy, d, n=40):
    r = d / 2
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


class Mesh:
    def __init__(self):
        self.tris = []

    def prism(self, loops, z0, z1):
        """Extrude the even-odd fill of closed loops from z0 to z1 (watertight, no T-junctions)."""
        R = lambda v: round(v, 6)
        loops = [[(R(x), R(y)) for x, y in lp] for lp in loops]
        xs = sorted({p[0] for lp in loops for p in lp})
        edges = [(lp[i], lp[(i + 1) % len(lp)]) for lp in loops for i in range(len(lp))
                 if lp[i] != lp[(i + 1) % len(lp)]]
        # for every vertical strip: the edges crossing it, sorted bottom to top
        strips = []
        for xa, xb in zip(xs, xs[1:]):
            cross = []
            for p, q in edges:
                if p[0] != q[0] and min(p[0], q[0]) <= xa and max(p[0], q[0]) >= xb:
                    ya = p[1] + (xa - p[0]) / (q[0] - p[0]) * (q[1] - p[1])
                    yb = p[1] + (xb - p[0]) / (q[0] - p[0]) * (q[1] - p[1])
                    cross.append(((ya + yb) / 2, R(ya), R(yb)))
            cross.sort()
            strips.append(cross)
        # every y that appears on a strip boundary: both neighbours must use the same points there
        bound = {x: set() for x in xs}
        for i, cross in enumerate(strips):
            for _, ya, yb in cross:
                bound[xs[i]].add(ya)
                bound[xs[i + 1]].add(yb)
        for lp in loops:
            for p in lp:
                bound[p[0]].add(p[1])
        for i, cross in enumerate(strips):
            xa, xb = xs[i], xs[i + 1]
            for lo, hi in zip(cross[0::2], cross[1::2]):
                right = [y for y in sorted(bound[xb]) if lo[2] < y < hi[2]]
                left = [y for y in sorted(bound[xa]) if lo[1] < y < hi[1]]
                poly = [(xa, lo[1]), (xb, lo[2])] + [(xb, y) for y in right] + [(xb, hi[2]), (xa, hi[1])]                     + [(xa, y) for y in reversed(left)]
                poly = [p for k, p in enumerate(poly) if p != poly[k - 1]]
                if len(poly) < 3:
                    continue
                cx = sum(p[0] for p in poly) / len(poly)
                cy = sum(p[1] for p in poly) / len(poly)
                for k in range(len(poly)):
                    p, q = poly[k], poly[(k + 1) % len(poly)]
                    self.tris.append(((cx, cy, z1), (p[0], p[1], z1), (q[0], q[1], z1)))
                    self.tris.append(((cx, cy, z0), (q[0], q[1], z0), (p[0], p[1], z0)))
        # walls, split at the same points the caps use
        for lp in loops:
            area = sum(lp[i][0] * lp[(i + 1) % len(lp)][1] - lp[(i + 1) % len(lp)][0] * lp[i][1]
                       for i in range(len(lp)))
            depth = sum(1 for other in loops if other is not lp and _inside(lp[0], other))
            ccw = area > 0
            outward = (depth % 2 == 0)
            n = len(lp)
            for i in range(n):
                a, b = lp[i], lp[(i + 1) % n]
                if a == b:
                    continue
                if a[0] == b[0]:
                    ys = [y for y in sorted(bound[a[0]]) if min(a[1], b[1]) < y < max(a[1], b[1])]
                    pts = [a] + [(a[0], y) for y in (ys if b[1] > a[1] else ys[::-1])] + [b]
                else:
                    cuts = [x for x in xs if min(a[0], b[0]) < x < max(a[0], b[0])]
                    pts = [a] + [(x, R(a[1] + (x - a[0]) / (b[0] - a[0]) * (b[1] - a[1]))) for x in
                                 (cuts if b[0] > a[0] else cuts[::-1])] + [b]
                for p, q in zip(pts, pts[1:]):
                    quad = [(p[0], p[1], z0), (q[0], q[1], z0), (q[0], q[1], z1), (p[0], p[1], z1)]
                    self._quad(quad, flip=(ccw != outward))

    def _quad(self, q, flip=False):
        if flip:
            q = q[::-1]
        self.tris.append((q[0], q[1], q[2]))
        self.tris.append((q[0], q[2], q[3]))

    def transformed(self, fn):
        m = Mesh()
        m.tris = [tuple(fn(v) for v in t) for t in self.tris]
        return m

    def save(self, path, mirror=False):
        with open(path, "wb") as f:
            f.write(b"lipo charger case".ljust(80, b" "))
            f.write(struct.pack("<I", len(self.tris)))
            for a, b, c in self.tris:
                if mirror:
                    b, c = c, b
                ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
                vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
                nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
                ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
                f.write(struct.pack("<12fH", nx / ln, ny / ln, nz / ln, *a, *b, *c, 0))

    def volume(self):
        v = 0.0
        for a, b, c in self.tris:
            v += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                  + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        return v


def _inside(p, loop):
    x, y = p
    inside = False
    n = len(loop)
    for i in range(n):
        x1, y1 = loop[i]
        x2, y2 = loop[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def B(x, y):
    """Board coordinates -> case outline coordinates."""
    return x + OX, y + OY


# ------------------------------------------------------------------ parts
def build_base():
    m = Mesh()
    outer = rect(0, 0, OUT_W, OUT_H)
    inner = rect(WALL, WALL, OUT_W - WALL, OUT_H - WALL)
    m.prism([outer], 0, FLOOR)
    # walls, with the USB-C opening cut through the left wall
    uy = USB[0] + OY
    z_usb0 = Z_BOARD + BOARD_T + 1.6 - USB[2] / 2     # receptacle axis is 1.6 mm above the board
    z_usb1 = z_usb0 + USB[2]
    m.prism([outer, inner], FLOOR - 0.01, z_usb0)
    y1, y2 = uy - USB[1] / 2, uy + USB[1] / 2
    c_shape = [(0, 0), (OUT_W, 0), (OUT_W, OUT_H), (0, OUT_H), (0, y2), (WALL, y2), (WALL, OUT_H - WALL),
               (OUT_W - WALL, OUT_H - WALL), (OUT_W - WALL, WALL), (WALL, WALL), (WALL, y1), (0, y1)]
    m.prism([c_shape], z_usb0 - 0.01, z_usb1)
    m.prism([outer, inner], z_usb1 - 0.01, Z_TOP)
    for hx, hy in HOLES:
        x, y = B(hx, hy)
        m.prism([circle(x, y, BOSS_D), circle(x, y, BOSS_HOLE)], FLOOR - 0.01, Z_BOARD)
    for sx, sy in SUPPORTS:
        x, y = B(sx, sy)
        m.prism([circle(x, y, 3.0, 24)], FLOOR - 0.01, Z_BOARD)
    return m


def build_lid():
    """Modelled in place (plate on top), then flipped for printing."""
    m = Mesh()
    z0, z1 = Z_TOP, Z_TOP + LID_T
    loops = [rect(0, 0, OUT_W, OUT_H)]
    for s in range(SLOTS):
        cx = CELL_X0 + CELL_W * s
        loops.append(crect(*B(cx + XT30[0], XT30[1]), XT30[2], XT30[3]))
        loops.append(crect(*B(cx + PICO[0], PICO[1]), PICO[2], PICO[3]))
        loops.append(circle(*B(cx + SLOT_LED[0], SLOT_LED[1]), SLOT_LED[2], 20))
    loops.append(circle(*B(PD_LED[0], PD_LED[1]), PD_LED[2], 20))
    loops.append(crect(*B(OLED_WIN[0], OLED_WIN[1]), OLED_WIN[2], OLED_WIN[3]))
    for bx, by in BUTTONS:
        loops.append(circle(*B(bx, by), BUTTON_D))
    loops.append(circle(*B(BUZZER[0], BUZZER[1]), BUZZER[2], 16))
    for hx, hy in HOLES:
        loops.append(circle(*B(hx, hy), POST_HOLE))
    m.prism(loops, z0, z1)
    # locating lip inside the walls
    a = WALL + LIP_FIT
    m.prism([rect(a, a, OUT_W - a, OUT_H - a),
             rect(a + LIP_T, a + LIP_T, OUT_W - a - LIP_T, OUT_H - a - LIP_T)], z0 - LIP_H, z0 + 0.01)
    # screw posts that clamp the board
    for hx, hy in HOLES:
        x, y = B(hx, hy)
        m.prism([circle(x, y, POST_D), circle(x, y, POST_HOLE)], Z_BOARD + BOARD_T + 0.1, z0 + 0.01)
    # flip so the outside face lies on the print bed
    return m.transformed(lambda v: (v[0], OUT_H - v[1], z1 - v[2]))


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    base, lid = build_base(), build_lid()
    base.save(os.path.join(here, "case_base.stl"))
    lid.save(os.path.join(here, "case_lid.stl"))
    print(f"outside {OUT_W:.1f} x {OUT_H:.1f} x {Z_TOP + LID_T:.1f} mm")
    print(f"base: {len(base.tris)} triangles, {base.volume() / 1000:.1f} cm3")
    print(f"lid:  {len(lid.tris)} triangles, {lid.volume() / 1000:.1f} cm3")
