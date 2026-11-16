#!/usr/bin/env python3
r"""
Writes the project footprint library charger.pretty and fp-lib-table.
Only needed again if KiCad's own footprints change; the results are committed.
    "C:\Program Files\KiCad\10.0\bin\python.exe" make_footprints.py
(Kept apart from gen_pcb.py: saving a footprint library leaves KiCad's Python module unable to
load boards in the same process.)
"""
import os

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
FP_ROOT = os.path.join("C:" + os.sep, "Program Files", "KiCad", "10.0", "share", "kicad", "footprints")
LOCAL_LIB = os.path.join(HERE, "charger.pretty")


def main():
    """Write the two footprints this board changes into the project library.

    A footprint on the board that differs from its library copy is a DRC error, so the changed
    versions get their own library entries instead of being edited after loading:
      BQ29209 VSON-8        silkscreen outline moved to F.Fab (it collides with the connectors)
      ESP32-C3-WROOM-02     courtyard trimmed to the module body, so the antenna can sit at the board edge
    """
    os.makedirs(LOCAL_LIB, exist_ok=True)
    jobs = (("Package_SON", "VSON-8-1EP_3x3mm_P0.65mm_EP1.65x2.4mm", "BQ29209_VSON-8-1EP_3x3mm_P0.65mm_EP1.65x2.4mm"),
            ("RF_Module", "ESP32-C3-WROOM-02", "ESP32-C3-WROOM-02_board-edge"))
    plug = pcbnew.GetPluginForPath(LOCAL_LIB)       # fetched once: a second lookup returns an unusable object
    for lib, name, new in jobs:
        fp = pcbnew.FootprintLoad(os.path.join(FP_ROOT, lib + ".pretty"), name)
        if new.startswith("BQ29209"):
            for item in fp.GraphicalItems():
                if item.GetLayer() == pcbnew.F_SilkS:
                    item.SetLayer(pcbnew.F_Fab)
        else:
            for item in list(fp.GraphicalItems()):
                if item.GetLayer() == pcbnew.F_CrtYd:
                    fp.Remove(item)
            box = pcbnew.PCB_SHAPE(fp)
            box.SetShape(pcbnew.SHAPE_T_RECT)
            box.SetLayer(pcbnew.F_CrtYd)
            box.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(-9.25), pcbnew.FromMM(-12.8)))
            box.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(9.25), pcbnew.FromMM(7.4)))
            box.SetWidth(pcbnew.FromMM(0.05))
            fp.Add(box)
        fp.SetFPID(pcbnew.LIB_ID("charger", new))
        plug.FootprintSave(LOCAL_LIB, fp)
    table = os.path.join(HERE, "fp-lib-table")
    with open(table, "w", encoding="utf8", newline="\n") as f:
        f.write('(fp_lib_table\n  (version 7)\n  (lib (name "charger")(type "KiCad")'
                '(uri "${KIPRJMOD}/charger.pretty")(options "")(descr "Footprints changed for this board"))\n)\n')
    print("wrote", LOCAL_LIB)


if __name__ == "__main__":
    main()
