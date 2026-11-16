#!/usr/bin/env python3
"""
Builds everything a fab needs from the generated KiCad files:
    python make_outputs.py
  ../fab/gerbers/*            Gerbers + drill
  ../fab/lipo-charger-gerbers.zip
  ../fab/BOM_JLCPCB.csv       Comment, Designator, Footprint, LCSC Part #
  ../fab/CPL_JLCPCB.csv       pick-and-place (top side)
  ../docs/schematic.pdf
  drc.json / erc.json         rule-check reports
"""
import csv
import json
import os
import shutil
import subprocess
import zipfile

import design

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
KC = r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"
PCB = os.path.join(HERE, "lipo-charger.kicad_pcb")
SCH = os.path.join(HERE, "lipo-charger.kicad_sch")
FAB = os.path.join(ROOT, "fab")
GERB = os.path.join(FAB, "gerbers")
DOCS = os.path.join(ROOT, "docs")


def run(*args):
    r = subprocess.run([KC, *args], capture_output=True, text=True)
    tail = (r.stdout + r.stderr).strip().splitlines()[-1:] or [""]
    print("  kicad-cli", " ".join(args[:3]), "->", tail[0][:110])
    return r


def bom():
    rows = {}
    for ref, p in design.PARTS.items():
        if p["dnp"]:
            continue
        key = (p["value"], p["fp"].split(":")[1], p["mpn"], p["lcsc"], p["tht"])
        rows.setdefault(key, []).append(ref)
    num = lambda r: (("".join(c for c in r if c.isalpha())), int("".join(c for c in r if c.isdigit())))
    with open(os.path.join(FAB, "BOM_JLCPCB.csv"), "w", newline="", encoding="utf8") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #", "MPN", "Qty", "Assembly"])
        for (value, fp, mpn, lcsc, tht), refs in sorted(rows.items(), key=lambda kv: num(sorted(kv[1], key=num)[0])):
            refs = sorted(refs, key=num)
            w.writerow([value, ",".join(refs), fp, lcsc, mpn, len(refs), "hand-solder (through-hole)" if tht else "SMT"])
    print("  BOM lines:", len(rows))


def cpl():
    tmp = os.path.join(FAB, "_pos.csv")
    run("pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "front",
        "--exclude-dnp", "-o", tmp, PCB)
    tht = {r for r, p in design.PARTS.items() if p["tht"]}
    with open(tmp, newline="", encoding="utf8") as f, \
            open(os.path.join(FAB, "CPL_JLCPCB.csv"), "w", newline="", encoding="utf8") as out:
        w = csv.writer(out)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        n = 0
        for row in csv.DictReader(f):
            if row["Ref"] in tht or row["Ref"].startswith("H"):
                continue
            w.writerow([row["Ref"], row["PosX"], row["PosY"], "Top", row["Rot"]])
            n += 1
    os.remove(tmp)
    print("  CPL rows:", n)


def main():
    shutil.rmtree(GERB, ignore_errors=True)
    os.makedirs(GERB, exist_ok=True)
    os.makedirs(DOCS, exist_ok=True)
    run("pcb", "export", "gerbers", "--layers",
        "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Mask,B.Mask,F.Paste,F.SilkS,B.SilkS,Edge.Cuts",
        "--subtract-soldermask", "-o", GERB + os.sep, PCB)
    run("pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th", "-o", GERB + os.sep, PCB)
    with zipfile.ZipFile(os.path.join(FAB, "lipo-charger-gerbers.zip"), "w", zipfile.ZIP_DEFLATED) as z:
        for name in sorted(os.listdir(GERB)):
            z.write(os.path.join(GERB, name), name)
    bom()
    cpl()
    run("sch", "export", "pdf", "-o", os.path.join(DOCS, "schematic.pdf"), SCH)
    run("pcb", "render", "--side", "top", "-w", "2400", "-h", "1600", "--background", "opaque",
        "-o", os.path.join(DOCS, "board-top.png"), PCB)
    run("sch", "erc", "--format", "json", "--severity-all", "-o", os.path.join(HERE, "erc.json"), SCH)
    run("pcb", "drc", "--format", "json", "--severity-all", "--schematic-parity",
        "-o", os.path.join(HERE, "drc.json"), PCB)
    d = json.load(open(os.path.join(HERE, "drc.json"), encoding="utf8"))
    kinds = {}
    for v in d.get("violations", []):
        kinds[(v["severity"], v["type"])] = kinds.get((v["severity"], v["type"]), 0) + 1
    print("DRC violations:", dict(sorted(kinds.items())) or "none")
    print("DRC unconnected:", len(d.get("unconnected_items", [])))
    print("DRC schematic parity issues:", len(d.get("schematic_parity", [])))


if __name__ == "__main__":
    main()
