#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import glob
import re
from statistics import mean

# -----------------------------
# Pfade
# -----------------------------
HOME = os.path.expanduser("~")
RESULTS_DIR = os.environ.get("RESULTS_DIR", os.path.join(HOME, "catkin_ws", "raster_results"))
SCREENSHOTS_ROOT = os.path.join(RESULTS_DIR, "screenshots")

# -----------------------------
# Discard-Einstellungen
# -----------------------------
DISCARD_HEAD = 5   # wie viele Messungen am Anfang ignorieren
DISCARD_TAIL = 5   # wie viele Messungen am Ende ignorieren

print(f"results_dir      = {RESULTS_DIR}")
print(f"screenshots_root = {SCREENSHOTS_ROOT}")
print(f"DISCARD_HEAD={DISCARD_HEAD}, DISCARD_TAIL={DISCARD_TAIL}")

# -----------------------------
# Hilfsfunktionen
# -----------------------------
def xy_dirname_from_xy(x: float, y: float, unit="mm") -> str:
    if unit == "mm":
        xi = int(round(float(x) * 1000.0))
        yi = int(round(float(y) * 1000.0))
        return f"x{xi}mm_y{yi}mm"
    xs = f"{float(x):.3f}".replace('.', '_')
    ys = f"{float(y):.3f}".replace('.', '_')
    return f"x{xs}m_y{ys}m"

def height_tag_from_z(z) -> str:
    z_mm = int(round(float(z) * 1000.0))
    return f"z{z_mm}mm"

def read_ocr_series_csv(series_csv_path: str):
    vals, times = [], []
    if not series_csv_path or not os.path.isfile(series_csv_path):
        return vals, times

    with open(series_csv_path, newline="") as f:
        r = csv.DictReader(f)
        rows = list(r)

    # Erste/letzte N Zeilen wegschneiden
    if len(rows) > (DISCARD_HEAD + DISCARD_TAIL):
        rows = rows[DISCARD_HEAD: len(rows) - DISCARD_TAIL]

    for row in rows:
        v = row.get("extracted_number")
        if v is None or str(v).strip() == "":
            raw_text = (row.get("ocr_text") or "").strip()
            nums = [int(m) for m in re.findall(r"\d+", raw_text)]
            v = nums[0] if nums else None
        if v is None or str(v).strip() == "":
            continue
        vals.append(int(v))
        t_iso = row.get("t_iso", "")
        if "T" in t_iso:
            t_iso = t_iso.split("T")[-1]
        if "." in t_iso:
            t_iso = t_iso.split(".")[0]
        times.append(t_iso)

    return vals, times

def find_series_csv_via_folder(z, x=None, y=None):
    #Suche im Layout: screenshots/z{mm}_*/<series>.csv (ohne x/y-Unterordner).#
    htag = height_tag_from_z(z)  # z.B. "z3mm"
    base_dir = os.path.join(SCREENSHOTS_ROOT, f"{htag}_*")
    matches = []
    for d in glob.glob(base_dir):
        if os.path.isdir(d):
            # früher: "*_ocr.csv"  -> findet nichts, wenn "_ocr" fehlt
            for p in glob.glob(os.path.join(d, "*.csv")):
                name = os.path.basename(p)
                if name.endswith("_summary.csv") or name.endswith("_with_ocr.csv"):
                    continue
                matches.append(p)

    if not matches:
        return ""
    matches.sort(key=lambda p: os.path.getmtime(p))
    return matches[-1]

def find_series_csv_via_folder_raster(z, x, y): #wenn ich das rechteckige raster fahre
    htag = height_tag_from_z(z)
    ptag = xy_dirname_from_xy(x, y, unit="mm")
    pos_dir = os.path.join(SCREENSHOTS_ROOT, htag, ptag)
    if not os.path.isdir(pos_dir):
        return ""
    candidates = sorted(glob.glob(os.path.join(pos_dir, "*_ocr.csv")))
    if not candidates:
        return ""
    candidates.sort(key=lambda p: os.path.getmtime(p))
    return candidates[-1]

# -----------------------------
# Hauptlogik
# -----------------------------
point_csvs = sorted(
    p for p in glob.glob(os.path.join(RESULTS_DIR, "*.csv"))
    if not p.endswith("_summary.csv") and not p.endswith("_with_ocr.csv")
)

if not point_csvs:
    raise FileNotFoundError(f"Keine Punkt-CSVs in {RESULTS_DIR} gefunden.")

print("Gefundene CSVs:")
for p in point_csvs:
    print("  •", os.path.basename(p))

for input_csv in point_csvs:
    out_csv = input_csv.replace(".csv", "_with_ocr.csv")
    print(f"\n➡ Verarbeite: {os.path.basename(input_csv)}")
    print(f"  ↳ Schreibe : {os.path.basename(out_csv)}")

    with open(input_csv, newline="") as infile, open(out_csv, "w", newline="") as outfile:
        reader = csv.DictReader(infile)
        fieldnames = reader.fieldnames + ["ocr_raw", "ocr_mean", "ocr_n", "ocr_times"]
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            series_csv_path = (row.get("ocr_series_csv") or "").strip()
            if not series_csv_path:
                try:
                    x = float(row.get("x", ""))
                    y = float(row.get("y", ""))
                    z = float(row.get("z", ""))
                except Exception:
                    x = y = z = None
                if x is not None and y is not None and z is not None:
                    series_csv_path = find_series_csv_via_folder(z, x, y)

            vals, times = read_ocr_series_csv(series_csv_path)
            row["ocr_raw"]   = ", ".join(map(str, vals))
            row["ocr_mean"]  = f"{mean(vals):.2f}" if vals else ""
            row["ocr_n"]     = len(vals)
            row["ocr_times"] = ", ".join(times)
            writer.writerow(row)

print("\n✅ Fertig.")
"""

#!/usr/bin/env python3
import os
import shutil
import glob

# Basisordner mit den z*-Unterordnern
BASE_DIR   = "/home/hannah_kunze/catkin_ws/raster_results/finale_Rasterscans/no_2"

# Parametrisierung
BATCH_SIZE = 24                 # 24 Bilder pro y-Ordner
X_MM       = 350                # x350mm
SORT_BY    = "name"             # "name" oder "mtime"
IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".tif", ".tiff", ".bmp")

def collect_images(dirpath):
    #Alle Bilddateien in einem Ordner sammeln (nicht rekursiv).
    entries = []
    with os.scandir(dirpath) as it:
        for e in it:
            if e.is_file() and e.name.lower().endswith(IMAGE_EXTS):
                entries.append(e)
    return entries

def make_unique_path(dst):
    if not os.path.exists(dst):
        return dst
    root, ext = os.path.splitext(dst)
    k = 1
    while True:
        cand = f"{root}_{k}{ext}"
        if not os.path.exists(cand):
            return cand
        k += 1

def process_height_dir(z_dir):
    # Bilder einsammeln
    entries = collect_images(z_dir)
    if not entries:
        print(f"  ⚠️  Keine Bilder in {os.path.basename(z_dir)} gefunden.")
        return

    # Sortierung
    if SORT_BY == "mtime":
        entries.sort(key=lambda e: e.stat().st_mtime)  # älteste zuerst
    else:
        entries.sort(key=lambda e: e.name)             # alphabetisch

    total = len(entries)
    print(f"  ✅ {os.path.basename(z_dir)}: {total} Bilder gefunden.")

    # In 24er-Blöcke in x350mm_y{n}mm verschieben (beginnend bei y0mm)
    for idx, entry in enumerate(entries):
        batch_num = idx // BATCH_SIZE              # 0,1,2,...
        target_dir = os.path.join(z_dir, f"x{X_MM}mm_y{batch_num}mm")
        os.makedirs(target_dir, exist_ok=True)

        src = entry.path
        dst = os.path.join(target_dir, entry.name)
        dst = make_unique_path(dst)

        shutil.move(src, dst)

    print(f"  🎉 Fertig: {os.path.basename(z_dir)} in 24er-Gruppen sortiert.")

def main():
    if not os.path.isdir(BASE_DIR):
        print(f"❌ Verzeichnis existiert nicht: {BASE_DIR}")
        return

    # Alle z-Ordner in no_1 (z.B. z2mm_1, z3mm_1, …)
    z_dirs = [d for d in glob.glob(os.path.join(BASE_DIR, "z*mm_*")) if os.path.isdir(d)]
    if not z_dirs:
        print("❌ Keine z*-Unterordner gefunden (z.B. z2mm_1).")
        return

    print(f"Gefundene Höhen-Ordner: {[os.path.basename(d) for d in z_dirs]}")
    for z_dir in sorted(z_dirs):
        process_height_dir(z_dir)

    print("✅ Alles erledigt.")

if __name__ == "__main__":
    main()

"""