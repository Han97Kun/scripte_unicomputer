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
DISCARD_HEAD = 4   # wie viele Messungen am Anfang ignorieren
DISCARD_TAIL = 2   # wie viele Messungen am Ende ignorieren

print(f"results_dir      = {RESULTS_DIR}")
print(f"screenshots_root = {SCREENSHOTS_ROOT}")
print(f"DISCARD_HEAD={DISCARD_HEAD}, DISCARD_TAIL={DISCARD_TAIL}")

# -----------------------------
# Hilfsfunktionen
# -----------------------------
def to_mm(val):
    """Heuristik: |val| <= 10 -> Meter (x1000), sonst schon mm."""
    try:
        v = float(val)
    except Exception:
        return None
    if abs(v) <= 10.0:
        return int(round(v * 1000.0))  # Meter -> mm
    return int(round(v))               # bereits mm

def xy_dirname_from_xy(x, y) -> str:
    """x{mm}mm_y{mm}mm, z.B. x350mm_y0mm."""
    x_mm = to_mm(x)
    y_mm = to_mm(y)
    if x_mm is None or y_mm is None:
        return ""
    return f"x{x_mm}mm_y{y_mm}mm"

def height_tag_from_z(z) -> str:
    """z-Tag ohne Suffix, z.B. 'z2mm'."""
    z_mm = to_mm(z)
    if z_mm is None:
        return ""
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
            nums = [int(m) for m in re.findall(r"-?\d+", raw_text)]
            v = nums[0] if nums else None
        if v is None or str(v).strip() == "":
            continue
        try:
            vals.append(int(v))
        except Exception:
            try:
                vals.append(int(float(v)))
            except Exception:
                continue

        t_iso = (row.get("t_iso") or "").strip()
        if "T" in t_iso:
            t_iso = t_iso.split("T")[-1]
        if "." in t_iso:
            t_iso = t_iso.split(".")[0]
        times.append(t_iso)

    return vals, times

def _list_csvs(folder: str):
    """Alle CSVs außer *_summary/_with_ocr."""
    return [
        p for p in glob.glob(os.path.join(folder, "*.csv"))
        if not (p.endswith("_summary.csv") or p.endswith("_with_ocr.csv"))
    ]

def find_series_csv_by_exact_token(z, x, y):
    """
    Suche unter screenshots/z{mm}*/x{..}mm_y{..}mm/ nach
    'raster_log_*_{z_token}.csv' (z_token = basename des z-Ordners, z.B. z2mm_1).
    Nimm die jüngste passende Datei. Fallback: irgendeine CSV im Positionsordner.
    """
    htag = height_tag_from_z(z)         # z.B. "z2mm"
    ptag = xy_dirname_from_xy(x, y)     # z.B. "x350mm_y0mm"
    if not htag or not ptag:
        return ""

    candidates = []
    # Alle z-Varianten (z2mm, z2mm_1, z2mm_2, ...):
    for z_dir in glob.glob(os.path.join(SCREENSHOTS_ROOT, f"{htag}*")):
        if not os.path.isdir(z_dir):
            continue
        z_token = os.path.basename(z_dir)  # z.B. "z2mm_1"
        pos_dir = os.path.join(z_dir, ptag)
        if not os.path.isdir(pos_dir):
            continue

        # 1) Bevorzugt: Namen wie 'raster_log_*_{z_token}.csv'
        pref = glob.glob(os.path.join(pos_dir, f"raster_log_*_{z_token}.csv"))
        if pref:
            candidates.extend(pref)
        else:
            # 2) Fallback: irgendeine CSV (ohne *_summary/_with_ocr)
            candidates.extend(_list_csvs(pos_dir))

    if not candidates:
        return ""
    candidates.sort(key=lambda p: os.path.getmtime(p))
    return candidates[-1]

def find_series_csv_flat(z):
    """
    Flaches Layout (falls CSVs direkt im z-Ordner ohne x/y liegen):
    - bevorzugt 'raster_log_*_{z_token}.csv'
    - sonst irgendeine CSV (ohne *_summary/_with_ocr)
    """
    htag = height_tag_from_z(z)
    if not htag:
        return ""
    matches = []
    for z_dir in glob.glob(os.path.join(SCREENSHOTS_ROOT, f"{htag}*")):
        if not os.path.isdir(z_dir):
            continue
        z_token = os.path.basename(z_dir)
        pref = glob.glob(os.path.join(z_dir, f"raster_log_*_{z_token}.csv"))
        if pref:
            matches.extend(pref)
        else:
            matches.extend(_list_csvs(z_dir))
    if not matches:
        return ""
    matches.sort(key=lambda p: os.path.getmtime(p))
    return matches[-1]

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
        fieldnames = list(reader.fieldnames or []) + ["ocr_raw", "ocr_mean", "ocr_n", "ocr_times"]
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            # Falls direkt angegeben, nutzen
            series_csv_path = (row.get("ocr_series_csv") or "").strip()

            if not series_csv_path:
                # x,y,z aus der Punkt-CSV
                try:
                    x = (row.get("x") or "").strip()
                    y = (row.get("y") or "").strip()
                    z = (row.get("z") or "").strip()
                except Exception:
                    x = y = z = ""

                path = ""
                # 1) Bevorzugt: Raster-Layout mit exaktem Z-Token im Dateinamen
                if x != "" and y != "" and z != "":
                    path = find_series_csv_by_exact_token(z, x, y)

                # 2) Fallback: flaches Layout im z-Ordner
                if not path and z != "":
                    path = find_series_csv_flat(z)

                series_csv_path = path

            vals, times = read_ocr_series_csv(series_csv_path)
            row["ocr_raw"]   = ", ".join(map(str, vals))
            row["ocr_mean"]  = f"{mean(vals):.2f}" if vals else ""
            row["ocr_n"]     = len(vals)
            row["ocr_times"] = ", ".join(times)
            writer.writerow(row)

print("\n✅ Fertig.")



