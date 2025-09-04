#!/usr/bin/env python3
import os, csv, glob, re
from statistics import mean

# --- Wurzeln ---
HOME = os.path.expanduser("~")
results_dir = os.environ.get("RESULTS_DIR", os.path.join(HOME, "catkin_ws", "raster_results"))
screenshots_root = os.path.join(results_dir, "screenshots")

print(f"results_dir      = {results_dir}")
print(f"screenshots_root = {screenshots_root}")

# --- Hilfen ---
def xy_dirname_from_xy(x: float, y: float, unit="mm") -> str:
    if unit == "mm":
        xi = int(round(float(x) * 1000.0))
        yi = int(round(float(y) * 1000.0))
        return f"x{xi}mm_y{yi}mm"
    xs = f"{float(x):.3f}".replace('.', '_')
    ys = f"{float(y):.3f}".replace('.', '_')
    return f"x{xs}m_y{ys}m"

def height_tag_from_z(z) -> str:
    # z kommt aus der Raster-CSV typischerweise in Metern (z.B. 0.150)
    z_mm = int(round(float(z) * 1000.0))
    return f"z{z_mm}mm"

def extract_cps_from_text(text: str):
    """Heuristik: CPS-Wert(e) aus OCR-Text ziehen; nimm den größten plausiblen."""
    if not text:
        return None
    nums3 = [int(m) for m in re.findall(r"\b\d{0,6}\b", text)]
    if nums3:
        return max(nums3)
    nums2 = [int(m) for m in re.findall(r"\b\d+\b", text) if int(m) > 20]
    return max(nums2) if nums2 else None

def read_ocr_series_csv(series_csv_path: str):
    vals, times = [], []
    if not series_csv_path or not os.path.isfile(series_csv_path):
        return vals, times
    with open(series_csv_path, newline="") as f:
        r = csv.DictReader(f)
        rows = list(r)
        # optional: erste/letzte 2 Messungen verwerfen (Auf- & Abbau)
        if len(rows) > 4:
            rows = rows[2:-2]
        for row in rows:
            raw_text = (row.get("ocr_text") or "").strip()
            #v = extract_cps_from_text(raw_text)
            v = row.get("extracted_number")
            if v is not None and v != "":
                vals.append(int(v))
                times.append(row.get("t_iso", ""))
            #if v is not None:
                #vals.append(v)
                #times.append(row.get("t_iso", ""))
    return vals, times

def find_series_csv_via_folder(z, x, y):
    """Falls Spalte 'ocr_series_csv' fehlt/leer: über Ordnerstruktur suchen."""
    htag = height_tag_from_z(z)                       # z150mm
    ptag = xy_dirname_from_xy(x, y, unit="mm")        # x350mm_y0mm
    pos_dir = os.path.join(screenshots_root, htag, ptag)
    if not os.path.isdir(pos_dir):
        return ""
    candidates = sorted(glob.glob(os.path.join(pos_dir, "*_ocr.csv")))
    if not candidates:
        return ""
    # nimm die neueste Serie in diesem Positionsordner
    candidates.sort(key=lambda p: os.path.getmtime(p))
    return candidates[-1]

# --- Alle Punkt-CSV-Dateien laden (keine Summaries / keine bereits veredelten) ---
point_csvs = sorted(
    p for p in glob.glob(os.path.join(results_dir, "*.csv"))
    if not p.endswith("_summary.csv") and not p.endswith("_with_ocr.csv")
)
if not point_csvs:
    raise FileNotFoundError(f"Keine Punkt-CSVs in {results_dir} gefunden.")

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
            # hole Pfad zur OCR-Serie, bevorzugt aus CSV-Spalte
            series_csv_path = (row.get("ocr_series_csv") or "").strip()

            # fallback: per Ordnerstruktur suchen
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
            row["ocr_times"] = ", ".join(
                t.split("T")[-1].split(".")[0] if "T" in t else t for t in times
            )
            writer.writerow(row)

print("\n✅ Fertig.")
