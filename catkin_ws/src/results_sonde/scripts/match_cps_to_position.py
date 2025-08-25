#!/usr/bin/env python3
'''

import csv, os, re, glob
from datetime import datetime
from PIL import Image
import pytesseract

# --- Tesseract unter Linux automatisch finden ---
# Falls du einen eigenen Pfad willst: export TESSERACT_CMD=/usr/local/bin/tesseract
pytesseract.pytesseract.tesseract_cmd = os.environ.get(
    "TESSERACT_CMD",
    "/usr/bin/tesseract" if os.path.exists("/usr/bin/tesseract") else "tesseract"
)

# --- Verzeichnisse (Linux-Defaults), per ENV übersteuerbar ---
HOME = os.path.expanduser("~")
results_dir = os.environ.get("RESULTS_DIR", os.path.join(HOME, "catkin_ws", "raster_results"))
screenshot_dir = os.environ.get("SCREENSHOT_DIR", os.path.join(results_dir, "screenshots"))
os.makedirs(screenshot_dir, exist_ok=True)

print(f"📁 results_dir   = {results_dir}")
print(f"🖼  screenshot_dir= {screenshot_dir}")

# --- CSV finden ---
csv_files = glob.glob(os.path.join(results_dir, "*.csv"))
if not csv_files:
    raise FileNotFoundError(f"❌ Keine .csv in {results_dir} gefunden.")
if len(csv_files) > 1:
    raise RuntimeError(f"⚠ Mehrere .csv in {results_dir} gefunden – bitte nur eine belassen.")
input_csv = csv_files[0]
output_csv = input_csv.replace(".csv", "_with_avg_ocr.csv")
print(f"📄 Eingabe: {input_csv}")
print(f"💾 Ausgabe: {output_csv}")

# --- Screenshots einsammeln ---
all_screens = sorted(glob.glob(os.path.join(screenshot_dir, "screenshot_*.png")))
print(f"🔍 {len(all_screens)} Screenshots gefunden.")

def extract_timestamp_from_filename(path):
    m = re.search(r'screenshot_(\d{8}_\d{6})', os.path.basename(path))
    return datetime.strptime(m.group(1), "%Y%m%d_%H%M%S") if m else None

def extract_ocr_value(image_path):
    try:
        img = Image.open(image_path)
        img_gray = img.convert("L")
        img_bw = img_gray.point(lambda x: 0 if x < 128 else 255, '1')
        img_scaled = img_bw.resize((img_bw.width * 2, img_bw.height * 2))
        text = pytesseract.image_to_string(
            img_scaled, config='--psm 6 -c tessedit_char_whitelist=0123456789:.'
        ).strip()
        matches = re.findall(r'\d+', text)
        if matches:
            return int(''.join(matches)), text
    except Exception as e:
        print(f"[Fehler bei {image_path}]: {e}")
    return None, ""

remaining_screens = all_screens.copy()

with open(input_csv, newline='') as infile, open(output_csv, "w", newline='') as outfile:
    reader = csv.DictReader(infile)
    fieldnames = reader.fieldnames + ["ocr_raw", "ocr_mean", "ocr_times"]
    writer = csv.DictWriter(outfile, fieldnames=fieldnames)
    writer.writeheader()

    for row in reader:
        try:
            t_start = datetime.strptime(row.get("time_start",""), "%Y-%m-%d %H:%M:%S")
            t_end   = datetime.strptime(row.get("time_end",""),   "%Y-%m-%d %H:%M:%S")
        except Exception:
            row.update({"ocr_raw":"", "ocr_mean":"", "ocr_times":""})
            writer.writerow(row); continue

        matching = [p for p in remaining_screens
                    if (ts := extract_timestamp_from_filename(p)) and t_start <= ts <= t_end]
        matching.sort(key=extract_timestamp_from_filename)
        usable = matching[2:-2] if len(matching) > 4 else []

        ocr_values, raw_texts, times = [], [], []
        for p in usable:
            val, raw = extract_ocr_value(p)
            if val is not None:
                ocr_values.append(val)
                raw_texts.append(str(val))
                ts = extract_timestamp_from_filename(p)
                times.append(ts.strftime("%H:%M:%S"))
                remaining_screens.remove(p)

        row["ocr_raw"]  = ", ".join(raw_texts)
        row["ocr_mean"] = round(sum(ocr_values)/len(ocr_values), 2) if ocr_values else ""
        row["ocr_times"]= ", ".join(times)
        writer.writerow(row)

print("\n✅ Fertig!")
print(f"→ {output_csv}")
'''
#!/usr/bin/env python3
import os, csv, glob, re
from statistics import mean

# --- Wurzeln ---
HOME = os.path.expanduser("~")
results_dir = os.environ.get("RESULTS_DIR", os.path.join(HOME, "catkin_ws", "raster_results"))
screenshots_root = os.path.join(results_dir, "screenshots")

print(f"📁 results_dir      = {results_dir}")
print(f"🖼  screenshots_root = {screenshots_root}")

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
    nums3 = [int(m) for m in re.findall(r"\b\d{3,}\b", text)]
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
            v = extract_cps_from_text(raw_text)
            if v is not None:
                vals.append(v)
                times.append(row.get("t_iso", ""))
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
