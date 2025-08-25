#!/usr/bin/env python3
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
