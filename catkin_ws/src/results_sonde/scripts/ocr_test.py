#!/usr/bin/env python3
import os, time, re
from datetime import datetime
import pytesseract
from PIL import Image
import mss


#OBS muss oben in die linke ecke plaziert werden, mit der usprungsgröße wenn ich es lffne, dann ist die REGIOn richtig ausgerichtet 
# Tesseract: unter Ubuntu meist /usr/bin/tesseract
pytesseract.pytesseract.tesseract_cmd = os.environ.get("TESSERACT_CMD", "/usr/bin/tesseract")

# speichern
screenshot_dir = os.path.join(os.path.expanduser("~"), "catkin_ws", "raster_results", "screenshots")
os.makedirs(screenshot_dir, exist_ok=True)

# Bereich wählen (anpassen!)
REGION = {"top": 100, "left": 300, "width": 800, "height": 500}

def ocr_loop():
    with mss.mss() as sct:
        # Monitore anzeigen, hilft bei der Region-Wahl
        print("[INFO] Monitors:", sct.monitors)  # monitors[1] = gesamter Desktop

        while True:
            sct_img = sct.grab(REGION)
            # mss liefert RGB-Bytes direkt
            img = Image.frombytes("RGB", sct_img.size, sct_img.rgb)

            # Vorverarbeitung
            img_gray = img.convert("L")
            img_bw   = img_gray.point(lambda x: 0 if x < 128 else 255, "1")
            img_scaled = img_bw.resize((img_bw.width * 2, img_bw.height * 2))

            # OCR
            text = pytesseract.image_to_string(
                img_scaled,
                config="--psm 6 -c tessedit_char_whitelist=0123456789:."
            ).strip()
            print(f"[DEBUG] OCR-Text: '{text}'")

            # Screenshot speichern
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_path = os.path.join(screenshot_dir, f"screenshot_{ts}.png")
            img_scaled.save(out_path)
            print(f"📸 Gespeichert: {out_path}")

            # Uhrzeit erkennen (z.B. 20:45)
            m_time = re.search(r"\d{1,2}:\d{2}", text)
            if m_time:
                print(f"[OCR] 🕒 Uhrzeit: {m_time.group(0)}")
            else:
                print("[OCR] ❌ Keine Uhrzeit erkannt.")

            # Datum erkennen (z.B. 04.05.2025)
            m_date = re.search(r"\d{1,2}\.\d{1,2}\.\d{4}", text)
            if m_date:
                print(f"[OCR] 📅 Datum: {m_date.group(0)}")
            else:
                print("[OCR] ❌ Kein Datum erkannt.")

            time.sleep(2)

if __name__ == "__main__":
    ocr_loop()
