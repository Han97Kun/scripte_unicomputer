#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import csv
import glob
import math
import numpy as np
import matplotlib.pyplot as plt
from statistics import mean
from datetime import date, datetime


def activity_decay(A0_Bq: float, t_elapsed_days: float, T12_days: float = 271.74) -> float:
    """
    Zerfallsgesetz für die Aktivität.
    Korrekte Form: A(t) = A0 * exp(-ln(2) * t / T12)

    Parameter
    ---------
    A0_Bq : float
        Aktivität zum Referenzzeitpunkt t0 in Becquerel
    t_elapsed_days : float
        verstrichene Zeit (Tage) seit t0 bis zum Mess-/Nutzzeitpunkt
    T12_days : float
        Halbwertszeit (Tage)

    Rückgabe
    --------
    Aktivität zum Zeitpunkt t in Becquerel
    """
    lam = math.log(2.0) / T12_days
    return A0_Bq * math.exp(-lam * t_elapsed_days)

# --- Referenz: 25 µCi am 11.03.2025 (deutsches Datumsformat: TT.MM.JJJJ) ---
ref_date = date(2025, 3, 11)            # 11.03.2025
today    = date.today()                  # heutiges Datum (Systemdatum)
t_elapsed_days = (today - ref_date).days # Tage seit Referenz

# Umrechnung: 1 Ci = 3.7e10 Bq, 1 µCi = 1e-6 Ci
A0_uCi = 25.0
A0_Bq  = A0_uCi * 1e-6 * 3.7e10  # = 25 * 37000 = 925000 Bq

# Aktivität zum heutigen Datum per Zerfall
T12_days = 271.74
activity_Bq_today = activity_decay(A0_Bq, t_elapsed_days, T12_days)

# ---------------------------
# Modellkonstanten / Config
# ---------------------------
constants = {
    "mu": 0.19,              # Medium-Attenuation-Koeffizient (Einheiten zu Distanz beachten!)
    "rho": 0.001225,         # Dichte/zweiter Faktor (Einheiten kompatibel zu mu und Distanz)
    "activity_Bq": activity_Bq_today,  # zeitkorrigierte Aktivität (heute)
    "probe_surface": 1.131,  # cm² (achte auf Konsistenz zu Distanz-Einheiten im Modell)
    "fwhm_rad": 0.7505,      # FWHM des Sonden-Response in RAD
    "voxel_size": 0.25,      # cm (nur falls im restlichen Code verwendet)
    "C2": 0.0,               # Offset immer 0
}
                  # constant for the intensity calculation

def fm_ss_sm_batch(sources, probes, orientations, constants, noise=False):
    """
    Vectorized forward model for multiple sources and multiple measurements.
    
    Parameters:
        sources: (M, 3) array of source positions
        probes: (N, 3) array of probe positions
        orientations: (N, 3) array of probe orientation vectors
        constants: dict with same keys as original
    
    Returns:
        intensities: (N,) array of total intensity at each probe (sum over sources)
    """
    mu = constants['mu']
    rho = constants['rho']
    activity_Bq = constants['activity_Bq']
    probe_surface = constants['probe_surface']
    fwhm_rad = constants['fwhm_rad']
    C1 = constants['C1']
    C2 = constants['C2']

    if noise: # if we calculate intensity from noise sources, let their activity be 1/100 of the point sources
        C1 = C1 * 0.01
    
    N = probes.shape[0]
    M = sources.shape[0]
    
    # Expand dimensions for broadcasting: (N, 1, 3) - (1, M, 3) = (N, M, 3)
    delta = sources[None, :, :] - probes[:, None, :]  # (N, M, 3)
    distances = np.linalg.norm(delta, axis=2)  # (N, M)
    
    # Compute angle between delta and orientation
    # Normalize vectors
    delta_unit = delta / (distances[..., None] + 1e-8)  # (N, M, 3)
    orient_norm = orientations / (np.linalg.norm(orientations, axis=1, keepdims=True) + 1e-8)  # (N, 3)
    orient_norm_exp = orient_norm[:, None, :]  # (N, 1, 3)
    cos_angle = np.clip(np.sum(delta_unit * orient_norm_exp, axis=2), -1.0, 1.0)  # (N, M)
    angle = np.arccos(cos_angle)  # (N, M)

    # Angle dependence
    angle_dep = np.exp(-4 * np.log(2) * (angle / fwhm_rad) ** 2)  # (N, M)

    # Distance dependence
    dist_dep = probe_surface / (4 * np.pi * distances ** 2 )  # (N, M)

    # Exponential attenuation
    attenuation = np.exp(-mu * rho * distances)  # (N, M)

    # Stochastic term: (N, M) from normal distribution
    #stoch = np.random.normal(loc=0.9, scale=0.05, size=(N, M))

    # Total intensity per source
    intensities = C1  * activity_Bq * attenuation * angle_dep * dist_dep + C2 # stoch

    # Sum over all sources
    return intensities.sum(axis=1)  # (N,)



# ============================
# Daten laden
# ============================

def parse_ts_maybe(s: str):
    s = (s or "").strip()
    if not s:
        return None
    for fmt in ("%Y-%m-%d %H:%M:%S.%f",
                "%Y-%m-%d %H:%M:%S",
                "%Y-%m-%dT%H:%M:%S.%f",
                "%Y-%m-%dT%H:%M:%S"):
        try:
            return datetime.strptime(s, fmt)
        except Exception:
            pass
    try:
        return datetime.fromisoformat(s)
    except Exception:
        return None

HOME = os.path.expanduser("~")
RESULTS_DIR = os.environ.get("RESULTS_DIR", os.path.join(HOME, "catkin_ws", "raster_results"))

def load_measurements(results_dir):
    """
    Erwartet *_with_ocr.csv (empfohlen). Nutzt Spalten:
      x, y, z (oder z_abs), ocr_mean, optional ox,oy,oz (sonst [0,0,-1]).
    Zeiten werden hier nicht mehr gebraucht (Aktivität ist schon "heute").
    """
    files = sorted(glob.glob(os.path.join(results_dir, "*_with_ocr.csv")))
    if not files:
        files = sorted(
            f for f in glob.glob(os.path.join(results_dir, "*.csv"))
            if not f.endswith("_summary.csv")
        )
        print("⚠️ Keine *_with_ocr.csv gefunden – verwende rohe CSVs:", [os.path.basename(f) for f in files])

    probes, orients, y_vals = [], [], []

    for fpath in files:
        with open(fpath, newline="") as f:
            r = csv.DictReader(f)
            for row in r:
                y_str = (row.get("ocr_mean") or "").strip()
                if y_str == "":
                    continue
                try:
                    y_val = float(y_str)
                except ValueError:
                    continue

                def getf(key):
                    v = row.get(key)
                    return float(v) if (v is not None and v != "") else None

                x = getf("x"); y_pos = getf("y")
                z_abs = getf("z_abs"); z_rel = getf("z")
                if x is None or y_pos is None:
                    continue
                z = z_abs if z_abs is not None else (z_rel if z_rel is not None else None)
                if z is None:
                    continue

                ox = getf("ox"); oy = getf("oy"); oz = getf("oz")
                if ox is None or oy is None or oz is None:
                    ori = [0.0, 0.0, -1.0]
                else:
                    ori = [ox, oy, oz]

                probes.append([x, y_pos, z])
                orients.append(ori)
                y_vals.append(y_val)

    if not probes:
        raise RuntimeError("Keine gültigen Messungen (x,y,z, ocr_mean) gefunden.")

    return np.asarray(probes, float), np.asarray(orients, float), np.asarray(y_vals, float)

# ============================
# Least Squares (C2 = 0)
# ============================

def fit_C1_through_origin(g, y):
    """C1 = (g^T y)/(g^T g)"""
    denom = float(np.dot(g, g))
    if denom == 0.0:
        raise ZeroDivisionError("Summe g^2 = 0 (Geometrie/Aktivität liefert g=0).")
    return float(np.dot(g, y) / denom)

# ============================
# MAIN
# ============================

if __name__ == "__main__":
    print(f"Heute:               {today.isoformat()}")
    print(f"Referenzdatum:       {ref_date.isoformat()}")
    print(f"Tage seit Referenz:  {t_elapsed_days} d")
    print(f"A0 (am t0):          {A0_Bq:.0f} Bq (25 µCi)")
    print(f"A(t=heute):          {activity_Bq_today:.2f} Bq")

    # ---- Quellenposition(en): ANPASSEN! ----
    # Beispiel: eine Quelle im Ursprung
    sources = np.array([[0.0, 0.0, 0.0]], dtype=float)

    # ---- Daten laden ----
    probes, orients, y = load_measurements(RESULTS_DIR)

    # ---- g berechnen (ohne C1/C2/Rauschen) ----
    g = fm_ss_sm_batch(sources, probes, orients, constants)

    # ---- C1 schätzen (C2=0 fest) ----
    C1_hat = fit_C1_through_origin(g, y)
    y_hat = C1_hat * g
    resid = y - y_hat
    rmse = float(np.sqrt(np.mean(resid**2)))
    mae  = float(np.mean(np.abs(resid)))

    print("\n=== Kalibrierung (C2 = 0) ===")
    print(f"C1_hat = {C1_hat:.6g}")
    print(f"RMSE   = {rmse:.6g}")
    print(f"MAE    = {mae:.6g}")

    # ============================
    # Plots der Ungenauigkeiten
    # ============================

    # 1) Parity-Plot: gemessen vs. vorhergesagt
    plt.figure()
    plt.scatter(y, y_hat, s=12)
    # Identitätslinie
    lo = min(float(np.min(y)), float(np.min(y_hat)))
    hi = max(float(np.max(y)), float(np.max(y_hat)))
    plt.plot([lo, hi], [lo, hi])
    plt.xlabel("gemessen (ocr_mean)")
    plt.ylabel("vorhergesagt (C1*g)")
    plt.title("Parity-Plot (Perfekt: Diagonale)")
    plt.tight_layout()

    # 2) Residuen vs. Vorhersage
    plt.figure()
    plt.scatter(y_hat, resid, s=12)
    plt.axhline(0.0)
    plt.xlabel("vorhergesagt (C1*g)")
    plt.ylabel("Residuum (y - y_hat)")
    plt.title(f"Residuen (RMSE={rmse:.3g}, MAE={mae:.3g})")
    plt.tight_layout()

    # 3) Histogramm der Residuen
    plt.figure()
    plt.hist(resid, bins=30)
    plt.xlabel("Residuum")
    plt.ylabel("Häufigkeit")
    plt.title("Histogramm der Residuen")
    plt.tight_layout()

    plt.show()