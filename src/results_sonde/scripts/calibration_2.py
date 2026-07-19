#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import csv
import glob
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import date, datetime



# TODO
# nicht jeden tag die gleiche aktivitä, kann ich das anpassen 
# luft massenschwächung


# -------------------------------
# Zerfallsgesetz & Aktivität heute
# -------------------------------
def activity_decay(A0_Bq: float, t_elapsed_days: float, T12_days: float = 271.74) -> float:
    lam = math.log(2.0) / T12_days
    return A0_Bq * math.exp(-lam * t_elapsed_days)

ref_date = date(2025, 3, 11)   # 11.03.2025
today    = date.today()
t_elapsed_days = (today - ref_date).days

# 25 µCi -> Bq (1 Ci = 3.7e10 Bq; 1 µCi = 1e-6 Ci)
A0_uCi = 25.0
A0_Bq  = A0_uCi * 1e-6 * 3.7e10  # 925000 Bq
T12_days = 271.74
activity_Bq_today = activity_decay(A0_Bq, t_elapsed_days, T12_days)

# -------------------------------
# Modellkonstanten (C2=0 fix)
# -------------------------------
constants = {
    "mu": 0.17,
    "rho": 0.001225,
    "activity_Bq": activity_Bq_today,
    "probe_surface": 1.131,  # cm^2 (Einheitenkonsistenz beachten)
    "fwhm_rad": 0.7505,
    "voxel_size": 0.25,      # cm (nur falls genutzt)
    "C2": 0.0,
}

# -------------------------------
# Dein Forward-Model (ohne stoch)
# -------------------------------
def fm_ss_sm_batch(sources, probes, orientations, constants, noise=False):
    mu = constants['mu']
    rho = constants['rho']
    activity_Bq = constants['activity_Bq']
    probe_surface = constants['probe_surface']
    fwhm_rad = constants['fwhm_rad']
    C1 = constants['C1']
    C2 = constants['C2']

    # KEIN stochastik-term für Kalibrierung
    N = probes.shape[0]
    # (N, M, 3)
    delta = sources[None, :, :] - probes[:, None, :]
    distances = np.linalg.norm(delta, axis=2)

    delta_unit = delta / (distances[..., None] + 1e-12)
    orient_norm = orientations / (np.linalg.norm(orientations, axis=1, keepdims=True) + 1e-12)
    orient_norm_exp = orient_norm[:, None, :]

    cos_angle = np.clip(np.sum(delta_unit * orient_norm_exp, axis=2), -1.0, 1.0)
    angle = np.arccos(cos_angle)

    angle_dep = np.exp(-4.0 * np.log(2.0) * (angle / fwhm_rad) ** 2)
    dist_dep  = probe_surface / (4.0 * np.pi * np.maximum(distances, 1e-12) ** 2)
    attenuation = np.exp(-mu * rho * distances)

    intensities = C1 * activity_Bq * attenuation * angle_dep * dist_dep + C2
    return intensities.sum(axis=1)  # (N,)

def compute_g(sources, probes, orientations, constants):
    """g_n = Modell ohne C1/C2 (wir setzen C1=1, C2=0)."""
    consts = dict(constants)
    consts['C1'] = 1.0
    consts['C2'] = 0.0
    return fm_ss_sm_batch(sources, probes, orientations, consts)

# -------------------------------
# Daten laden
# -------------------------------
HOME = os.path.expanduser("~")
RESULTS_DIR = os.environ.get("RESULTS_DIR", os.path.join(HOME, "catkin_ws", "raster_results", "Ergebnisse"))
OCR_INTERVAL_S_DEFAULT = float(os.environ.get("OCR_INTERVAL_S", "0.5"))  # ~3 Hz default

def _f(s):
    """float-parser mit Dezimal-Komma-Toleranz."""
    if s is None: return None
    s = str(s).strip().replace(",", ".")
    if not s: return None
    try: return float(s)
    except: return None

def _parse_ts(s):
    """Zeitstempel robust parsen (CSV: t_start / t_end)."""
    if not s: return None
    s = str(s).strip()
    for fmt in ("%Y-%m-%d %H:%M:%S.%f",
                "%Y-%m-%d %H:%M:%S",
                "%Y-%m-%dT%H:%M:%S.%f",
                "%Y-%m-%dT%H:%M:%S"):
        try:
            return datetime.strptime(s, fmt)
        except:
            pass
    try:
        return datetime.fromisoformat(s)
    except:
        return None
    
def load_measurements(results_dir):
    """
    Erwartet *_with_ocr.csv 
    Nutzt: x,y,z(/z_abs), ocr_mean, optional duration_s/measurement_time_seconds/ocr_n.
    """
    files = sorted(glob.glob(os.path.join(results_dir, "*_with_ocr.csv")))
    if not files:
        print("Keine *_with_ocr.csv gefunden :",
              [os.path.basename(f) for f in files])

    probes, orients, y_vals, T_vals = [], [], [], []

    def getf(row, key):
        v = row.get(key)
        return float(v) if (v is not None and str(v).strip() != "") else None

    for fpath in files:
        with open(fpath, newline="") as f:
            r = csv.DictReader(f)
            for row in r:
                # Ziel (cps)
                y_str = (row.get("ocr_mean") or "").strip()
                if y_str == "":
                    continue
                try:
                    y_val = float(y_str)
                except ValueError:
                    continue

                x = getf(row, "x"); y_pos = getf(row, "y")
                z = getf(row, "z")

                ori = [0.0, 0.0, -1.0]

                # --- Messdauer T aus t_start/t_end --- heuristisch
                t_start = _parse_ts(row.get("t_start") or row.get("time_start"))
                t_end   = _parse_ts(row.get("t_end")   or row.get("time_end"))
                if t_start and t_end:
                    T = max(0.0, (t_end - t_start).total_seconds())

                probes.append([x, y_pos, z])
                orients.append(ori)
                y_vals.append(y_val)
                T_vals.append(T)

    if not probes:
        raise RuntimeError("Keine gültigen Messungen (x,y,z, ocr_mean) gefunden.")

    return (np.asarray(probes, float),
            np.asarray(orients, float),
            np.asarray(y_vals, float),
            np.asarray(T_vals, float))


# ---------------------------------
# CPS vs. Abstand zur Quelle
# ---------------------------------
def plot_cps_vs_distance(probes, y, sources):
    # Abstand jedes Messpunkts zur Quelle berechnen
    # (bei 1 Quelle: einfache Norm)
    delta = probes - sources[0]   # shape (N,3)
    distances = np.linalg.norm(delta, axis=1)

    plt.figure()
    plt.scatter(distances, y, s=18, c='tab:blue')
    plt.xlabel("Abstand zur Quelle [m]")   # falls deine Einheiten cm sind -> [cm]
    plt.ylabel("gemessene cps")
    plt.title("Gemessene cps vs. Abstand zur Quelle")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


# -------------------------------
# Fits: LS & Poisson
# -------------------------------
def fit_c1_ls_origin(g, y):
    """Least Squares durch Ursprung: C1 = (g^T y)/(g^T g)"""
    denom = float(np.dot(g, g))
    if denom <= 0:
        raise ValueError("sum(g^2) <= 0.")
    return float(np.dot(g, y) / denom)

def fit_c1_poisson_mle(g, y, T=None):
    """
    Poisson-MLE: k_n ~ Poisson(C1 * g_n * T_n), k_n = y_n * T_n
    => C1_hat = sum(k_n) / sum(g_n * T_n) = sum(y_n T_n) / sum(g_n T_n)
    """
    g = np.asarray(g, float)
    y = np.asarray(y, float)
    if T is None:
        T = np.ones_like(y)
    else:
        T = np.asarray(T, float)
    denom = float(np.sum(g * T))
    if denom <= 0:
        raise ValueError("sum(g*T) <= 0.")
    return float(np.sum(y * T) / denom)

# -------------------------------
# Plots
# -------------------------------
def plotting(y, y_hat_ls, y_hat_pois, resid_ls, resid_pois, T):
    # Parity-Plots
    plt.figure()
    plt.scatter(y, y_hat_ls, s=12, label="LS")
    plt.scatter(y, y_hat_pois, s=12, marker='x', label="Poisson")
    lo = float(min(y.min(), y_hat_ls.min(), y_hat_pois.min()))
    hi = float(max(y.max(), y_hat_ls.max(), y_hat_pois.max()))
    plt.plot([lo, hi], [lo, hi])
    plt.xlabel("gemessen (cps)")
    plt.ylabel("vorhergesagt (cps)")
    plt.title("Parity-Plot")
    plt.legend()
    plt.tight_layout()

    # Residuen vs. Vorhersage (LS)
    plt.figure()
    plt.scatter(y_hat_ls, resid_ls, s=12)
    plt.axhline(0.0)
    plt.xlabel("vorhergesagt (LS)")
    plt.ylabel("Residuum (y - ŷ)")
    plt.title("Residuen (LS)")
    plt.tight_layout()

    # Residuen vs. Vorhersage (Poisson)
    plt.figure()
    plt.scatter(y_hat_pois, resid_pois, s=12)
    plt.axhline(0.0)
    plt.xlabel("vorhergesagt (Poisson)")
    plt.ylabel("Residuum (y - ŷ)")
    plt.title("Residuen (Poisson)")
    plt.tight_layout()

    # Pearson-Residuen (Poisson-Approx. mit k=y*T, λ̂=ŷ*T)
    k = y * T
    lam_hat = y_hat_pois * T
    pearson = (k - lam_hat) / np.sqrt(np.maximum(lam_hat, 1e-12))

    plt.figure()
    plt.scatter(y_hat_pois, pearson, s=12)
    plt.axhline(0.0)
    plt.xlabel("vorhergesagt (Poisson)")
    plt.ylabel("Pearson-Residuum")
    plt.title("Pearson-Residuen vs. Vorhersage (Poisson)")
    plt.tight_layout()

    plt.figure()
    plt.hist(pearson, bins=30)
    plt.xlabel("Pearson-Residuum")
    plt.ylabel("Häufigkeit")
    plt.title("Histogramm der Pearson-Residuen")
    plt.tight_layout()

    plt.show()


### übersicht plotten der cps

# -------------------------------
# MAIN
# -------------------------------
if __name__ == "__main__":
    print(f"Heute:               {today.isoformat()}")
    print(f"Referenzdatum:       {ref_date.isoformat()}")
    print(f"Tage seit Referenz:  {t_elapsed_days} d")
    print(f"A0 (am t0):          {A0_Bq:.0f} Bq (25 µCi)")
    print(f"A(t=heute):          {activity_Bq_today:.2f} Bq")
    print(f"RESULTS_DIR:         {RESULTS_DIR}")
    print(f"OCR_INTERVAL_S:      {OCR_INTERVAL_S_DEFAULT} s (nur Fallback)")

    # Quellen (ANPASSEN!): z. B. eine Quelle im Ursprung
    sources = np.array([[0.35, 0.0, 0.0]], dtype=float)

    # Daten laden
    probes, orients, y, T = load_measurements(RESULTS_DIR)

    # g ohne C1/C2
    g = compute_g(sources, probes, orients, constants)

    # C1-Schätzungen
    C1_ls   = fit_c1_ls_origin(g, y)
    C1_pois = fit_c1_poisson_mle(g, y, T)

    # Vorhersagen & Residuen
    y_hat_ls   = C1_ls   * g
    y_hat_pois = C1_pois * g
    resid_ls   = y - y_hat_ls
    resid_pois = y - y_hat_pois

    rmse_ls = float(np.sqrt(np.mean(resid_ls**2)))
    rmse_pois = float(np.sqrt(np.mean(resid_pois**2)))

    print("\n=== Kalibrierungsergebnisse ===")
    print(f"C1 (Least Squares, Ursprung): {C1_ls:.6g}   | RMSE={rmse_ls:.6g}")
    print(f"C1 (Poisson-MLE)            : {C1_pois:.6g} | RMSE={rmse_pois:.6g}")

    # Plots
    plotting(y, y_hat_ls, y_hat_pois, resid_ls, resid_pois, T)
    plot_cps_vs_distance(probes, y, sources)