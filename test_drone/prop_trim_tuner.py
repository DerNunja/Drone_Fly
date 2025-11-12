#!/usr/bin/env python3
import time, csv, os
from statistics import fmean
from djitellopy import Tello

# ===== Parameter =====
SAMPLE_HZ       = 20
ASCEND_TOF_CM   = 60     # Zielhöhe (ToF) für stabilen Optical-Flow
ASCEND_MAX_S    = 4.0
STABILIZE_S     = 1.0    # Warte nach Manövern
PHASE_LEN_S     = 3.0    # Messdauer je Phase
TRIM_TEST       = 12     # Test-LR-Trim (Stick-Wert, [-100..100])
MAX_RECO_TRIM   = 20     # empfohlene Trimmung begrenzen
LOG_CSV         = "prop_trim_log.csv"

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def get_tof_cm(t: Tello):
    try:
        v = t.get_distance_tof()
        return int(v) if v and v > 0 else None
    except Exception:
        return None

def mean(xs):
    xs = [x for x in xs if x is not None]
    return fmean(xs) if xs else None

def collect(t: Tello, dur_s: float, lr: int = 0, fb: int = 0, ud: int = 0, yaw: int = 0):
    """
    Hält die Sticks konstant (lr/fb/ud/yaw) und loggt Telemetrie dur_s Sekunden.
    Gibt Liste von Messzeilen zurück.
    """
    rows = []
    dt = 1.0 / SAMPLE_HZ
    start = time.time()
    while time.time() - start < dur_s:
        ts = time.time() - start
        try:
            vx = t.get_speed_x()    # + vorwärts
            vy = t.get_speed_y()    # + rechts
            vz = t.get_speed_z()    # + aufwärts
        except Exception:
            vx = vy = vz = None
        try:
            roll  = t.get_roll()
            pitch = t.get_pitch()
            yaw_g = t.get_yaw()
        except Exception:
            roll = pitch = yaw_g = None
        tof = get_tof_cm(t)
        batt = None
        try:
            # Batterie nicht zu oft pollen
            if int(ts * 2) % 2 == 0:
                batt = t.get_battery()
        except Exception:
            pass

        rows.append([ts, lr, fb, ud, yaw, tof, vx, vy, vz, roll, pitch, yaw_g, batt])

        # Wichtig: kontinuierlich RC senden
        t.send_rc_control(lr, fb, ud, yaw)
        time.sleep(dt)
    # Sticks neutralisieren
    t.send_rc_control(0, 0, 0, 0)
    return rows

def main():
    t = Tello()
    t.connect()
    batt = t.get_battery()
    sdk  = t.query_sdk_version()
    print(f"[INFO] Connected. Battery={batt}%  SDK={sdk}")

    # Video aus
    try: t.streamoff()
    except: pass

    # Takeoff
    t.takeoff()
    time.sleep(0.5)

    # sanft auf Zielhöhe
    print("[INFO] Steige auf ~60 cm …")
    t0 = time.time()
    while True:
        tof = get_tof_cm(t)
        if tof is not None and tof >= ASCEND_TOF_CM: break
        if time.time() - t0 > ASCEND_MAX_S: break
        t.send_rc_control(0, 0, 25, 0)
        time.sleep(0.08)
    t.send_rc_control(0, 0, 0, 0)
    time.sleep(STABILIZE_S)

    # ===== Phasen =====
    print("[INFO] Phase A: Baseline (neutral) …")
    base_rows = collect(t, PHASE_LEN_S, lr=0, fb=0, ud=0, yaw=0)
    time.sleep(STABILIZE_S)

    print(f"[INFO] Phase B: Rechts-Trim (lr=+{TRIM_TEST}) …")
    right_rows = collect(t, PHASE_LEN_S, lr=+TRIM_TEST, fb=0, ud=0, yaw=0)
    time.sleep(STABILIZE_S)

    print(f"[INFO] Phase C: Links-Trim (lr=-{TRIM_TEST}) …")
    left_rows  = collect(t, PHASE_LEN_S, lr=-TRIM_TEST, fb=0, ud=0, yaw=0)
    time.sleep(0.5)

    # Landen
    print("[INFO] Landen …")
    try:
        t.land()
    finally:
        try: t.end()
        except: pass

    # ===== Auswertung =====
    # Mittelwerte vy & roll je Phase
    def phase_stats(rows):
        vy_m = mean([r[7] for r in rows])      # vy in cm/s (rechts/links)
        roll_m = mean([r[9] for r in rows])    # roll in Grad
        tof_m = mean([r[5] for r in rows])
        return vy_m, roll_m, tof_m

    vy_base, roll_base, tof_base   = phase_stats(base_rows)
    vy_right, roll_right, _        = phase_stats(right_rows)
    vy_left,  roll_left,  _        = phase_stats(left_rows)

    # Sensitivität: wie stark ändert sich vy pro LR-Stick-Einheit?
    # delta_vy ≈ vy_right - vy_left bei 2*TRIM_TEST Differenz
    sensitivity = None
    if vy_right is not None and vy_left is not None:
        delta_vy = vy_right - vy_left
        sensitivity = delta_vy / (2.0 * TRIM_TEST) if abs(delta_vy) > 1e-3 else None

    # Empfohlene LR_TRIM: neutralisiert vy_base
    recom_lr = None
    if sensitivity is not None and vy_base is not None:
        # vy ≈ vy_base + sensitivity * lr  -> lr* = -vy_base / sensitivity
        recom_lr = int(round(clamp(-vy_base / sensitivity, -MAX_RECO_TRIM, MAX_RECO_TRIM)))

    # ===== Log speichern =====
    header = ["t","lr","fb","ud","yaw","tof_cm","vx_cms","vy_cms","vz_cms","roll_deg","pitch_deg","yaw_deg","battery_pct"]
    with open(LOG_CSV, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["# Baseline"])
        w.writerow(header); w.writerows(base_rows)
        w.writerow(["# Right +TRIM_TEST"]); w.writerow(header); w.writerows(right_rows)
        w.writerow(["# Left -TRIM_TEST"]);  w.writerow(header); w.writerows(left_rows)
    print(f"[INFO] Log gespeichert: {os.path.abspath(LOG_CSV)}")

    # ===== Bericht =====
    print("\n========= PROP-TRIM ANALYSE =========")
    def pf(v): return "n/a" if v is None else f"{v:+.2f}"
    print(f"ToF Mittel (Baseline): {tof_base:.1f} cm" if tof_base else "ToF: n/a")
    print(f"Ø vy  Baseline: {pf(vy_base)} cm/s   (negativ ⇒ Drift LINKS, positiv ⇒ Drift RECHTS)")
    print(f"Ø vy  Right(+{TRIM_TEST}): {pf(vy_right)} cm/s")
    print(f"Ø vy  Left(-{TRIM_TEST}):  {pf(vy_left)} cm/s")
    print(f"Ø roll Baseline: {pf(roll_base)} °")
    print(f"Modell-Sensitivität vy/Stick: {pf(sensitivity)} (cm/s pro LR-Einheit)" if sensitivity is not None else "Sensitivität: n/a")

    if recom_lr is not None:
        direction = "nach rechts" if recom_lr > 0 else "nach links" if recom_lr < 0 else "neutral"
        print(f"\n👉 EMPFOHLENE Trim-Korrektur: LR_TRIM = {recom_lr}  ({direction})")
        print("   → Beispiel in deinem Hover:  t.send_rc_control(LR_TRIM, 0, ud, 0)")
    else:
        # fallback Heuristik
        if vy_base is None:
            print("\n⚠️ Keine verlässliche Empfehlung (keine vy-Daten).")
        elif abs(vy_base) < 0.6:
            print("\n✅ Drift sehr gering – keine Trimmung nötig.")
        else:
            heur = clamp(int(round(-vy_base / 1.5)), -MAX_RECO_TRIM//2, MAX_RECO_TRIM//2)
            print(f"\n🛈 Heuristische Empfehlung: LR_TRIM ≈ {heur}  (aus vy_baseline)")
            print("   (Sensitivität konnte nicht robust geschätzt werden)")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Abbruch per Tastatur.")
