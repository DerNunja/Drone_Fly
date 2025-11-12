#!/usr/bin/env python3
import time, csv, math, os
from collections import deque
from djitellopy import Tello

SAMPLE_HZ = 20
DURATION_S = 8.0
ASCEND_TARGET_TOF = 60      # cm: über 40–50 cm ist der Flow meist stabiler
ASCEND_MAX_TIME = 4.0
LOG_PATH = "drift_log.csv"

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def get_tof_cm(t: Tello):
    v = t.get_distance_tof()
    return int(v) if v and v > 0 else None

def main():
    t = Tello()
    t.connect()
    batt = t.get_battery()
    sdk = t.query_sdk_version()
    print(f"[INFO] Connected. Battery={batt}%  SDK={sdk}")

    # Video aus (stabilere Telemetrie)
    try: t.streamoff()
    except: pass

    # Takeoff
    t.takeoff()
    time.sleep(0.5)

    # Sanft auf Zielhöhe (ToF) steigen
    print("[INFO] Steige auf ~60 cm für stabilen Optical-Flow …")
    t0 = time.time()
    while True:
        tof = get_tof_cm(t)
        if tof is not None and tof >= ASCEND_TARGET_TOF: break
        if time.time() - t0 > ASCEND_MAX_TIME: break
        t.send_rc_control(0, 0, 25, 0)
        time.sleep(0.08)
    t.send_rc_control(0, 0, 0, 0)
    time.sleep(0.6)

    # Logging
    print(f"[INFO] Starte Logging {DURATION_S:.1f}s @ {SAMPLE_HZ} Hz → {LOG_PATH}")
    dt = 1.0 / SAMPLE_HZ
    rows = []
    last_yaw = None
    yaw_rate_buf = deque(maxlen=5)  # geglättete Yaw-Rate

    start = time.time()
    try:
        while time.time() - start < DURATION_S:
            ts = time.time() - start

            # Geschwindigkeit (cm/s) – vom Tello aus Optical Flow/IMU abgeleitet
            try:
                vx = t.get_speed_x()  # + vorwärts (zur Frontkamera hin)
                vy = t.get_speed_y()  # + nach rechts (vermutete Konvention)
                vz = t.get_speed_z()  # + aufwärts
            except Exception:
                vx = vy = vz = None

            # Lagewinkel
            try:
                yaw = t.get_yaw()     # Grad
                pitch = t.get_pitch() # Grad (vor/zurück)
                roll = t.get_roll()   # Grad (links/rechts)
            except Exception:
                yaw = pitch = roll = None

            # Yaw-Driftrate (deg/s) aus Differenzen
            yaw_rate = None
            if yaw is not None:
                if last_yaw is not None:
                    dy = yaw - last_yaw
                    # Winkelwrap [-180, 180]
                    if dy > 180: dy -= 360
                    if dy < -180: dy += 360
                    yaw_rate = dy / dt
                    yaw_rate_buf.append(yaw_rate)
                last_yaw = yaw

            tof = get_tof_cm(t)
            batt = t.get_battery()

            rows.append([ts, tof, vx, vy, vz, roll, pitch, yaw,
                         (sum(yaw_rate_buf)/len(yaw_rate_buf)) if yaw_rate_buf else None,
                         batt])

            # Sticks neutral lassen, nur Keep-Alive
            t.send_rc_control(0, 0, 0, 0)
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[INFO] Abgebrochen – lande …")
    finally:
        try:
            t.send_rc_control(0, 0, 0, 0); time.sleep(0.1)
        except: pass
        try:
            t.land()
        except: pass
        try:
            t.end()
        except: pass

    # CSV speichern
    header = ["t", "tof_cm", "vx_cms", "vy_cms", "vz_cms",
              "roll_deg", "pitch_deg", "yaw_deg", "yaw_rate_dps", "battery_pct"]
    with open(LOG_PATH, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    # Auswertung
    # Mittelwerte & Drifts
    def mean(xs):
        xs = [x for x in xs if x is not None]
        return sum(xs)/len(xs) if xs else None

    vx_mean = mean([r[2] for r in rows])
    vy_mean = mean([r[3] for r in rows])
    vz_mean = mean([r[4] for r in rows])
    yaw_rate_mean = mean([r[8] for r in rows])
    tof_mean = mean([r[1] for r in rows])

    print("\n========== DRIFT-ANALYSE ==========")
    print(f"Durchschnitt ToF:       {tof_mean:.1f} cm" if tof_mean is not None else "ToF: n/a")
    print(f"Ø vx (vor/zurück):      {vx_mean:+.2f} cm/s" if vx_mean is not None else "vx: n/a")
    print(f"Ø vy (rechts/links):    {vy_mean:+.2f} cm/s" if vy_mean is not None else "vy: n/a")
    print(f"Ø vz (hoch/runter):     {vz_mean:+.2f} cm/s" if vz_mean is not None else "vz: n/a")
    print(f"Ø Yaw-Drift:            {yaw_rate_mean:+.2f} °/s" if yaw_rate_mean is not None else "yaw_rate: n/a")
    print(f"Log gespeichert unter:  {os.path.abspath(LOG_PATH)}")

    # Heuristische Interpretation seitlicher Drift
    # Konvention (meistens): vy > 0 ⇒ Drift nach rechts, vy < 0 ⇒ Drift nach links.
    if vy_mean is not None:
        if abs(vy_mean) < 0.8:
            print("Seitendrift: gering.")
        else:
            if vy_mean < 0:
                print("Seitendrift: nach LINKS (vy < 0).")
                print("→ Wahrscheinlich WENIGER Schub links ODER MEHR Schub rechts.")
                print("   Prüfe/tausche linke Props, reinige linken Motor; IMU kalibrieren.")
            else:
                print("Seitendrift: nach RECHTS (vy > 0).")
                print("→ Wahrscheinlich WENIGER Schub rechts ODER MEHR Schub links.")
                print("   Prüfe/tausche rechte Props, reinige rechten Motor; IMU kalibrieren.")
    # Yaw-Heuristik
    if yaw_rate_mean is not None and abs(yaw_rate_mean) > 1.0:
        d = "im Uhrzeigersinn" if yaw_rate_mean > 0 else "gegen Uhrzeigersinn"
        print(f"Konstanter Yaw-Drift ({d}, {abs(yaw_rate_mean):.1f} °/s) → Prop-Unwucht oder Gyro-Offset.")
        print("→ Props neu stecken/tauschen; IMU-Kalibrierung in der Tello-App durchführen.")

if __name__ == "__main__":
    main()
