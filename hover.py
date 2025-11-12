#!/usr/bin/env python3
import time
from statistics import median
from collections import deque
from djitellopy import Tello

TARGET_CM = 20            # Sollabstand in cm
DEADBAND_CM = 1.5         # Toleranz: innerhalb kein Up/Down
K_P = 2.0                 # Proportionalfaktor: speed = K_P * error (cm)
MAX_UD_SPEED = 40         # Begrenze Up/Down-Geschwindigkeit [-100..100]
SAMPLE_HZ = 25            # Regel-Frequenz
LP_WINDOW = 5             # Rauschen unterdrücken

DEBUG_PRINT_EVERY = 0.20  # Sekunden
BATTERY_POLL_EVERY = 1.00 # Sekunden
CALIBRATION_TIME = 2.0    # Sekunden Messungen sammeln
CALIB_MIN_VALID = 10      # mind. gültige Messungen

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def read_tof_cm(t: Tello):
    try:
        v = t.get_distance_tof()
        if v is None or v <= 0:
            return None
        return int(v)
    except Exception:
        return None

def calibrate_tof(t: Tello, buf: deque):
    print("[CAL] Starte Kalibrierung des ToF-Sensors …")
    samples = []
    t0 = time.time()
    while time.time() - t0 < CALIBRATION_TIME:
        v = read_tof_cm(t)
        if v:
            samples.append(v)
        time.sleep(1.0 / SAMPLE_HZ)

    if len(samples) < CALIB_MIN_VALID:
        print(f"[CAL] Zu wenige gültige Messungen ({len(samples)}). Fahre ohne Kalibrierung fort.")
        return

    med = median(samples)
    filtered = [x for x in samples if abs(x - med) <= 15]
    if not filtered:
        filtered = samples

    buf.clear()
    for _ in range(buf.maxlen):
        buf.append(int(median(filtered)))

    print(f"[CAL] OK. Median-ToF={med:.1f} cm, n_valid={len(filtered)}. Filterpuffer vorgefüllt.")

def main():
    t = Tello()
    t.connect()
    battery = t.get_battery()
    print(f"[INFO] Battery: {battery}%")
    print(f"[INFO] SDK: {t.query_sdk_version()}")

    try:
        t.streamoff()
    except Exception:
        pass

    t.takeoff()
    time.sleep(0.8)

    t.send_rc_control(0, 0, 15, 0)
    time.sleep(0.5)
    t.send_rc_control(0, 0, 0, 0)

    buf = deque(maxlen=LP_WINDOW)

    calibrate_tof(t, buf)

    dt = 1.0 / SAMPLE_HZ
    print("[INFO] Hover-Regelung aktiv. Ctrl+C zum Beenden.")

    last_dbg = 0.0
    last_batt_poll = 0.0

    try:
        while True:
            now = time.time()
            if now - last_batt_poll >= BATTERY_POLL_EVERY:
                try:
                    battery = t.get_battery()
                except Exception:
                    pass
                last_batt_poll = now

            raw = read_tof_cm(t)
            if raw is None:
                est = buf[-1] if buf else TARGET_CM
            else:
                buf.append(raw)
                est = sum(buf) / len(buf)

            error = TARGET_CM - est 
            if abs(error) <= DEADBAND_CM:
                ud = 0
            else:
                ud = clamp(int(K_P * error), -MAX_UD_SPEED, MAX_UD_SPEED)

            t.send_rc_control(0, 0, ud, 0)

            if now - last_dbg >= DEBUG_PRINT_EVERY:
                print(f"[DBG] ToF_raw={raw if raw is not None else 'NA'} cm "
                      f"| ToF_est={est:.1f} cm | err={error:+.1f} cm | ud={ud} | batt={battery}%")
                last_dbg = now

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[INFO] Beende & lande…")
    finally:
        try:
            t.send_rc_control(0, 0, 0, 0); time.sleep(0.1)
        except Exception:
            pass
        try:
            t.land()
        except Exception:
            pass
        try:
            t.end()
        except Exception:
            pass
        print("[INFO] Done.")

if __name__ == "__main__":
    main()
