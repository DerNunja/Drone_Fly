import time
from statistics import median
from collections import deque

from config import SAMPLE_HZ, HOVER_KP, HOVER_MAX_UD, HOVER_TARGET_CM, HOVER_DEADBAND_CM, LP_WINDOW

def clamp(v, lo, hi): 
    return lo if v < lo else hi if v > hi else v

def robust_tof_cm(tello):
    try:
        v = tello.get_distance_tof()
        return int(v) if v and v > 0 else None
    except Exception:
        return None

def calibrate_tof(tello, buf: deque, seconds=2.5, min_valid=12):
    print("[CAL] Kalibriere ToF …")
    samples, t0, dt = [], time.time(), 1.0 / SAMPLE_HZ
    while time.time() - t0 < seconds:
        v = robust_tof_cm(tello)
        if v:
            samples.append(v)
        time.sleep(dt)
    if len(samples) < min_valid:
        print(f"[CAL] Zu wenige Messungen ({len(samples)}). Weiter ohne Vorfüllung.")
        return
    med = median(samples)
    filtered = [x for x in samples if abs(x - med) <= 15] or samples
    buf.clear()
    for _ in range(buf.maxlen):
        buf.append(int(median(filtered)))
    print(f"[CAL] OK. ToF-Median={med:.1f} cm, n={len(filtered)}")

def settle_to_altitude(tello, target_cm=HOVER_TARGET_CM, timeout_s=6.0):
    """Kurzes Einregeln auf Zielhöhe mit Burst-UD-Kommandos (vor Start des Reglers)."""
    t0 = time.time()
    while True:
        tof = robust_tof_cm(tello)
        if tof and abs(tof - target_cm) <= 5:
            break
        if time.time() - t0 > timeout_s:
            break
        ud_burst = 25 if (tof is None or tof < target_cm) else -25
        tello.send_rc_control(0, 0, ud_burst, 0)
        time.sleep(0.08)
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(0.3)

def hover_ud_from_alt(est_alt_cm):
    """UD vom Hover-Regler (P) – nutzt HOVER_TARGET_CM / HOVER_KP etc."""
    err_h = HOVER_TARGET_CM - est_alt_cm
    if abs(err_h) <= HOVER_DEADBAND_CM:
        return 0
    return clamp(int(HOVER_KP * err_h), -HOVER_MAX_UD, HOVER_MAX_UD)

def switch_camera_with_reset(tello, to_down: bool):
    """Erzwingt Kamerawechsel inkl. Stream-Reset. Gibt (using_down, frame_read) zurück."""
    try:
        if to_down:
            if hasattr(tello.__class__, "CAMERA_DOWNWARD"):
                tello.set_video_direction(tello.CAMERA_DOWNWARD)
            elif hasattr(tello.__class__, "CAMERA_BOTTOM"):
                tello.set_video_direction(tello.CAMERA_BOTTOM)
            else:
                tello.send_control_command("downvision 1")
        else:
            if hasattr(tello.__class__, "CAMERA_FORWARD"):
                tello.set_video_direction(tello.CAMERA_FORWARD)
            else:
                tello.send_control_command("downvision 0")
        try: tello.streamoff()
        except: pass
        time.sleep(0.2)
        tello.streamon()
        time.sleep(0.5)
        fr = tello.get_frame_read()
        time.sleep(0.2)
        return to_down, fr
    except Exception as e:
        print("[WARN] Kamera-Umschaltung fehlgeschlagen:", e)
        return (False, tello.get_frame_read())
