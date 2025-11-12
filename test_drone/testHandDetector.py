import time
from statistics import median
from collections import deque
import cv2
import pygame
from djitellopy import Tello

BACKEND = "mediapipe"             # "mediapipe" | "yolo"
YOLO_WEIGHTS = "hand_detector.pt"
YOLO_IMGSZ = 320
YOLO_CONF  = 0.35

HOVER_TARGET_CM   = 40
HOVER_DEADBAND_CM = 2.0
HOVER_KP          = 2.0
HOVER_MAX_UD      = 40
ALT_MIN_CM        = 20
ALT_MAX_CM        = 120
LP_WINDOW         = 7
SAMPLE_HZ         = 25

FOLLOW_DEADBAND_PX = 50
FOLLOW_KP_LR       = 0.20
FOLLOW_KP_FB       = 0.25      # nur Down-Cam
FOLLOW_KP_UD       = 0.35      # nur Front-Cam
FOLLOW_MAX_LR      = 25
FOLLOW_MAX_FB      = 25
FOLLOW_MAX_UD      = 35
EMA_ALPHA          = 0.5

STARTUP_GRACE_S   = 8.0
HAND_LOST_HOVER_S = 5.0
HAND_LOST_LAND_S  = 12.0

DEBUG_EVERY = 0.25
BATTERY_POLL_EVERY = 1.0
SHOW_HELP = True  # Hotkey-Overlay

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def robust_tof_cm(t):
    try:
        v = t.get_distance_tof()
        return int(v) if v and v > 0 else None
    except Exception:
        return None

def calibrate_tof(t, buf, seconds=2.5, min_valid=12):
    print("[CAL] Kalibriere ToF …")
    samples, t0, dt = [], time.time(), 1.0 / SAMPLE_HZ
    while time.time() - t0 < seconds:
        v = robust_tof_cm(t)
        if v: samples.append(v)
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


if BACKEND == "mediapipe":
    import mediapipe as mp
    class HandDetector:
        def __init__(self, max_num_hands=1, det_conf=0.6, track_conf=0.6):
            self.hands = mp.solutions.hands.Hands(
                static_image_mode=False, max_num_hands=max_num_hands,
                model_complexity=0, min_detection_confidence=det_conf,
                min_tracking_confidence=track_conf
            )
        def find_hand_center(self, frame_bgr):
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            res = self.hands.process(rgb)
            if not res.multi_hand_landmarks: return None
            h, w = frame_bgr.shape[:2]
            lm = res.multi_hand_landmarks[0]
            idxs = [0,5,9,13,17]
            cx = sum(lm.landmark[i].x for i in idxs)/len(idxs)*w
            cy = sum(lm.landmark[i].y for i in idxs)/len(idxs)*h
            return int(cx), int(cy)
else:
    from ultralytics import YOLO
    import torch
    class HandDetector:
        def __init__(self, weights=YOLO_WEIGHTS, imgsz=YOLO_IMGSZ, conf=YOLO_CONF):
            self.model = YOLO(weights)
            self.imgsz = imgsz; self.conf = conf
            self.device = "0" if torch.cuda.is_available() else "cpu"
        def find_hand_center(self, frame_bgr):
            h, w = frame_bgr.shape[:2]
            size = self.imgsz
            im = cv2.resize(frame_bgr, (size, size))
            r = self.model.predict(source=im, imgsz=size, conf=self.conf, verbose=False, device=self.device)[0]
            if len(r.boxes) == 0: return None
            xyxy = r.boxes.xyxy
            areas = (xyxy[:,2]-xyxy[:,0])*(xyxy[:,3]-xyxy[:,1])
            i = int(areas.argmax().item())
            x1,y1,x2,y2 = xyxy[i].tolist()
            sx, sy = w/size, h/size
            cx = (x1+x2)/2 * sx
            cy = (y1+y2)/2 * sy
            return int(cx), int(cy)


def compute_controls(dx, dy, using_down, est_alt_cm):
    """OpenCV: y nach unten positiv."""
    lr = fb = ud = 0
    if abs(dx) > FOLLOW_DEADBAND_PX:
        lr = clamp(int(FOLLOW_KP_LR * dx), -FOLLOW_MAX_LR, FOLLOW_MAX_LR)

    if using_down:
        if abs(dy) > FOLLOW_DEADBAND_PX:
            fb = clamp(int(FOLLOW_KP_FB * dy), -FOLLOW_MAX_FB, FOLLOW_MAX_FB)  
        ud = None  
    else:
        if abs(dy) > FOLLOW_DEADBAND_PX:
            ud = clamp(int(FOLLOW_KP_UD * dy), -FOLLOW_MAX_UD, FOLLOW_MAX_UD)  
        else:
            ud = 0
        if est_alt_cm is not None:
            if est_alt_cm < ALT_MIN_CM:  ud = max(ud, +15)
            if est_alt_cm > ALT_MAX_CM:  ud = min(ud, -15)
    return lr, fb, ud

def draw_hotkey_help(img, using_down, backend_name):
    lines = [
        f"Mode: {'DOWN (dy->FB)' if using_down else 'FRONT (dy->UD)'} | Backend: {backend_name}",
        "Hotkeys:",
        "  D  : Kamera/Mappings wechseln",
        "  H  : Hilfe ein/aus",
        "  ESC: Landen & beenden",
    ]
    x, y = 10, 24
    for i, txt in enumerate(lines):
        cv2.putText(img, txt, (x, y + i*22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0,255,255) if i==0 else (255,255,255), 2)

def switch_camera(t: Tello, to_down: bool):
    """erzwingt Cam-Switch + Stream-Reset, liefert (using_down, new_frame_read)."""
    try:
        if to_down:
            if hasattr(Tello, "CAMERA_DOWNWARD"):
                t.set_video_direction(Tello.CAMERA_DOWNWARD)
            elif hasattr(Tello, "CAMERA_BOTTOM"):
                t.set_video_direction(Tello.CAMERA_BOTTOM)
            else:
                t.send_control_command("downvision 1")
        else:
            if hasattr(Tello, "CAMERA_FORWARD"):
                t.set_video_direction(Tello.CAMERA_FORWARD)
            else:
                t.send_control_command("downvision 0")
        # Stream neu starten + BackgroundFrameRead erneuern
        try: t.streamoff()
        except: pass
        time.sleep(0.2)
        t.streamon()
        time.sleep(0.5)
        fr = t.get_frame_read()
        time.sleep(0.2)
        return to_down, fr
    except Exception as e:
        print("[WARN] Kamera-Umschaltung fehlgeschlagen:", e)
        return (False, t.get_frame_read())

def main():
    global SHOW_HELP
    # ==== Pygame nur für Input ====
    pygame.init()
    pygame.display.set_caption("Controls (Fokus hier für Hotkeys: D/H/ESC)")
    screen = pygame.display.set_mode((380, 60))  # kleines Fenster, reicht für Fokus
    clock = pygame.time.Clock()

    t = Tello()
    t.connect()
    battery = t.get_battery()
    print(f"[INFO] Battery: {battery}%  SDK: {t.query_sdk_version()}")

    # Stream + initial Kamera: Front
    try: t.streamoff()
    except: pass
    t.streamon(); time.sleep(0.4)

    using_down = False
    try:
        if hasattr(Tello, "CAMERA_FORWARD"):
            t.set_video_direction(Tello.CAMERA_FORWARD)
        else:
            t.send_control_command("downvision 0")
    except Exception:
        pass
    print("[INFO] Kamera: Front")

    frame_read = t.get_frame_read(); time.sleep(0.2)

    # Takeoff
    t.takeoff()
    time.sleep(0.6)
    t.send_rc_control(0, 0, 20, 0); time.sleep(0.4)
    t.send_rc_control(0, 0, 0, 0)

    # Höhe glätten + kalibrieren
    buf = deque(maxlen=LP_WINDOW)
    calibrate_tof(t, buf)

    # auf ~40 cm einregeln
    print("[INFO] Steige/Senke auf 40 cm …")
    t0 = time.time()
    while True:
        tof = robust_tof_cm(t)
        if tof and abs(tof - HOVER_TARGET_CM) <= 5: break
        if time.time() - t0 > 6.0: break
        ud_burst = 25 if (tof is None or tof < HOVER_TARGET_CM) else -25
        t.send_rc_control(0, 0, ud_burst, 0); time.sleep(0.08)
    t.send_rc_control(0, 0, 0, 0); time.sleep(0.3)

    detector = HandDetector()
    cx_s = cy_s = None
    hand_seen_once = False
    last_hand_seen_ts = None
    last_dbg = last_batt = time.time()
    backend_name = "MediaPipe" if BACKEND == "mediapipe" else "YOLO"

    print("[INFO] Follow aktiv. (D) Kamera wechseln, (H) Hilfe, (ESC) landen")
    running = True
    d_down = h_down = False  # Edge-Trigger für Tasten
    try:
        while running:
            # ==== Pygame-Events ====
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("[INFO] ESC → Landen."); running = False
                    elif event.key == pygame.K_d and not d_down:
                        d_down = True
                        using_down, frame_read = switch_camera(t, to_down=not using_down)
                        print("[INFO] Kamera:", "Down (dy->FB)" if using_down else "Front (dy->UD)")
                    elif event.key == pygame.K_h and not h_down:
                        h_down = True
                        SHOW_HELP = not SHOW_HELP
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_d: d_down = False
                    if event.key == pygame.K_h: h_down = False

            # ==== Sensorik / Regler ====
            now = time.time()
            if now - last_batt >= BATTERY_POLL_EVERY:
                try: battery = t.get_battery()
                except: pass
                last_batt = now

            raw = robust_tof_cm(t)
            if raw is not None:
                buf.append(raw)
            est = (sum(buf)/len(buf)) if buf else HOVER_TARGET_CM

            frame = frame_read.frame
            if frame is None:
                t.send_rc_control(0, 0, 0, 0)
                clock.tick(SAMPLE_HZ); continue

            h, w = frame.shape[:2]
            cx_img, cy_img = w//2, h//2

            center = detector.find_hand_center(frame)
            lr = fb = ud = 0
            status = "WAITING HAND (Front)" if not using_down else "WAITING HAND (Down)"

            if center is not None:
                # EMA
                if cx_s is None:
                    cx_s, cy_s = center
                else:
                    cx_s = int(EMA_ALPHA*center[0] + (1-EMA_ALPHA)*cx_s)
                    cy_s = int(EMA_ALPHA*center[1] + (1-EMA_ALPHA)*cy_s)

                dx = cx_s - cx_img
                dy = cy_s - cy_img

                lr, fb, ud_from_map = compute_controls(dx, dy, using_down, est)

                if using_down:
                    err_h = HOVER_TARGET_CM - est
                    ud_hover = 0 if abs(err_h) <= HOVER_DEADBAND_CM else clamp(int(HOVER_KP*err_h), -HOVER_MAX_UD, HOVER_MAX_UD)
                    ud = ud_hover
                else:
                    ud = ud_from_map

                hand_seen_once = True
                last_hand_seen_ts = now
                status = "FOLLOWING (Down)" if using_down else "FOLLOWING (Front)"

                # Overlay
                cv2.circle(frame, (cx_s, cy_s), 8, (0,255,0), 2)
                cv2.circle(frame, (cx_img, cy_img), 6, (255,255,255), 1)
                cv2.line(frame, (cx_s, cy_s), (cx_img, cy_img), (0,255,255), 1)
            else:
                # Keine Hand
                if using_down:
                    err_h = HOVER_TARGET_CM - est
                    ud = 0 if abs(err_h) <= HOVER_DEADBAND_CM else clamp(int(HOVER_KP*err_h), -HOVER_MAX_UD, HOVER_MAX_UD)
                else:
                    ud = 0
                lr = fb = 0

                if hand_seen_once:
                    missing = (now - last_hand_seen_ts) if last_hand_seen_ts else 0
                    if missing >= HAND_LOST_LAND_S:
                        print("[FAILSAFE] Hand seit > %.1fs weg → Landen." % HAND_LOST_LAND_S)
                        running = False
                    elif missing >= HAND_LOST_HOVER_S:
                        status = f"HAND LOST ({missing:.1f}s) — hover"
                    else:
                        status = f"HAND LOST ({missing:.1f}s) — grace"

            # Senden wenn downkamera
            #t.send_rc_control(lr, fb, ud, 0)

            # Senden wenn frontkamera
            t.send_rc_control(lr, ud, fb, 0)

            # Debug
            if now - last_dbg >= DEBUG_EVERY:
                print(f"[DBG] alt={raw if raw else 'NA'} est={est:.1f}cm | "
                      f"lr={lr} fb={fb} ud={ud} | batt={battery}% | {status}")
                last_dbg = now

            # Overlay / Hilfe
            if SHOW_HELP:
                draw_hotkey_help(frame, using_down, backend_name)
            else:
                cv2.putText(frame, f"{'DOWN (dy->FB)' if using_down else 'FRONT (dy->UD)'}  |  batt {battery}%  |  h_est {est:.0f}cm",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

            cv2.imshow("Tello Hand-Follow (OpenCV Video + Pygame Keys)", frame)
            cv2.waitKey(1)  # nur für GUI-Refresh (keine Key-Abfrage hier)

            clock.tick(SAMPLE_HZ)

    except KeyboardInterrupt:
        print("\n[INFO] Abbruch per KeyboardInterrupt.")
    finally:
        cv2.destroyAllWindows()
        try: t.send_rc_control(0,0,0,0); time.sleep(0.1)
        except: pass
        try: t.land()
        except: pass
        try: t.end()
        except: pass
        pygame.quit()
        print("[INFO] Landed & beendet.")

if __name__ == "__main__":
    main()