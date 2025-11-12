import time
from collections import deque
import cv2
import pygame
from djitellopy import Tello

from config import *
from ui.hud import draw_hotkey_help
from control.mapping import compute_controls
from utils.tello_utils import (
    robust_tof_cm, calibrate_tof, settle_to_altitude, hover_ud_from_alt,
    switch_camera_with_reset
)

# Backend wählen
if BACKEND.lower() == "mediapipe":
    from detectors.mediapipe_detector import HandDetector
    BACKEND_NAME = "MediaPipe"
else:
    from detectors.yolo_detector import HandDetector
    BACKEND_NAME = "YOLO"

def main():
    global SHOW_HELP_DEFAULT
    pygame.init()
    pygame.display.set_caption("Controls (Fokus hier für Hotkeys: D/H/ESC)")
    screen = pygame.display.set_mode((420, 60))
    clock = pygame.time.Clock()

    t = Tello()
    t.connect()
    battery = t.get_battery()
    print(f"[INFO] Battery: {battery}%  SDK: {t.query_sdk_version()}")

    # Stream & initial Kamera (Front)
    try: t.streamoff()
    except: pass
    t.streamon(); time.sleep(0.4)
    try:
        if hasattr(Tello, "CAMERA_FORWARD"):
            t.set_video_direction(Tello.CAMERA_FORWARD)
        else:
            t.send_control_command("downvision 0")
    except Exception:
        pass
    print("[INFO] Kamera: Front")
    frame_read = t.get_frame_read(); time.sleep(0.2)

    # Start
    t.takeoff()
    time.sleep(0.6)
    t.send_rc_control(0, 0, 20, 0); time.sleep(0.4)
    t.send_rc_control(0, 0, 0, 0)

    # Höhe filtern & kalibrieren
    buf = deque(maxlen=LP_WINDOW)
    calibrate_tof(t, buf)
    print(f"[INFO] Auf {HOVER_TARGET_CM} cm einregeln …")
    settle_to_altitude(t, HOVER_TARGET_CM, timeout_s=6.0)

    detector = HandDetector(
        **({"weights": YOLO_WEIGHTS, "imgsz": YOLO_IMGSZ, "conf": YOLO_CONF} if BACKEND_NAME=="YOLO" else {})
    )

    cx_s = cy_s = None
    hand_seen_once = False
    last_hand_seen_ts = None
    last_dbg = last_batt = time.time()
    using_down = False
    running = True
    SHOW_HELP = SHOW_HELP_DEFAULT

    print("[INFO] Follow aktiv. (D) Kamera wechseln, (H) Hilfe, (ESC) landen")
    d_down = h_down = False

    try:
        while running:
            # --- Input (Pygame) ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("[INFO] ESC → Landen."); running = False
                    elif event.key == pygame.K_d and not d_down:
                        d_down = True
                        using_down, frame_read = switch_camera_with_reset(t, to_down=not using_down)
                        print("[INFO] Kamera:", "Down (dy->FB)" if using_down else "Front (dy->UD)")
                    elif event.key == pygame.K_h and not h_down:
                        h_down = True
                        SHOW_HELP = not SHOW_HELP
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_d: d_down = False
                    if event.key == pygame.K_h: h_down = False

            # --- Sensorik / Höhe ---
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
                clock.tick(SAMPLE_HZ); 
                continue

            h, w = frame.shape[:2]
            cx_img, cy_img = w//2, h//2

            center = detector.find_hand_center(frame)
            lr = fb = ud = 0
            status = "WAITING HAND (Down)" if using_down else "WAITING HAND (Front)"

            if center is not None:
                # EMA
                if cx_s is None:
                    cx_s, cy_s = center
                else:
                    cx_s = int(EMA_ALPHA*center[0] + (1-EMA_ALPHA)*cx_s)
                    cy_s = int(EMA_ALPHA*center[1] + (1-EMA_ALPHA)*cy_s)

                dx = cx_s - cx_img
                dy = cy_s - cy_img

                lr, fb, ud_map = compute_controls(dx, dy, using_down, est)
                ud = hover_ud_from_alt(est) if using_down else ud_map

                hand_seen_once = True
                last_hand_seen_ts = now
                status = "FOLLOWING (Down)" if using_down else "FOLLOWING (Front)"

                # Visualisierung
                cv2.circle(frame, (cx_s, cy_s), 8, (0,255,0), 2)
                cv2.circle(frame, (cx_img, cy_img), 6, (255,255,255), 1)
                cv2.line(frame, (cx_s, cy_s), (cx_img, cy_img), (0,255,255), 1)
            else:
                # keine Hand
                ud = hover_ud_from_alt(est) if using_down else 0
                lr = fb = 0
                if hand_seen_once:
                    missing = now - last_hand_seen_ts if last_hand_seen_ts else 0
                    if missing >= HAND_LOST_LAND_S:
                        print("[FAILSAFE] Hand seit > %.1fs weg → Landen." % HAND_LOST_LAND_S)
                        running = False
                    elif missing >= HAND_LOST_HOVER_S:
                        status = f"HAND LOST ({missing:.1f}s) — hover"
                    else:
                        status = f"HAND LOST ({missing:.1f}s) — grace"

            # --- RC senden (immer Reihenfolge lr, fb, ud, yaw!) ---
            t.send_rc_control(lr, fb, ud, 0)

            # --- Debug ---
            if now - last_dbg >= DEBUG_EVERY:
                print(f"[DBG] alt={raw if raw else 'NA'} est={est:.1f}cm | lr={lr} fb={fb} ud={ud} | batt={battery}% | {status}")
                last_dbg = now

            # --- Overlay ---
            if SHOW_HELP:
                draw_hotkey_help(frame, using_down, BACKEND_NAME)
            else:
                cv2.putText(frame, f"{'DOWN (dy->FB)' if using_down else 'FRONT (dy->UD)'}  |  batt {battery}%  |  h_est {est:.0f}cm",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

            cv2.imshow("Tello Hand-Follow (OpenCV Video + Pygame Keys)", frame)
            cv2.waitKey(1)
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