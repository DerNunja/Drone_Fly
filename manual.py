from djitellopy import Tello
import cv2
import pygame
import time

S = 60         # Grundgeschwindigkeit
FPS = 60       # Loop-Frequenz (Steuer-/Video-Update)


def open_tello_stream_with_opencv():
    url = "udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000"
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        raise RuntimeError("Konnte den Tello-UDP-Stream via OpenCV nicht öffnen")
    return cap

class Frontend:
    def __init__(self, tello: Tello):
        self.tello = tello
        self.cap = None
        self.frame_read = None
        # RC-State
        self.lr = 0   # left/right
        self.fb = 0   # forward/back
        self.ud = 0   # up/down
        self.yw = 0   # yaw
        self.flying = False

    def run(self):
        # --- Pygame vorbereiten ---
        pygame.init()
        screen = pygame.display.set_mode((320, 240))
        pygame.display.set_caption("Tello Control (Focus hier für Tastatur)")
        clock = pygame.time.Clock()

        # --- Stream starten ---
        self.tello.streamoff()
        time.sleep(0.2)
        self.tello.streamon()
        time.sleep(1.0)

        # PyAV versuchen, sonst OpenCV
        use_opencv = False
        try:
            self.frame_read = self.tello.get_frame_read()
            for _ in range(10):
                if self.frame_read.frame is not None:
                    break
                time.sleep(0.05)
            else:
                raise RuntimeError("PyAV liefert keine Frames")
        except Exception as e:
            print("[WARN] PyAV fehlgeschlagen, Fallback auf OpenCV:", e)
            self.frame_read = None
            self.cap = open_tello_stream_with_opencv()
            use_opencv = True

        try:
            running = True
            while running:
                # --- Events ---
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                # Tastaturstatus abfragen (kontinuierlich)
                keys = pygame.key.get_pressed()

                # ESC schließt
                if keys[pygame.K_ESCAPE]:
                    running = False

                # Takeoff/Land
                if keys[pygame.K_t]:
                    if not self.flying:
                        try:
                            self.tello.takeoff()
                            self.flying = True
                        except Exception as e:
                            print("[ERR] takeoff:", e)
                        time.sleep(0.3)  # debounce
                if keys[pygame.K_l]:
                    if self.flying:
                        try:
                            self.tello.land()
                        except Exception as e:
                            print("[ERR] land:", e)
                        self.flying = False
                        # RC auf 0 setzen
                        self.lr = self.fb = self.ud = self.yw = 0
                        time.sleep(0.3)  # debounce

                # Bewegungen setzen
                self.lr = (S if keys[pygame.K_RIGHT] else -S if keys[pygame.K_LEFT] else 0)
                self.fb = (S if keys[pygame.K_UP]    else -S if keys[pygame.K_DOWN] else 0)
                self.ud = (S if keys[pygame.K_w]     else -S if keys[pygame.K_s]    else 0)
                self.yw = (S if keys[pygame.K_d]     else -S if keys[pygame.K_a]    else 0)

                # --- RC-Befehle regelmäßig senden ---
                try:
                    # Auch wenn 0 – die Tello erwartet regelmäßige Updates!
                    self.tello.send_rc_control(self.lr, self.fb, self.ud, self.yw)
                except Exception as e:
                    # Wenn am Boden, ignorierbar
                    pass

                # --- Video anzeigen ---
                if use_opencv:
                    ok, frame = self.cap.read()
                    if ok:
                        cv2.imshow("Tello Video", frame)
                else:
                    frame = self.frame_read.frame
                    if frame is not None:
                        cv2.imshow("Tello Video", frame)

                if cv2.waitKey(1) & 0xFF == 27:  # ESC in OpenCV-Fenster
                    running = False

                clock.tick(FPS)

        finally:
            cv2.destroyAllWindows()
            if self.cap is not None:
                self.cap.release()
            try:
                if self.flying:
                    self.tello.land()
            except Exception:
                pass
            try:
                self.tello.streamoff()
            except Exception:
                pass
            pygame.quit()
            try:
                self.tello.end()
            except Exception:
                pass

def main():
    tello = Tello()
    tello.connect()
    tello.set_speed(10)
    print("SDK:", tello.query_sdk_version())   # sollte 3.x sein
    print("Battery:", tello.get_battery())
    # Optional: Video-Encoder wählen (nicht immer nötig)
    # tello.set_video_direction(Tello.CAMERA_FORWARD)
    # tello.set_video_bitrate(Tello.BITRATE_2MBPS)

    app = Frontend(tello)
    app.run()

if __name__ == "__main__":
    main()