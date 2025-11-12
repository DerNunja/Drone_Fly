import cv2
import mediapipe as mp

class HandDetector:
    """Gibt Hand-Mittelpunkt (cx,cy) im Bild zurück oder None."""
    def __init__(self, max_num_hands=1, det_conf=0.6, track_conf=0.6):
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=max_num_hands,
            model_complexity=0, min_detection_confidence=det_conf,
            min_tracking_confidence=track_conf
        )

    def find_hand_center(self, frame_bgr):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        if not res.multi_hand_landmarks:
            return None
        h, w = frame_bgr.shape[:2]
        lm = res.multi_hand_landmarks[0]
        idxs = [0,5,9,13,17]
        cx = sum(lm.landmark[i].x for i in idxs)/len(idxs)*w
        cy = sum(lm.landmark[i].y for i in idxs)/len(idxs)*h
        return int(cx), int(cy)