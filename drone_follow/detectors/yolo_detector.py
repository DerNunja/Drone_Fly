import cv2
import torch
from ultralytics import YOLO

class HandDetector:
    """YOLO-Hand-Detektor (nimmt größte Box als Hand)."""
    def __init__(self, weights: str, imgsz=320, conf=0.35):
        if not weights:
            raise ValueError("YOLO_WEIGHTS darf nicht leer sein.")
        self.model = YOLO(weights)
        self.imgsz = imgsz
        self.conf  = conf
        self.device = "0" if torch.cuda.is_available() else "cpu"

    def find_hand_center(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        size = self.imgsz
        im = cv2.resize(frame_bgr, (size, size))
        r = self.model.predict(source=im, imgsz=size, conf=self.conf, verbose=False, device=self.device)[0]
        if len(r.boxes) == 0:
            return None
        xyxy = r.boxes.xyxy
        areas = (xyxy[:,2]-xyxy[:,0])*(xyxy[:,3]-xyxy[:,1])
        i = int(areas.argmax().item())
        x1,y1,x2,y2 = xyxy[i].tolist()
        sx, sy = w/size, h/size
        cx = (x1+x2)/2 * sx
        cy = (y1+y2)/2 * sy
        return int(cx), int(cy)
