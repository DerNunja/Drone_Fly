import cv2

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