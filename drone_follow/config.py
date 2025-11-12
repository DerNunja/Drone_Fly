BACKEND = "mediapipe"            # "mediapipe" | "yolo"
YOLO_WEIGHTS = "hand_detector.pt"
YOLO_IMGSZ   = 320
YOLO_CONF    = 0.35

# Hover / Höhe
HOVER_TARGET_CM   = 40
HOVER_DEADBAND_CM = 2.0
HOVER_KP          = 2.0
HOVER_MAX_UD      = 40
ALT_MIN_CM        = 20
ALT_MAX_CM        = 120
LP_WINDOW         = 7
SAMPLE_HZ         = 25

# Follow Gains
FOLLOW_DEADBAND_PX = 50
FOLLOW_KP_LR       = 0.20
FOLLOW_KP_FB       = 0.25   # nur Down-Cam
FOLLOW_KP_UD       = 0.35   # nur Front-Cam
FOLLOW_MAX_LR      = 25
FOLLOW_MAX_FB      = 25
FOLLOW_MAX_UD      = 35
EMA_ALPHA          = 0.5

# Fail-Safe
STARTUP_GRACE_S   = 8.0
HAND_LOST_HOVER_S = 5.0
HAND_LOST_LAND_S  = 12.0

DEBUG_EVERY         = 0.25
BATTERY_POLL_EVERY  = 1.0
SHOW_HELP_DEFAULT   = True  # Hotkey-Overlay