from config import (
    FOLLOW_DEADBAND_PX, FOLLOW_KP_LR, FOLLOW_KP_FB, FOLLOW_KP_UD,
    FOLLOW_MAX_LR, FOLLOW_MAX_FB, FOLLOW_MAX_UD,
    ALT_MIN_CM, ALT_MAX_CM
)

def clamp(v, lo, hi): 
    return lo if v < lo else hi if v > hi else v

def compute_controls(dx, dy, using_down, est_alt_cm):
    """
    OpenCV: y nach unten positiv.
    Returns (lr, fb, ud). Bei Down-Cam kommt ud extern vom Hover-Regler.
    """
    lr = fb = ud = 0

    # links/rechts (dx>0 => rechts => +lr)
    if abs(dx) > FOLLOW_DEADBAND_PX:
        lr = clamp(int(FOLLOW_KP_LR * dx), -FOLLOW_MAX_LR, FOLLOW_MAX_LR)

    if using_down:
        # Down-Cam: dy>0 => nach vorne
        if abs(dy) > FOLLOW_DEADBAND_PX:
            fb = clamp(int(FOLLOW_KP_FB * dy), -FOLLOW_MAX_FB, FOLLOW_MAX_FB)
        ud = None
    else:
        # Front-Cam: dy>0 => steigen
        if abs(dy) > FOLLOW_DEADBAND_PX:
            ud = clamp(int(FOLLOW_KP_UD * dy), -FOLLOW_MAX_UD, FOLLOW_MAX_UD)
        else:
            ud = 0
        # Soft Guards
        if est_alt_cm is not None:
            if est_alt_cm < ALT_MIN_CM:
                ud = max(ud, +15)
            elif est_alt_cm > ALT_MAX_CM:
                ud = min(ud, -15)

    return lr, fb, ud