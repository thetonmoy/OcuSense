# run_all.py
# Reads:
#  - Arduino Serial: BH1750 (lux) + VL53L0X (distance)
#  - Webcam: blink metrics via MediaPipe FaceMesh
# Outputs:
#  - current.json (live snapshot for your PHP dashboard)
# Shows:
#  - Professional HUD overlay on the camera window
# NOTE: HUD shows distance in *cm* (not mm)

import cv2
import mediapipe as mp
import serial
import time
import json
import os
import re
from math import sqrt
from collections import deque


# =========================
# CONFIG (EDIT THESE)
# =========================

# --- Serial / Arduino ---
SERIAL_PORT = "COM4"        # <<< your Nano
BAUD = 9600                 # must match Arduino Serial.begin(9600)
SERIAL_TIMEOUT = 0          # non-blocking

# --- Output JSON for dashboard ---
# Example for XAMPP:
# OUTPUT_JSON = r"C:\xampp\htdocs\eye_dashboard\current.json"
OUTPUT_JSON = "current.json"

# --- Camera index (0 is default webcam) ---
CAMERA_INDEX = 0

# --- Mirror preview (fixes left/right feeling) ---
MIRROR_PREVIEW = True

# --- Smoothing for sensor values (moving average) ---
SMOOTH_N = 5

# =========================
# BLINK DETECTION (IMPROVED)
# =========================

# Fallback threshold (used only before calibration completes)
RATIO_THRESHOLD = 0.21

# How many consecutive frames must be "closed" to count a blink
CONSEC_FRAMES = 3

# EAR smoothing to reduce jitter
EAR_SMOOTH_N = 3

# Auto-calibrate threshold for your face/camera
AUTO_CALIBRATE_EAR = True
CALIBRATE_SECONDS = 2.0     # collect baseline EAR for first N seconds
EAR_DROP_FACTOR = 0.75      # threshold = baseline * factor (0.70-0.85 typical)

# Prevent double-counting the same blink
BLINK_COOLDOWN_S = 0.25

# Ignore obviously broken EAR values (FaceMesh jitter / bad detection)
EAR_MIN_VALID = 0.05
EAR_MAX_VALID = 0.60

# Optional: show debug EAR & threshold on the video
# SHOW_EAR_DEBUG = True
SHOW_EAR_DEBUG = False


# --- DISTANCE RULES ---
# < DIST_TOO_CLOSE_CM_MAX  => TOO CLOSE (ALERT)
# DIST_TOO_CLOSE_CM_MAX..DIST_SAFE_CM_MAX => SAFE (OK)
# > DIST_SAFE_CM_MAX       => TOO FAR (WARN)
DIST_TOO_CLOSE_CM_MAX = 35
DIST_SAFE_CM_MAX = 55

# --- Lux thresholds (lx) ---
LUX_TOO_DARK = 100
LUX_WARN_DARK = 150
LUX_WARN_BRIGHT = 350
LUX_TOO_BRIGHT = 1000

# --- Blink rate thresholds (blinks/min over last 60s) ---
BLINK_TOO_LOW = 8
BLINK_WARN_LOW = 12
BLINK_WARN_HIGH = 30

# --- Time since last blink (seconds) ---
NO_BLINK_WARN_S = 12
NO_BLINK_ALERT_S = 20

# --- 20-20-20 reminder ---
BREAK_REMINDER_EVERY_S = 20 * 60
BREAK_LOOK_AWAY_S = 20


# =========================
# SERIAL PARSING
# =========================
# Expected Arduino line:
# "L:245.7 lx; D:612 mm"
LINE_RE = re.compile(r"L:\s*([0-9.]+)\s*lx;?\s*D:\s*(\d+)\s*mm", re.IGNORECASE)


# =========================
# UI HELPERS (professional HUD)
# =========================

def level_color(level: str):
    """BGR colors for OpenCV drawing."""
    if level == "OK":
        return (60, 200, 120)
    if level == "WARN":
        return (0, 200, 255)
    return (60, 60, 255)  # ALERT

def blend_panel(frame, x, y, w, h, alpha=0.55):
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + w, y + h), (15, 15, 15), -1)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

def draw_badge(frame, x, y, text, level, font, scale=0.55):
    pad_x, pad_y = 10, 6
    (tw, th), _ = cv2.getTextSize(text, font, scale, 2)
    bw, bh = tw + pad_x * 2, th + pad_y * 2
    color = level_color(level)

    cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, -1)
    cv2.putText(frame, text, (x + pad_x, y + bh - pad_y),
                font, scale, (10, 10, 10), 2, cv2.LINE_AA)

def draw_row(frame, x, y, label, value, level, font, w=420):
    cv2.putText(frame, label, (x, y), font, 0.58, (220, 220, 220), 1, cv2.LINE_AA)

    value = value if value is not None else "--"
    (tw, _), _ = cv2.getTextSize(value, font, 0.58, 1)
    vx = x + w - tw - 28
    cv2.putText(frame, value, (vx, y), font, 0.58, (245, 245, 245), 1, cv2.LINE_AA)

    cv2.circle(frame, (x + w - 12, y - 7), 6, level_color(level), -1)

def mm_to_cm(mm):
    if mm is None:
        return None
    return mm / 10.0

def format_distance_cm(dist_mm):
    if dist_mm is None:
        return None
    return f"{mm_to_cm(dist_mm):.1f} cm"

def format_lux(lux):
    if lux is None:
        return None
    return f"{lux:.0f} lx"

def format_bpm(bpm):
    if bpm is None:
        return None
    return f"{int(bpm)}/min"

def format_seconds(s):
    if s is None:
        return None
    return f"{int(s)} s"

def draw_hud(frame, payload):
    font = cv2.FONT_HERSHEY_SIMPLEX

    x, y = 20, 20
    w, h = 520, 360
    blend_panel(frame, x, y, w, h, alpha=0.55)

    cv2.putText(frame, "OcuSense", (x + 16, y + 36),
                font, 0.86, (245, 245, 245), 2, cv2.LINE_AA)
    cv2.putText(frame, time.strftime("%H:%M:%S"), (x + w - 120, y + 36),
                font, 0.60, (205, 205, 205), 1, cv2.LINE_AA)

    overall = payload.get("overall_status", "WARN")
    # draw_badge(frame, x + 16, y + 52, f"OVERALL: {overall}", overall, font, scale=0.60)
    draw_badge(frame, x + 16, y + 52, f"{overall}", overall, font, scale=0.60)

    statuses = payload.get("statuses", {})
    dist_lv = statuses.get("distance", {}).get("level", "WARN")
    light_lv = statuses.get("light", {}).get("level", "WARN")
    br_lv = statuses.get("blink_rate", {}).get("level", "WARN")
    bg_lv = statuses.get("blink_gap", {}).get("level", "WARN")

    row_x = x + 16
    row_w = w - 32
    row_y0 = y + 120
    step = 34

    draw_row(frame, row_x, row_y0 + 0 * step, "Distance",   format_distance_cm(payload.get("distance_mm")), dist_lv, font, w=row_w)
    draw_row(frame, row_x, row_y0 + 1 * step, "Light",      format_lux(payload.get("lux")),               light_lv, font, w=row_w)
    draw_row(frame, row_x, row_y0 + 2 * step, "Blinks/min", format_bpm(payload.get("blinks_per_min")),    br_lv, font, w=row_w)
    draw_row(frame, row_x, row_y0 + 3 * step, "Last blink", format_seconds(payload.get("seconds_since_last_blink")), bg_lv, font, w=row_w)

    sep_y = row_y0 + 3 * step + 20
    cv2.line(frame, (x + 16, sep_y), (x + w - 16, sep_y), (80, 80, 80), 1, cv2.LINE_AA)

    face_ok = payload.get("face_detected", False)
    face_text = "Face: OK" if face_ok else "Face: Not detected"
    face_level = "OK" if face_ok else "WARN"

    footer_y = sep_y + 28
    cv2.putText(frame, f"Total blinks: {payload.get('total_blinks', 0)}",
                (x + 16, footer_y), font, 0.58, (220, 220, 220), 1, cv2.LINE_AA)

    (ftw, _), _ = cv2.getTextSize(face_text, font, 0.58, 2)
    cv2.putText(frame, face_text,
                (x + w - 16 - ftw, footer_y), font, 0.58, level_color(face_level), 2, cv2.LINE_AA)

    alerts = payload.get("alerts", [])
    if alerts:
        alerts_title_y = footer_y + 28
        cv2.putText(frame, "Alerts", (x + 16, alerts_title_y),
                    font, 0.56, (205, 205, 205), 1, cv2.LINE_AA)

        ay = alerts_title_y + 22
        line_h = 20
        for a in alerts[:2]:
            short = a[:72] + ("..." if len(a) > 72 else "")
            cv2.putText(frame, f"- {short}", (x + 16, ay),
                        font, 0.50, (235, 235, 235), 1, cv2.LINE_AA)
            ay += line_h

    if payload.get("break_banner", False):
        bx1, by1 = x, y + h + 14
        bx2, by2 = x + w, y + h + 58
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (30, 30, 30), -1)
        cv2.putText(frame, "20-20-20: Look away for 20 seconds",
                    (bx1 + 16, by1 + 32), font, 0.74, (245, 245, 245), 2, cv2.LINE_AA)


# =========================
# LOGIC HELPERS
# =========================

def avg_or_none(q):
    return (sum(q) / len(q)) if q else None

def worst_level(levels):
    rank = {"OK": 0, "WARN": 1, "ALERT": 2}
    best = "OK"
    for lv in levels:
        if lv is None:
            continue
        if rank.get(lv, 1) > rank[best]:
            best = lv
    return best

def atomic_write_json(path, data, retries=12, wait=0.02):
    """Retry to avoid Windows file-lock crash when dashboard reads the JSON."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f)

    for _ in range(retries):
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            time.sleep(wait)

    os.replace(tmp, path)

def evaluate_eye_health(lux, dist_mm, blinks_per_min, seconds_since_last_blink, face_detected):
    safe_band = f"{int(DIST_TOO_CLOSE_CM_MAX)}-{int(DIST_SAFE_CM_MAX)} cm"

    alerts = []
    statuses = {
        "distance":   {"level": "WARN", "msg": "No reading"},
        "light":      {"level": "WARN", "msg": "No reading"},
        "blink_rate": {"level": "WARN", "msg": "No face"},
        "blink_gap":  {"level": "WARN", "msg": "No face"},
    }

    # ---- Distance ----
    if dist_mm is None:
        statuses["distance"] = {"level": "ALERT", "msg": "Distance missing"}
        alerts.append("Distance: no data (check VL53L0X).")
    else:
        dist_cm = dist_mm / 10.0
        if dist_cm <= 0.5:
            statuses["distance"] = {"level": "ALERT", "msg": f"Invalid ({dist_cm:.1f} cm)"}
            alerts.append("Distance reading looks invalid. Check sensor aim/angle.")
        elif dist_cm < DIST_TOO_CLOSE_CM_MAX:
            statuses["distance"] = {"level": "ALERT", "msg": f"Too close ({dist_cm:.1f} cm)"}
            alerts.append(f"Too close. Move back to {safe_band}.")
        elif dist_cm <= DIST_SAFE_CM_MAX:
            statuses["distance"] = {"level": "OK", "msg": f"Safe ({dist_cm:.1f} cm)"}
        else:
            statuses["distance"] = {"level": "WARN", "msg": f"Too far ({dist_cm:.1f} cm)"}
            alerts.append(f"Too far. Move closer to {safe_band}.")

    # ---- Light ----
    if lux is None:
        statuses["light"] = {"level": "ALERT", "msg": "Light missing"}
        alerts.append("Light: no data (check BH1750).")
    else:
        if lux < LUX_TOO_DARK:
            statuses["light"] = {"level": "ALERT", "msg": f"Too dark ({int(lux)} lx)"}
            alerts.append("Too dark. Turn on a lamp or increase ambient light.")
        elif lux < LUX_WARN_DARK:
            statuses["light"] = {"level": "WARN", "msg": f"Dim ({int(lux)} lx)"}
            alerts.append("Lighting is low. Improve ambient lighting.")
        elif lux > LUX_TOO_BRIGHT:
            statuses["light"] = {"level": "ALERT", "msg": f"Too bright ({int(lux)} lx)"}
            alerts.append("Very bright. Reduce glare / reposition lighting.")
        elif lux > LUX_WARN_BRIGHT:
            statuses["light"] = {"level": "WARN", "msg": f"Bright ({int(lux)} lx)"}
            alerts.append("Quite bright. Watch for glare on the screen.")
        else:
            statuses["light"] = {"level": "OK", "msg": f"OK ({int(lux)} lx)"}

    # ---- Blink metrics ----
    if not face_detected:
        statuses["blink_rate"] = {"level": "WARN", "msg": "Face not detected"}
        statuses["blink_gap"]  = {"level": "WARN", "msg": "Face not detected"}
        alerts.append("Face not detected (blink monitoring paused).")
    else:
        if blinks_per_min is None:
            statuses["blink_rate"] = {"level": "WARN", "msg": "Unknown"}
        else:
            if blinks_per_min < BLINK_TOO_LOW:
                statuses["blink_rate"] = {"level": "ALERT", "msg": f"Very low ({blinks_per_min}/min)"}
                alerts.append("Blink rate very low. Blink consciously / take a short break.")
            elif blinks_per_min < BLINK_WARN_LOW:
                statuses["blink_rate"] = {"level": "WARN", "msg": f"Low ({blinks_per_min}/min)"}
                alerts.append("Blink rate low. Try blinking more often.")
            elif blinks_per_min > BLINK_WARN_HIGH:
                statuses["blink_rate"] = {"level": "WARN", "msg": f"High ({blinks_per_min}/min)"}
                alerts.append("Blink rate high (possible irritation/dryness).")
            else:
                statuses["blink_rate"] = {"level": "OK", "msg": f"OK ({blinks_per_min}/min)"}

        if seconds_since_last_blink is None:
            statuses["blink_gap"] = {"level": "WARN", "msg": "No blink yet"}
        else:
            if seconds_since_last_blink > NO_BLINK_ALERT_S:
                statuses["blink_gap"] = {"level": "ALERT", "msg": f"No blink {int(seconds_since_last_blink)}s"}
                alerts.append("No blink for a while. Blink now.")
            elif seconds_since_last_blink > NO_BLINK_WARN_S:
                statuses["blink_gap"] = {"level": "WARN", "msg": f"No blink {int(seconds_since_last_blink)}s"}
                alerts.append("Long time since last blink. Try blinking.")
            else:
                statuses["blink_gap"] = {"level": "OK", "msg": f"{int(seconds_since_last_blink)}s ago"}

    overall = worst_level([
        statuses["distance"]["level"],
        statuses["light"]["level"],
        statuses["blink_rate"]["level"],
        statuses["blink_gap"]["level"],
    ])

    return statuses, alerts, overall


# =========================
# BLINK DETECTION SETUP
# =========================

LEFT_EYE  = [362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398]
RIGHT_EYE = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246]

def euclideanDistance(p0, p1):
    x0, y0 = p0
    x1, y1 = p1
    return sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

def blinkRatio(landmarks, right_indices, left_indices):
    r_lc = landmarks[right_indices[0]]
    r_rc = landmarks[right_indices[8]]
    r_top = landmarks[right_indices[12]]
    r_bot = landmarks[right_indices[4]]

    l_lc = landmarks[left_indices[0]]
    l_rc = landmarks[left_indices[8]]
    l_top = landmarks[left_indices[12]]
    l_bot = landmarks[left_indices[4]]

    r_h = euclideanDistance(r_lc, r_rc)
    l_h = euclideanDistance(l_lc, l_rc)
    if r_h == 0 or l_h == 0:
        return 0.0

    r_v = euclideanDistance(r_top, r_bot)
    l_v = euclideanDistance(l_top, l_bot)

    return ((r_v / r_h) + (l_v / l_h)) / 2.0


# =========================
# MAIN
# =========================

def main():
    # Serial open
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=SERIAL_TIMEOUT)
    ser.reset_input_buffer()

    lux_q = deque(maxlen=SMOOTH_N)
    dist_q = deque(maxlen=SMOOTH_N)

    def read_serial_latest():
        while ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            m = LINE_RE.search(line)
            if m:
                lux_q.append(float(m.group(1)))
                dist_q.append(int(m.group(2)))

    # MediaPipe FaceMesh
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.7
    )

    cap = cv2.VideoCapture(CAMERA_INDEX)

    # Blink state
    counter = 0
    total_blinks = 0
    blink_times = deque()
    last_blink_time = 0.0

    # EAR smoothing + calibration state
    ear_hist = deque(maxlen=EAR_SMOOTH_N)
    ear_threshold = RATIO_THRESHOLD
    ear_auto_calib = AUTO_CALIBRATE_EAR
    calib_ears = []
    calib_end = time.time() + CALIBRATE_SECONDS

    last_break_reminder = time.time()
    break_message_until = 0

    last_json_write = 0

    while True:
        now = time.time()

        # Read Arduino data
        read_serial_latest()
        lux = avg_or_none(lux_q)
        dist_mm = avg_or_none(dist_q)

        # Read camera frame
        ret, frame = cap.read()
        if not ret:
            break

        if MIRROR_PREVIEW:
            frame = cv2.flip(frame, 1)

        frame = cv2.resize(frame, None, fx=1.4, fy=1.4, interpolation=cv2.INTER_CUBIC)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_mesh.process(rgb)

        ear = None
        blinks_per_min = None
        since_last_blink = None
        face_detected = False

        if results.multi_face_landmarks:
            face_detected = True

            h, w = frame.shape[:2]
            mesh = [(int(p.x * w), int(p.y * h)) for p in results.multi_face_landmarks[0].landmark]

            # ---- EAR raw ----
            ear_raw = blinkRatio(mesh, RIGHT_EYE, LEFT_EYE)

            # Reject garbage EAR (jittery detection)
            if EAR_MIN_VALID <= ear_raw <= EAR_MAX_VALID:
                # Smooth EAR
                ear_hist.append(ear_raw)
                ear = sum(ear_hist) / len(ear_hist)

                # Auto-calibration (baseline = median EAR over first N seconds)
                if ear_auto_calib:
                    if now < calib_end:
                        calib_ears.append(ear)
                    else:
                        if calib_ears:
                            calib_ears_sorted = sorted(calib_ears)
                            baseline = calib_ears_sorted[len(calib_ears_sorted) // 2]
                            ear_threshold = baseline * EAR_DROP_FACTOR
                            print(f"[CALIB] baseline={baseline:.3f}  threshold={ear_threshold:.3f}")
                        ear_auto_calib = False
                        calib_ears.clear()

                # Blink detection with debounce
                if ear < ear_threshold:
                    counter += 1
                else:
                    if counter >= CONSEC_FRAMES:
                        if (now - last_blink_time) >= BLINK_COOLDOWN_S:
                            total_blinks += 1
                            blink_times.append(now)
                            last_blink_time = now
                    counter = 0

            else:
                # If EAR is invalid this frame, don’t trigger blink logic
                counter = 0

            # blinks/min: count events in last 60 seconds
            while blink_times and (now - blink_times[0] > 60):
                blink_times.popleft()
            blinks_per_min = len(blink_times)

            since_last_blink = (now - blink_times[-1]) if blink_times else None

            # Optional debug text (outside HUD)
            if SHOW_EAR_DEBUG:
                msg = f"EAR {ear:.3f}  TH {ear_threshold:.3f}" if ear is not None else f"EAR --  TH {ear_threshold:.3f}"
                if ear_auto_calib and now < calib_end:
                    msg += "  (calibrating)"
                cv2.putText(frame, msg, (20, frame.shape[0] - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)

        else:
            # If face lost, reset counters to avoid false blinks on re-detect
            counter = 0
            ear_hist.clear()

        # Evaluate health rules
        statuses, alerts, overall = evaluate_eye_health(
            lux=lux,
            dist_mm=dist_mm,
            blinks_per_min=blinks_per_min,
            seconds_since_last_blink=since_last_blink,
            face_detected=face_detected
        )

        # 20-20-20 reminder banner
        if (now - last_break_reminder > BREAK_REMINDER_EVERY_S) and face_detected:
            break_message_until = now + BREAK_LOOK_AWAY_S
            last_break_reminder = now
        break_banner = now < break_message_until

        # HUD payload (also written to JSON)
        hud_payload = {
            "ts": now,
            "lux": lux,
            "distance_mm": dist_mm,
            "distance_cm": mm_to_cm(dist_mm) if dist_mm is not None else None,
            "total_blinks": total_blinks,
            "blinks_per_min": blinks_per_min,
            "ear": ear,
            "ear_threshold": ear_threshold,
            "seconds_since_last_blink": since_last_blink,
            "face_detected": face_detected,
            "overall_status": overall,
            "statuses": statuses,
            "alerts": alerts,
            "break_banner": break_banner
        }

        draw_hud(frame, hud_payload)

        # Write JSON ~5 times/sec
        if now - last_json_write > 0.2:
            try:
                atomic_write_json(OUTPUT_JSON, hud_payload)
            except PermissionError:
                pass
            last_json_write = now

        cv2.imshow("OcuSense", frame)
        if cv2.waitKey(2) == 27:  # ESC
            break

    cv2.destroyAllWindows()
    cap.release()
    ser.close()


if __name__ == "__main__":
    main()