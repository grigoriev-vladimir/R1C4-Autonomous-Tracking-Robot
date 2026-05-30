"""
reconnaissance.py — Tourelle de suivi dual-thread
==================================================
Corrections v5 :
  - Tourelle moins réactive : DEAD_ZONE_PX agrandi, DEG_PER_PX réduit, ROT_PERIOD_S allongé
  - Commande "stop" dans le terminal → programme s'arrête proprement
  - Vérin retiré complètement
"""

import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import serial

# ══════════════════════════════════════════════════════════════════════════════
# ARRÊT PROPRE PAR "stop" DANS LE TERMINAL
# ══════════════════════════════════════════════════════════════════════════════
stop_event = threading.Event()

def _keyboard_listener():
    """Thread qui écoute le terminal : tape 'stop' + Entrée pour arrêter."""
    while not stop_event.is_set():
        try:
            line = input()
            if line.strip().lower() == "stop":
                print("Commande STOP reçue — arrêt en cours...")
                stop_event.set()
        except EOFError:
            break

_kb_thread = threading.Thread(target=_keyboard_listener, daemon=True)
_kb_thread.start()

# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ══════════════════════════════════════════════════════════════════════════════
PORT        = "COM3"        # ← votre port ESP32
BAUD        = 115200
SANS_SERIE  = False         # True = test sans ESP32

CAM_INDEX   = 1
# Résolution NATIVE de la caméra (avant rotation)
CAM_W       = 640
CAM_H       = 480
# Après rotation 90° : largeur et hauteur s'inversent
FRAME_W     = CAM_H        # = 480
FRAME_H     = CAM_W        # = 640

MODEL_PATH  = Path("lbph_model.yml")
LABELS_PATH = Path("labels.json")

FACE_SIZE   = (200, 200)
MIN_FACE    = (80, 80)
THRESHOLD   = 75.0        # distance LBPH max — ajustez selon vos distances
TRACK_FACE_THRESHOLD = 125.0   # plus tolérant pendant le suivi, utile profil/dos

# Tourelle
DEAD_ZONE_PX  = 50          # zone morte large → ignore les petits écarts
DEG_PER_PX    = 0.1        # gain très doux
MAX_ROT_DEG   = 15.0        # amplitude max réduite
ROT_PERIOD_S  = 0.25        # commande toutes les 250ms max

# Balayage recherche
SCAN_STEP_DEG  = 10
SCAN_LIMIT_DEG = 180
SCAN_PERIOD_S  = 1.8

# Tracker
TRACKER_LOST_LIMIT = 45

# Validation forme personne pour éviter de tracker le mur
MIN_PERSON_RATIO = 0.60   # hauteur / largeur minimale
MAX_PERSON_RATIO = 8.00   # hauteur / largeur maximale
MIN_BODY_AREA    = 1200   # surface min de la boîte


# ══════════════════════════════════════════════════════════════════════════════
# FILTRE DE KALMAN
# ══════════════════════════════════════════════════════════════════════════════
def make_kalman() -> cv2.KalmanFilter:
    kf = cv2.KalmanFilter(4, 2)
    dt = 1.0
    kf.transitionMatrix = np.array(
        [[1, 0, dt, 0],
         [0, 1,  0, dt],
         [0, 0,  1,  0],
         [0, 0,  0,  1]], dtype=np.float32)
    kf.measurementMatrix = np.array(
        [[1, 0, 0, 0],
         [0, 1, 0, 0]], dtype=np.float32)
    kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 5e-3
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5e-2
    kf.errorCovPost        = np.eye(4, dtype=np.float32)
    return kf


def kalman_get_xy(matrix) -> tuple:
    """Extrait (x, y) depuis la matrice état Kalman (4x1) en toute sécurité."""
    return int(matrix[0][0]), int(matrix[1][0])


# ══════════════════════════════════════════════════════════════════════════════
# COMMUNICATION SÉRIE
# ══════════════════════════════════════════════════════════════════════════════
def wait_for_done(ser, timeout_s=8.0) -> str:
    if ser is None:
        return "DONE"
    start = time.time()
    while time.time() - start < timeout_s:
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print("Carte ->", line)
                if line == "DONE":
                    return line
        time.sleep(0.05)
    return ""


def send_rot(ser, deg: float):
    if ser is None:
        return

    deg = max(-MAX_ROT_DEG, min(MAX_ROT_DEG, deg))
    ideg = int(round(deg))

    if ideg == 0:
        return

    sign = "+" if ideg >= 0 else "-"
    cmd = f"ROT:{sign}{abs(ideg):03d}\n"
    print("PC ->", cmd.strip())
    ser.write(cmd.encode())
    ser.flush()


def send_buzzer(ser, duration_ms: int = 800):
    """Déclenche le buzzer de l'ESP32 pour signaler une perte de cible."""
    if ser is None:
        print("[BUZZER] bip simulé (pas de série)")
        return
    cmd = f"BUZ:{duration_ms:04d}\n"
    print("PC ->", cmd.strip())
    ser.write(cmd.encode())
    ser.flush()

# ══════════════════════════════════════════════════════════════════════════════
# CHARGEMENT RESSOURCES
# ══════════════════════════════════════════════════════════════════════════════
def load_labels(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    return {int(v): k for k, v in data["name_to_id"].items()}


def load_cascade() -> cv2.CascadeClassifier:
    p = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    c = cv2.CascadeClassifier(p)
    if c.empty():
        raise RuntimeError("Haar cascade introuvable")
    return c


def preprocess_face(bgr_roi: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, FACE_SIZE, interpolation=cv2.INTER_AREA)
    return cv2.equalizeHist(gray)


def face_to_body_box(x, y, w, h, frame_h, frame_w=None) -> tuple:
    """
    Boîte de suivi agrandie : tête + torse + bras.
    Elle reste centrée sur toi, mais évite de prendre trop de mur.
    """
    margin_x = int(w * 1.05)
    body_h   = int(h * 4.3)

    bx = max(0, x - margin_x)
    by = max(0, y - int(h * 0.25))
    bw = w + 2 * margin_x
    bh = min(body_h, frame_h - by)

    if frame_w is not None:
        bw = min(bw, frame_w - bx)

    return (int(bx), int(by), int(bw), int(bh))


# ══════════════════════════════════════════════════════════════════════════════
# ÉTAT PARTAGÉ ENTRE THREADS
# ══════════════════════════════════════════════════════════════════════════════
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.phase         = "SEARCH"   # "SEARCH" | "TRACK" | "RELOCK"
        self.det_name      = ""
        self.det_dist      = 999.0
        self.face_box      = None       # (x,y,w,h) visage reconnu
        self.body_box      = None       # (x,y,w,h) corps HOG
        self.recognized    = False      # visage reconnu ce cycle
        self.frame         = None
        self.stop          = False


# ══════════════════════════════════════════════════════════════════════════════
# THREAD DÉTECTION (arrière-plan, ~12 Hz)
# ══════════════════════════════════════════════════════════════════════════════
def detection_thread(state: SharedState, recognizer, cascade, hog, id_to_name):
    while not state.stop:
        with state.lock:
            if state.frame is None:
                pass
            else:
                frame = state.frame.copy()
                phase = state.phase

        time.sleep(0.08)

        with state.lock:
            if state.frame is None:
                continue
            frame = state.frame.copy()
            phase = state.phase

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if phase == "SEARCH":
            faces = cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=MIN_FACE)

            recognized = False
            for (x, y, w, h) in faces:
                roi      = frame[y:y+h, x:x+w]
                face_img = preprocess_face(roi)
                lid, dist = recognizer.predict(face_img)
                name = id_to_name.get(lid, f"ID:{lid}")
                print(f"[DET] {name}  dist={dist:.1f}")

                with state.lock:
                    state.det_name = name
                    state.det_dist = dist

                if dist <= THRESHOLD:
                    # Chercher corps HOG
                    body = _find_body_hog(hog, frame)
                    if body is None:
                        # Pas de corps HOG → estimer depuis le visage
                        body = face_to_body_box(x, y, w, h, frame.shape[0], frame.shape[1])

                    with state.lock:
                        state.recognized = True
                        state.face_box   = (x, y, w, h)
                        state.body_box   = body
                    recognized = True
                    break

            if not recognized:
                with state.lock:
                    state.recognized = False

        elif phase == "RELOCK":
            # Phase RELOCK : on cherche TON visage spécifiquement (LBPH strict)
            faces = cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=MIN_FACE)

            recognized = False
            for (x, y, w, h) in faces:
                roi      = frame[y:y+h, x:x+w]
                face_img = preprocess_face(roi)
                lid, dist = recognizer.predict(face_img)
                name = id_to_name.get(lid, f"ID:{lid}")
                print(f"[RELOCK] {name}  dist={dist:.1f}")

                with state.lock:
                    state.det_name = name
                    state.det_dist = dist

                if dist <= THRESHOLD:
                    body = _find_body_hog(hog, frame)
                    if body is None:
                        body = face_to_body_box(x, y, w, h, frame.shape[0], frame.shape[1])

                    with state.lock:
                        state.recognized = True
                        state.face_box   = (x, y, w, h)
                        state.body_box   = body
                    recognized = True
                    break

            if not recognized:
                with state.lock:
                    state.recognized = False

        else:
            # Phase TRACK : recalage par visage reconnu, pas par HOG.
            faces = cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=MIN_FACE)

            for (x, y, w, h) in faces:
                roi = frame[y:y+h, x:x+w]
                face_img = preprocess_face(roi)
                lid, dist = recognizer.predict(face_img)

                if dist <= TRACK_FACE_THRESHOLD:
                    body = face_to_body_box(x, y, w, h, frame.shape[0], frame.shape[1])
                    with state.lock:
                        state.body_box = body
                    break


def _find_body_hog(hog, frame):
    return find_body_near_tracker(hog, frame, None)



def is_person_like_box(box, frame_shape) -> bool:
    """
    Filtre permissif : refuse seulement les boîtes absurdes.
    Avant c'était trop strict, donc ça affichait souvent "pas assez personne".
    """
    if box is None:
        return False

    x, y, w, h = [int(v) for v in box]
    if w <= 0 or h <= 0:
        return False

    frame_h, frame_w = frame_shape[:2]
    area = w * h

    if area < 1200:
        return False

    if w > frame_w * 0.95 or h > frame_h * 0.98:
        return False

    if w < 15 or h < 25:
        return False

    return True



def box_center(box):
    x, y, w, h = [int(v) for v in box]
    return x + w // 2, y + h // 2


def find_body_near_tracker(hog, frame, previous_box=None):
    """
    Cherche une silhouette humaine avec HOG, mais refuse les boîtes absurdes.
    Si on a une position précédente, on prend la détection proche de l'ancien centre.
    """
    rects, _ = hog.detectMultiScale(
        frame, winStride=(8, 8), padding=(4, 4), scale=1.05,
        useMeanshiftGrouping=True)

    candidates = []
    for r in rects:
        box = tuple(int(v) for v in r)
        if is_person_like_box(box, frame.shape):
            candidates.append(box)

    if not candidates:
        return None

    if previous_box is None:
        # Si aucune ancienne position : plus grande boîte personne valide
        return max(candidates, key=lambda b: b[2] * b[3])

    pcx, pcy = box_center(previous_box)

    def score(b):
        cx, cy = box_center(b)
        dist2 = (cx - pcx) ** 2 + (cy - pcy) ** 2
        area = b[2] * b[3]
        # proche de l'ancien tracker + surface correcte
        return dist2 - area * 0.15

    return min(candidates, key=score)


# ══════════════════════════════════════════════════════════════════════════════
# PROGRAMME PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════
def main():

    # ── Série ─────────────────────────────────────────────────────────────────
    ser = None
    if not SANS_SERIE:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(3)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"Série ouverte sur {PORT}")
    else:
        print("MODE SANS SERIE")

    # ── Chargement ────────────────────────────────────────────────────────────
    id_to_name = load_labels(LABELS_PATH)
    recognizer = cv2.face.LBPHFaceRecognizer_create()
    recognizer.read(str(MODEL_PATH))
    cascade    = load_cascade()
    hog        = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # ── Caméra ────────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # réduit la latence
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    if not cap.isOpened():
        raise RuntimeError(f"Impossible d'ouvrir la caméra {CAM_INDEX}")

    # Laisser le temps à la caméra de s'initialiser
    time.sleep(1.0)
    for _ in range(10):
        cap.read()  # vider les premières frames noires

    # Centre horizontal après rotation
    cx = FRAME_W // 2   # = 240 (sur 480px de large après rotation)

    # ── Kalman ────────────────────────────────────────────────────────────────
    kf          = make_kalman()
    kalman_init = False

    # ── Tracker CSRT ──────────────────────────────────────────────────────────
    tracker      = None
    tracker_lost = 0
    previous_good_box = None

    # ── Thread détection ──────────────────────────────────────────────────────
    state = SharedState()
    det_t = threading.Thread(
        target=detection_thread,
        args=(state, recognizer, cascade, hog, id_to_name),
        daemon=True)
    det_t.start()

    # ── Variables tourelle ────────────────────────────────────────────────────
    scan_angle  = 0.0
    scan_dir    = +1
    last_scan_t = time.time() - SCAN_PERIOD_S
    last_rot_t  = time.time() - ROT_PERIOD_S
    err_history = []   # historique des erreurs pour lissage

    print(f"Démarrage RECHERCHE | seuil={THRESHOLD} | cam={CAM_INDEX} | cx={cx}")

    try:
        while not stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                break

            # ── Rotation 90° ──────────────────────────────────────────────────────
            # Changez en ROTATE_90_COUNTERCLOCKWISE si l'image est à l'envers
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE )

            # Mettre à jour la frame partagée
            with state.lock:
                state.frame = frame.copy()

            # Lire l'état partagé
            with state.lock:
                phase      = state.phase
                recognized = state.recognized
                body_box   = state.body_box
                det_name   = state.det_name
                det_dist   = state.det_dist

            now = time.time()

            # ══════════════════════════════════════════════════════════════════════
            # PHASE RECHERCHE
            # ══════════════════════════════════════════════════════════════════════
            if phase == "SEARCH":
                tracker      = None
                kalman_init  = False
                tracker_lost = 0

                cv2.putText(frame, f"TH={THRESHOLD:.0f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                label = f"{det_name} dist={det_dist:.1f}" if det_name else "Aucun visage"
                cv2.putText(frame, label, (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                # ── Visage reconnu → passage en TRACK ───────────────────────────
                if recognized:
                    # Initialiser le tracker sur le corps (ou visage élargi)
                    with state.lock:
                        bb = state.body_box
                        state.recognized = False   # consommer le flag

                    if bb is not None:
                        tracker = cv2.TrackerCSRT_create()
                        tracker.init(frame, bb)
                        previous_good_box = bb
                        print(f"Tracker initialisé sur corps {bb}")
                    else:
                        print("Pas de box corps disponible — attente frame suivante")

                    with state.lock:
                        state.phase = "TRACK"

                # ── Balayage ─────────────────────────────────────────────────────
                elif not recognized and now - last_scan_t >= SCAN_PERIOD_S:
                    last_scan_t = now
                    next_a = scan_angle + scan_dir * SCAN_STEP_DEG
                    if abs(next_a) >= SCAN_LIMIT_DEG:
                        scan_dir *= -1
                        next_a = scan_angle + scan_dir * SCAN_STEP_DEG
                    delta      = next_a - scan_angle
                    scan_angle = next_a
                    send_rot(ser, delta)

                cv2.putText(frame, f"Scan {scan_angle:+.0f}deg", (10, 85),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180,180,180), 1)

            # ══════════════════════════════════════════════════════════════════════
            # PHASE RELOCK — perdu pendant le suivi → re-identification LBPH
            # ══════════════════════════════════════════════════════════════════════
            elif phase == "RELOCK":
                tracker      = None
                kalman_init  = False
                tracker_lost = 0

                # Fond rouge clignotant pour signaler la perte
                blink = int(now * 3) % 2 == 0
                color_alert = (0, 0, 220) if blink else (0, 0, 100)
                cv2.rectangle(frame, (0, 0), (FRAME_W, FRAME_H), color_alert, 4)

                cv2.putText(frame, "!! CIBLE PERDUE !!", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 255), 3)
                cv2.putText(frame, "Refais face a la camera", (10, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 100, 255), 2)
                label = f"{det_name} dist={det_dist:.1f}" if det_name else "En attente..."
                cv2.putText(frame, label, (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 200, 0), 2)

                if recognized:
                    with state.lock:
                        bb = state.body_box
                        state.recognized = False

                    if bb is not None:
                        tracker = cv2.TrackerCSRT_create()
                        tracker.init(frame, bb)
                        previous_good_box = bb
                        print(f"[RELOCK] Visage retrouvé — tracker recalé sur {bb}")
                    else:
                        print("[RELOCK] Visage reconnu mais pas de box corps — attente")

                    with state.lock:
                        state.phase = "TRACK"

            # ══════════════════════════════════════════════════════════════════════
            # PHASE SUIVI
            # ══════════════════════════════════════════════════════════════════════
            elif phase == "TRACK":
                # Recalage seulement si la boîte ressemble à une personne
                # ET que c'est bien toi (vérification LBPH dans la boîte)
                if body_box is not None and (tracker is None or tracker_lost > 8):
                    if is_person_like_box(body_box, frame.shape):
                        bx, by, bw, bh = [int(v) for v in body_box]
                        roi = frame[by:by+bh, bx:bx+bw]
                        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                        faces_in_box = cascade.detectMultiScale(
                            gray_roi, scaleFactor=1.1, minNeighbors=4,
                            minSize=(40, 40))
                        good_person = False
                        for (fx, fy, fw, fh) in faces_in_box:
                            face_img = preprocess_face(roi[fy:fy+fh, fx:fx+fw])
                            lid, dist = recognizer.predict(face_img)
                            if dist <= TRACK_FACE_THRESHOLD:
                                good_person = True
                                break
                        if good_person or tracker is None or tracker_lost > 20:
                            tracker = cv2.TrackerCSRT_create()
                            tracker.init(frame, body_box)
                            previous_good_box = body_box
                            kalman_init  = False
                            tracker_lost = 0
                            print(f"Tracker recalé sur personne {body_box}")
                    else:
                        print(f"Boîte refusée absurde: {body_box}")

                    with state.lock:
                        state.body_box = None

                tracked_ok = False

                if tracker is not None:
                    ok_t, bbox = tracker.update(frame)

                    if ok_t:
                        tx, ty, tw, th = [int(v) for v in bbox]

                        # Si la boîte devient trop mur/objet, on la refuse.
                        # Très important : on ne calcule tcx/tcy QUE si la boîte est valide.
                        if not is_person_like_box((tx, ty, tw, th), frame.shape):
                            tracker_lost += 1
                            tracked_ok = False
                        else:
                            tracked_ok   = True
                            tracker_lost = 0
                            previous_good_box = (tx, ty, tw, th)

                            tcx = tx + tw // 2
                            tcy = ty + th // 2

                            # Kalman
                            meas = np.array([[np.float32(tcx)],
                                             [np.float32(tcy)]])
                            if not kalman_init:
                                kf.statePre  = np.array([tcx, tcy, 0, 0],
                                                         dtype=np.float32).reshape(4, 1)
                                kf.statePost = kf.statePre.copy()
                                kf.errorCovPre  = np.eye(4, dtype=np.float32)
                                kf.errorCovPost = np.eye(4, dtype=np.float32)
                                kalman_init = True

                            kf.predict()
                            est      = kf.correct(meas)
                            px, py   = kalman_get_xy(est)

                            # Dessin
                            cv2.rectangle(frame, (tx, ty), (tx+tw, ty+th), (0, 220, 0), 2)
                            cv2.circle(frame, (px, py), 6, (0, 0, 255), -1)
                            cv2.line(frame, (cx, 0), (cx, FRAME_H), (80, 80, 80), 1)

                            # Commande rotation
                            err = px - cx
                            # Lissage : moyenne des 4 dernières erreurs
                            err_history.append(err)
                            if len(err_history) > 4:
                                err_history.pop(0)
                            smooth_err = int(sum(err_history) / len(err_history))

                            if abs(smooth_err) > DEAD_ZONE_PX and now - last_rot_t >= ROT_PERIOD_S:
                                last_rot_t = now
                                send_rot(ser, smooth_err * DEG_PER_PX)

                            cv2.putText(frame, f"SUIVI  err={err:+d}px", (10, 25),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(frame, "LOCK: OUI", (10, 55),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    else:
                        tracker_lost += 1

                if not tracked_ok:
                    # Prédiction Kalman pure
                    if kalman_init:
                        pred   = kf.predict()
                        px, py = kalman_get_xy(pred)   # ← corrigé
                        err    = px - cx
                        err_history.append(err)
                        if len(err_history) > 4:
                            err_history.pop(0)
                        smooth_err = int(sum(err_history) / len(err_history))
                        if abs(smooth_err) > DEAD_ZONE_PX and now - last_rot_t >= ROT_PERIOD_S:
                            last_rot_t = now
                            send_rot(ser, smooth_err * DEG_PER_PX)
                        cv2.putText(frame, "PREDICTION Kalman", (10, 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                    cv2.putText(frame, f"LOCK: NON ({tracker_lost})", (10, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                    if tracker_lost > TRACKER_LOST_LIMIT:
                        print("Tracker perdu → buzzer + RELOCK facial LBPH")
                        send_buzzer(ser, 800)      # bip d'alerte
                        tracker     = None
                        kalman_init = False
                        err_history.clear()
                        with state.lock:
                            state.phase      = "RELOCK"
                            state.recognized = False
                            state.body_box   = None

            # ── Affichage commun ──────────────────────────────────────────────────
            with state.lock:
                ph = state.phase
            cv2.putText(frame, ph, (FRAME_W - 170, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.imshow("LBPH Tourelle", frame)

            if cv2.waitKey(1) & 0xFF == 27:
                stop_event.set()
                break

    except KeyboardInterrupt:
        print("\nCtrl+C reçu — arrêt demandé.")

    finally:
        state.stop = True
        cap.release()
        cv2.destroyAllWindows()

        if ser:
            ser.close()
        print("Arrêt propre.")


if __name__ == "__main__":
    main()
