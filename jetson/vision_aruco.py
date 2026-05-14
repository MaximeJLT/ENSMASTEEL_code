import cv2
import numpy as np
import time

USE_CAMERA = True
CAM_INDEX = 0
CAP_WIDTH = 1280
CAP_HEIGHT = 720

_cap = None

TABLE_CORNER_IDS = {
    "TL": 21,
    "TR": 23,
    "BR": 22,
    "BL": 20,
}

# Couleur d'équipe : "blue" ou "yellow"
# À CHANGER LE MATIN DU MATCH selon le tirage au sort
with open("scenario.json", "r") as f:
    TEAM_COLOR = json.load(f)["team"]

# IDs ArUco selon le règlement Eurobot 2026
CRATE_BLUE_ID  = 36
CRATE_YELLOW_ID = 47
CRATE_EMPTY_ID  = 41


def _init_camera():
    global _cap
    if _cap is not None:
        return True
    
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return False
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)

    _cap = cap
    print("Camera initialized.")
    return True


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# ---- PARAMÈTRES DE DÉTECTION OPTIMISÉS pour détecter MAX d'ArUcos ----
params = cv2.aruco.DetectorParameters()

# Améliorer la détection des marqueurs petits et éloignés
params.adaptiveThreshWinSizeMin = 3
params.adaptiveThreshWinSizeMax = 23
params.adaptiveThreshWinSizeStep = 4
params.adaptiveThreshConstant = 7

# Taille minimale du marqueur (en proportion de l'image) — réduit pour voir les petits
params.minMarkerPerimeterRate = 0.01   # défaut 0.03, on baisse pour voir les petits ArUcos
params.maxMarkerPerimeterRate = 4.0

# Précision des contours
params.polygonalApproxAccuracyRate = 0.05
params.minCornerDistanceRate = 0.05
params.minDistanceToBorder = 3
params.minMarkerDistanceRate = 0.05

# Affinement des coins (sub-pixel, plus précis)
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
params.cornerRefinementWinSize = 5
params.cornerRefinementMaxIterations = 30
params.cornerRefinementMinAccuracy = 0.1

# Marges de tolérance sur le contenu du marqueur
params.markerBorderBits = 1
params.perspectiveRemovePixelPerCell = 4
params.perspectiveRemoveIgnoredMarginPerCell = 0.13

# Tolérance sur les erreurs de décodage (plus permissif)
params.maxErroneousBitsInBorderRate = 0.35
params.errorCorrectionRate = 0.6

detector = cv2.aruco.ArucoDetector(aruco_dict, params)


def _id_to_team(detected_id):
    """Retourne 'us', 'enemy', 'empty', ou None selon l'ID et notre couleur d'équipe."""
    if detected_id == CRATE_EMPTY_ID:
        return "empty"
    if TEAM_COLOR == "blue":
        if detected_id == CRATE_BLUE_ID:
            return "us"
        if detected_id == CRATE_YELLOW_ID:
            return "enemy"
    elif TEAM_COLOR == "yellow":
        if detected_id == CRATE_YELLOW_ID:
            return "us"
        if detected_id == CRATE_BLUE_ID:
            return "enemy"
    return None


def get_objects():
    """
    Retourne (objects, table_markers).
    objects = liste de tous les ArUcos détectés.
    Chaque objet a un champ "team" : "us", "enemy", "empty", "unknown".
    """

    if USE_CAMERA and _init_camera():
        ret, frame = _cap.read()
        if not ret or frame is None:
            print("[vision] Camera read failed -> fallback synthetic")
            frame = None
    else:
        frame = None

    if frame is None:
        # Fallback de test
        marker_id = 7
        marker_size_px = 400
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
        scene = np.full((600, 800), 255, dtype=np.uint8)
        y0, x0 = 100, 200
        scene[y0:y0+marker_size_px, x0:x0+marker_size_px] = marker_img
        input_img = scene
        debug_img = cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR)
    else:
        input_img = frame
        debug_img = frame.copy()

    # Détection (avec les paramètres tunés)
    corners, ids, rejected = detector.detectMarkers(input_img)

    objects = []
    if ids is not None:
        for i in range(len(ids)):
            pts = corners[i][0]
            u_center = float(pts[:, 0].mean())
            v_center = float(pts[:, 1].mean())
            detected_id = int(ids[i][0])

            team = _id_to_team(detected_id) or "unknown"

            objects.append({
                "id": detected_id,
                "u_px": u_center,
                "v_px": v_center,
                "team": team,
                "score_team": None
            })

    # Détection des 4 coins table
    table_markers = None
    id_to_corner = {v: k for k, v in TABLE_CORNER_IDS.items() if v is not None}
    table_ids = set(id_to_corner.keys())

    if len(table_ids) == 4:
        found = {}
        for obj in objects:
            corner = id_to_corner.get(obj["id"])
            if corner is not None:
                found[corner] = {
                    "id": obj["id"],
                    "u_px": obj["u_px"],
                    "v_px": obj["v_px"],
                }
        if len(found) == 4:
            table_markers = found
            objects = [o for o in objects if o["id"] not in table_ids]
    else:
        table_markers = None

    # Debug overlay
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)

    # Affiche aussi le nombre détecté
    cv2.putText(debug_img, f"Detected: {len(objects)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(debug_img, f"Team: {TEAM_COLOR}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    cv2.imshow("vision", debug_img)
    cv2.waitKey(1)

    return objects, table_markers


def close():
    global _cap
    if _cap is not None:
        _cap.release()
        _cap = None
    cv2.destroyAllWindows()
