import cv2
import numpy as np
import time

#--------------------Test d'un algorithme de detection sur dataset connu----------------------------

USE_CAMERA = True #on passera a TRUE quand on aura la camera
CAM_INDEX = 0
CAP_WIDTH = 1280
CAP_HEIGHT = 720 #parametres openCV camera

_cap = None

TABLE_CORNER_IDS = {
    "TL": 21,
    "TR": 23,
    "BR": 22,
    "BL": 20,
}

def _init_camera():

    """
    Initialise la camera si ce n'est pas deja fait.
    Retourne True si la camera est prete, False sinon.
    1 seule instance de camera est geree globalement.
    """

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
detector = cv2.aruco.ArucoDetector(aruco_dict) #algo de detection aruco

marker_id = 7
marker_size_px = 400

def get_objects():
    """
    Retourne une liste d'objets detectes sous la forme :
    [{"id": int, "u_px": float, "v_px": float, "team":"unknown", "score_team": None}, ...]
    """

    # 1) Choix de la source image
    if USE_CAMERA and _init_camera():
        ret, frame = _cap.read()
        if not ret or frame is None:
            print("[vision] Camera read failed -> fallback synthetic")
            frame = None
    else:
        frame = None

    # 2) Fallback: scene simulée (comme avant)
    if frame is None:
        marker_id = 7
        marker_size_px = 400
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)

        scene = np.full((600, 800), 255, dtype=np.uint8)
        y0, x0 = 100, 200
        scene[y0:y0+marker_size_px, x0:x0+marker_size_px] = marker_img

        # detecteur attend une image (gray ou bgr). Ici c'est gray
        input_img = scene
        debug_img = cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR)
        H, W = debug_img.shape[:2]
    else:
        # frame camera est BGR
        input_img = frame
        debug_img = frame.copy()
        H, W = debug_img.shape[:2]

    # 3) Detection ArUco
    corners, ids, rejected = detector.detectMarkers(input_img)

    objects = []
    if ids is not None:
        for i in range(len(ids)):
            pts = corners[i][0]  # (4,2)
            u_center = float(pts[:, 0].mean())
            v_center = float(pts[:, 1].mean())
            detected_id = int(ids[i][0])

            objects.append({
                "id": detected_id,
                "u_px": u_center,
                "v_px": v_center,
                "team": "unknown",
                "score_team": None
            })

    # ---- Détection des 4 ArUco de mapping (coins table) : PAR IDS ----
    table_markers = None

    # On construit id -> corner uniquement avec les IDs renseignés (non None)
    id_to_corner = {v: k for k, v in TABLE_CORNER_IDS.items() if v is not None}
    table_ids = set(id_to_corner.keys())

    if len(table_ids) == 4:
        # On récupère dans objects les 4 tags table
        found = {}
        for obj in objects:
            corner = id_to_corner.get(obj["id"])
            if corner is not None:
                found[corner] = {
                    "id": obj["id"],
                    "u_px": obj["u_px"],
                    "v_px": obj["v_px"],
                }

        # Valide seulement si on a les 4 coins
        if len(found) == 4:
            table_markers = found

            # On retire ces IDs de la liste des objets "mobiles"
            objects = [o for o in objects if o["id"] not in table_ids]
    else:
        # Pas configuré -> on ne tente pas de mapping
        table_markers = None

    # 4) Debug overlay + fenêtre (TOUJOURS)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)

    cv2.imshow("vision", debug_img)
    cv2.waitKey(1) 

    return objects, table_markers

def close():
    global _cap
    if _cap is not None:
        _cap.release()
        _cap = None
    cv2.destroyAllWindows()
