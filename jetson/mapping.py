# mapping.py
# Objectif: construire une homographie H pour convertir pixels -> mm
# Hypothese: table plane + 4 ArUco fixes visibles (reperes table)
# Version v0: pas de fisheye; on fera la correction plus tard si besoin

import numpy as np
import cv2


class TableMapper:
    def __init__(self):
        self.H = None          # homographie pixel->world
        self.H_inv = None      # optionnel world->pixel
        self.last_ok = False

        # TODO 1: IDs des 4 tags fixes de table
        self.table_ids = {20, 21, 22, 23}

        # Convention repere (origine centre table):
        # 4 aruco table en mm :
        self.world_corners_mm = {
            "A": (-1500, 1000),
            "B": ( 1500, 1000),
            "C": ( 1500, -1000),
            "D": ( -1500, -1000),
        }
        # self.world_corners_mm = {
        #     "A": (-300,  200),  # TL
        #     "B": ( 300,  200),  # TR
        #     "C": ( 300, -200),  # BR
        #     "D": (-300, -200),  # BL
        # }

        # TODO 2: associer chaque ID table a un coin (BL/BR/TR/TL)
        # Exemple: 0->BL, 1->BR, 2->TR, 3->TL
        self.id_to_corner = {
            0: "D",  # BL
            1: "C",  # BR
            2: "B",  # TR
            3: "A",  # TL
        }

    def update(self, table_markers):
        """
        Recalcule l'homographie si on a 4 coins table valides.
        Renvoie True si mapping OK, sinon False (sans jamais crasher).
        """
        # table_markers attendu: dict avec TL/TR/BR/BL (chacun: {"id", "u_px", "v_px"} ou similaire)
        if table_markers is None:
            self.last_ok = False
            return False

        required = ["TL", "TR", "BR", "BL"]
        if not all(k in table_markers and table_markers[k] is not None for k in required):
            self.last_ok = False
            return False

        # --- récupérer les 4 points pixels ---
        try:
            pix_pts = np.array([
                [float(table_markers["TL"]["u_px"]), float(table_markers["TL"]["v_px"])],
                [float(table_markers["TR"]["u_px"]), float(table_markers["TR"]["v_px"])],
                [float(table_markers["BR"]["u_px"]), float(table_markers["BR"]["v_px"])],
                [float(table_markers["BL"]["u_px"]), float(table_markers["BL"]["v_px"])],
            ], dtype=np.float32)
        except Exception:
            self.last_ok = False
            return False

        # --- checks anti-dégénérescence ---
        # 1) points trop proches (doublons / quasi-doublons)
        min_dist2 = 1e18
        for i in range(4):
            for j in range(i + 1, 4):
                d = pix_pts[i] - pix_pts[j]
                dist2 = float(d[0]*d[0] + d[1]*d[1])
                min_dist2 = min(min_dist2, dist2)
        if min_dist2 < 25.0:  # < 5 px
            self.last_ok = False
            return False

        # 2) aire du quadrilatère trop petite (quasi-aligné)
        def poly_area(pts):
            # shoelace
            x = pts[:, 0]
            y = pts[:, 1]
            return 0.5 * abs(
                x[0]*y[1] + x[1]*y[2] + x[2]*y[3] + x[3]*y[0]
                - (y[0]*x[1] + y[1]*x[2] + y[2]*x[3] + y[3]*x[0])
            )

        area = poly_area(pix_pts)
        if area < 2000.0:  # ajuste si besoin
            self.last_ok = False
            return False

        # --- destination en mm (déjà définie dans ton init) ---
        # world_corners_mm est un dict {"TL": (x,y), ...} → on extrait dans l'ordre TL/TR/BR/BL
        dst_pts = np.array([
            list(self.world_corners_mm["A"]),  # TL
            list(self.world_corners_mm["B"]),  # TR
            list(self.world_corners_mm["C"]),  # BR
            list(self.world_corners_mm["D"]),  # BL
        ], dtype=np.float32)

        # --- calcul H ---
        try:
            H = cv2.getPerspectiveTransform(pix_pts, dst_pts)
            # Si det ~ 0 => inv impossible
            det = float(np.linalg.det(H))
            if abs(det) < 1e-12:
                self.last_ok = False
                return False

            H_inv = np.linalg.inv(H)
        except Exception:
            self.last_ok = False
            return False

        self.H = H
        self.H_inv = H_inv
        self.last_ok = True
        return True

    def pixel_to_world(self, u_px: float, v_px: float):
        """
        Convertit un point pixel en (x_mm, y_mm).
        Necessite que update() ait reussi au moins une fois.
        """
        if self.H is None:
            return None

        pt = np.array([[[float(u_px), float(v_px)]]], dtype=np.float32)  # shape (1,1,2)
        out = cv2.perspectiveTransform(pt, self.H)  # shape (1,1,2)
        x_mm = float(out[0, 0, 0])
        y_mm = float(out[0, 0, 1])
        return x_mm, y_mm