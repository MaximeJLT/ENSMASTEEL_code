import numpy as np
import cv2


class TableMapper:
    def __init__(self):
        self.H = None         
        self.H_inv = None      
        self.last_ok = False

        self.table_ids = {20, 21, 22, 23}

        self.world_corners_mm = {
            "A": (-1500, 1000),
            "B": ( 1500, 1000),
            "C": ( 1500, -1000),
            "D": ( -1500, -1000),
        }
     
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
        if table_markers is None:
            self.last_ok = False
            return False

        required = ["TL", "TR", "BR", "BL"]
        if not all(k in table_markers and table_markers[k] is not None for k in required):
            self.last_ok = False
            return False

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

        min_dist2 = 1e18
        for i in range(4):
            for j in range(i + 1, 4):
                d = pix_pts[i] - pix_pts[j]
                dist2 = float(d[0]*d[0] + d[1]*d[1])
                min_dist2 = min(min_dist2, dist2)
        if min_dist2 < 25.0:  # < 5 px
            self.last_ok = False
            return False

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

        dst_pts = np.array([
            list(self.world_corners_mm["A"]),  # TL
            list(self.world_corners_mm["B"]),  # TR
            list(self.world_corners_mm["C"]),  # BR
            list(self.world_corners_mm["D"]),  # BL
        ], dtype=np.float32)

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
