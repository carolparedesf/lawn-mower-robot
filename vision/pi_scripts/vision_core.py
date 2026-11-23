import cv2
import numpy as np
from collections import deque
from potential_field import dist_from_y, bearing_from_x, compute_command

COLORS = {
    "arbol": (0, 128, 0), "barrera": (0, 0, 255), "caminero": (200, 200, 0),
    "otro": (255, 0, 255), "pasto": (0, 255, 0), "planta": (0, 200, 255),
    "tierra": (30, 80, 150),
}
OBSTACLE_CLASSES = ("arbol", "planta", "barrera", "otro")
ALIAS = {"potted plant": "planta", "chair": "barrera", "person": "otro", "couch": "barrera"}

def base_point(pts):
    ys = pts[:, 1]
    ymax = ys.max()
    xs = pts[ys >= ymax - 2][:, 0]
    return int(xs.mean()), int(ymax)

def centroid(pts):
    m = cv2.moments(pts.reshape(-1, 1, 2))
    if m["m00"] == 0:
        return None
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])

class Pipeline:
    def __init__(self, model, conf=0.4, imgsz=640, min_grass=0.10):
        self.model = model
        self.conf = conf
        self.imgsz = imgsz
        self.min_grass = min_grass
        self.hist = deque(maxlen=7)

    def process(self, frame):
        h, w = frame.shape[:2]
        r = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        overlay = frame.copy()
        marks = []
        obstacles = []
        grass_area = 0.0
        if r.masks is not None:
            for i, pts in enumerate(r.masks.xy):
                if len(pts) < 3:
                    continue
                pts = pts.astype(np.int32)
                raw = self.model.names[int(r.boxes.cls[i])]; name = ALIAS.get(raw, raw)
                cv2.fillPoly(overlay, [pts], COLORS.get(name, (255, 255, 255)))
                if name == "pasto":
                    grass_area += cv2.contourArea(pts.reshape(-1, 1, 2))
                if name in OBSTACLE_CLASSES:
                    bx, by = base_point(pts)
                    rho = dist_from_y(by)
                    obstacles.append((bearing_from_x(bx, w), rho, name))
                    cen = centroid(pts) if name == "planta" else None
                    marks.append((name, bx, by, rho, cen))
        vis = cv2.addWeighted(overlay, 0.4, frame, 0.6, 0)
        for name, bx, by, rho, cen in marks:
            cv2.circle(vis, (bx, by), 8, (0, 0, 255), -1)
            cv2.putText(vis, name, (bx + 10, by - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if cen:
                cv2.circle(vis, cen, 8, (255, 0, 0), -1)
        v, wz = compute_command(obstacles)
        self.hist.append(grass_area / (w * h) >= self.min_grass)
        cut_on = sum(self.hist) > len(self.hist) / 2
        cv2.putText(vis, f"v={v:.2f} m/s  w={wz:+.2f} rad/s", (20, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        info = {"cut_on": cut_on, "v": v, "w": wz, "grass": grass_area / (w * h),
                "n_obst": len(obstacles)}
        return vis, info
