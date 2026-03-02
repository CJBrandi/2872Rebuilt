"""
Limelight SnapScript for field-relative fuel clustering.

Rio input (llrobot):
  [robot_x_m, robot_y_m, robot_yaw_rad, rio_timestamp_s]

Script output (llpython):
  [version, cluster_count, capture_timestamp_s,
   cluster_id_0, x_m_0, y_m_0, count_0, score_0, best_flag_0,
   cluster_id_1, x_m_1, y_m_1, count_1, score_1, best_flag_1, ...]

The Rio side expects version=1 and a stride of 6 values per cluster entry.
"""

import math
import time
import cv2
import numpy as np

try:
    import ntcore  # type: ignore
except Exception:
    ntcore = None


PACKET_VERSION = 1.0
PACKET_HEADER_SIZE = 3
PACKET_CLUSTER_STRIDE = 6
MAX_PACKET_CLUSTERS = 8

FRAME_WIDTH = 640.0
FRAME_HEIGHT = 480.0
H_FOV_DEG = 63.3
V_FOV_DEG = 49.7
FX = (FRAME_WIDTH * 0.5) / math.tan(math.radians(H_FOV_DEG) * 0.5)
FY = (FRAME_HEIGHT * 0.5) / math.tan(math.radians(V_FOV_DEG) * 0.5)
CX = FRAME_WIDTH * 0.5
CY = FRAME_HEIGHT * 0.5

FIELD_LENGTH_M = 16.51
FIELD_WIDTH_M = 8.04

# Match VisionConstants.robotToDetectionCamera.
CAMERA_X_M = -25.382 * 0.0254
CAMERA_Y_M = 9.906 * 0.0254
CAMERA_Z_M = 10.004 * 0.0254
CAMERA_ROLL_R = 0.0
CAMERA_PITCH_R = math.radians(10.0)
CAMERA_YAW_R = math.pi

DBSCAN_EPSILON_M = 0.80
DBSCAN_MIN_POINTS = 2
CLUSTER_ASSOCIATION_DISTANCE_M = 1.2
CLUSTER_MAX_MISSED_FRAMES = 8
CLUSTER_CENTROID_EMA_ALPHA = 0.45
COUNT_WEIGHT = 2.5
DISTANCE_WEIGHT = 1.0

MIN_CONTOUR_AREA_PX = 40.0
ORANGE_LOWER_1 = np.array([4, 90, 70], dtype=np.uint8)
ORANGE_UPPER_1 = np.array([22, 255, 255], dtype=np.uint8)
ORANGE_LOWER_2 = np.array([0, 90, 70], dtype=np.uint8)
ORANGE_UPPER_2 = np.array([4, 255, 255], dtype=np.uint8)
RAW_DETECTION_STRIDE = 12
DEFAULT_LIMELIGHT_TABLE = "limelight"

UNVISITED = -2
NOISE = -1

STATE = {
    "stable_clusters": {},  # stable_id -> {"centroid": np.array([x, y]), "missed": int}
    "next_stable_id": 0,
    "nt_entry": None,
}


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _rot_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _rot_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _camera_pose(robot_x, robot_y, robot_yaw):
    r_field_from_robot = _rot_z(robot_yaw)
    r_robot_from_camera = _rot_z(CAMERA_YAW_R) @ _rot_y(CAMERA_PITCH_R) @ _rot_x(CAMERA_ROLL_R)
    r_field_from_camera = r_field_from_robot @ r_robot_from_camera
    camera_offset_robot = np.array([CAMERA_X_M, CAMERA_Y_M, CAMERA_Z_M], dtype=np.float64)
    camera_translation = np.array([robot_x, robot_y, 0.0], dtype=np.float64) + (
        r_field_from_robot @ camera_offset_robot
    )
    return camera_translation, r_field_from_camera


def _back_project_to_field(x_px, y_px, camera_translation, r_field_from_camera):
    if not (0.0 <= x_px < FRAME_WIDTH and 0.0 <= y_px < FRAME_HEIGHT):
        return None

    ray_camera = np.array([1.0, (CX - x_px) / FX, (CY - y_px) / FY], dtype=np.float64)
    ray_field = r_field_from_camera @ ray_camera
    if abs(ray_field[2]) < 1e-6:
        return None

    t = (0.0 - camera_translation[2]) / ray_field[2]
    if t <= 0.0:
        return None

    point = camera_translation + ray_field * t
    x_m = float(point[0])
    y_m = float(point[1])
    if x_m < 0.0 or x_m > FIELD_LENGTH_M or y_m < 0.0 or y_m > FIELD_WIDTH_M:
        return None
    return x_m, y_m


def _segment_fuel(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, ORANGE_LOWER_1, ORANGE_UPPER_1)
    mask |= cv2.inRange(hsv, ORANGE_LOWER_2, ORANGE_UPPER_2)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    largest_contour = None
    largest_area = -1.0
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < MIN_CONTOUR_AREA_PX:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        detections.append(
            {
                "x_px": float(x + 0.5 * w),
                "y_px": float(y + h),
                "radius_px": float(max(w, h) * 0.5),
                "area": area,
            }
        )
        if area > largest_area:
            largest_area = area
            largest_contour = contour
    return detections, largest_contour, mask


def _get_raw_detections_entry():
    if ntcore is None:
        return None
    cached = STATE.get("nt_entry")
    if cached is not None:
        return cached
    try:
        table = ntcore.NetworkTableInstance.getDefault().getTable(DEFAULT_LIMELIGHT_TABLE)
        entry = table.getEntry("rawdetections")
        STATE["nt_entry"] = entry
        return entry
    except Exception:
        return None


def _decode_raw_detections(raw_values):
    detections = []
    if raw_values is None:
        return detections
    usable = (len(raw_values) // RAW_DETECTION_STRIDE) * RAW_DETECTION_STRIDE
    for base in range(0, usable, RAW_DETECTION_STRIDE):
        try:
            area = float(raw_values[base + 3])
            corners = raw_values[base + 4 : base + 12]
            c0x, c0y, c1x, c1y, c2x, c2y, c3x, c3y = [float(v) for v in corners]
        except Exception:
            continue

        min_x = min(c0x, c1x, c2x, c3x)
        max_x = max(c0x, c1x, c2x, c3x)
        min_y = min(c0y, c1y, c2y, c3y)
        max_y = max(c0y, c1y, c2y, c3y)
        width = max(2.0, max_x - min_x)
        height = max(2.0, max_y - min_y)
        center_x = 0.5 * (min_x + max_x)
        bottom_y = max_y
        if center_x < 0.0 or center_x >= FRAME_WIDTH or bottom_y < 0.0 or bottom_y >= FRAME_HEIGHT:
            continue
        detections.append(
            {
                "x_px": center_x,
                "y_px": bottom_y,
                "radius_px": max(width, height) * 0.5,
                "area": max(0.0, area),
                "bbox": (min_x, min_y, max_x, max_y),
            }
        )
    return detections


def _neural_detections_from_nt():
    entry = _get_raw_detections_entry()
    if entry is None:
        return []
    try:
        raw_values = entry.getDoubleArray([])
    except Exception:
        return []
    return _decode_raw_detections(raw_values)


def _distance_xy(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)


def _dbscan(points, epsilon, min_points):
    labels = [UNVISITED] * len(points)
    cluster_id = 0

    def neighbors(index):
        p = points[index]
        result = []
        for j, q in enumerate(points):
            if _distance_xy(p, q) <= epsilon:
                result.append(j)
        return result

    for i in range(len(points)):
        if labels[i] != UNVISITED:
            continue
        n = neighbors(i)
        if len(n) < min_points:
            labels[i] = NOISE
            continue
        labels[i] = cluster_id
        queue = list(n)
        while queue:
            idx = queue.pop(0)
            if labels[idx] == NOISE:
                labels[idx] = cluster_id
            if labels[idx] != UNVISITED:
                continue
            labels[idx] = cluster_id
            n2 = neighbors(idx)
            if len(n2) >= min_points:
                queue.extend(n2)
        cluster_id += 1
    return labels, cluster_id


def _average(points):
    if not points:
        return (0.0, 0.0)
    x = sum(p[0] for p in points) / len(points)
    y = sum(p[1] for p in points) / len(points)
    return (x, y)


def _associate_stable_ids(raw_centroids):
    stable_clusters = STATE["stable_clusters"]
    candidates = []
    for raw_id, raw_centroid in raw_centroids.items():
        for stable_id, state in stable_clusters.items():
            distance = _distance_xy(raw_centroid, state["centroid"])
            if distance <= CLUSTER_ASSOCIATION_DISTANCE_M:
                candidates.append((distance, raw_id, stable_id))
    candidates.sort(key=lambda c: c[0])

    raw_to_stable = {}
    used_raw = set()
    used_stable = set()
    for _, raw_id, stable_id in candidates:
        if raw_id in used_raw or stable_id in used_stable:
            continue
        used_raw.add(raw_id)
        used_stable.add(stable_id)
        raw_to_stable[raw_id] = stable_id

    touched = set()
    for raw_id, raw_centroid in raw_centroids.items():
        if raw_id not in raw_to_stable:
            stable_id = STATE["next_stable_id"]
            STATE["next_stable_id"] += 1
            stable_clusters[stable_id] = {"centroid": np.array(raw_centroid), "missed": 0}
            raw_to_stable[raw_id] = stable_id
        stable_id = raw_to_stable[raw_id]
        state = stable_clusters[stable_id]
        current = state["centroid"]
        target = np.array(raw_centroid)
        alpha = _clamp(CLUSTER_CENTROID_EMA_ALPHA, 0.0, 1.0)
        state["centroid"] = (1.0 - alpha) * current + alpha * target
        state["missed"] = 0
        touched.add(stable_id)

    to_remove = []
    for stable_id, state in stable_clusters.items():
        if stable_id in touched:
            continue
        state["missed"] += 1
        if state["missed"] > CLUSTER_MAX_MISSED_FRAMES:
            to_remove.append(stable_id)
    for stable_id in to_remove:
        stable_clusters.pop(stable_id, None)

    return raw_to_stable


def _build_llpython(cluster_rows):
    payload = [PACKET_VERSION, float(len(cluster_rows)), time.time()]
    for row in cluster_rows[:MAX_PACKET_CLUSTERS]:
        payload.extend(row)
    return payload


def runPipeline(image, llrobot):
    # Parse Rio pose packet.
    robot_x = float(llrobot[0]) if len(llrobot) > 0 else 0.0
    robot_y = float(llrobot[1]) if len(llrobot) > 1 else 0.0
    robot_yaw = float(llrobot[2]) if len(llrobot) > 2 else 0.0
    camera_translation, r_field_from_camera = _camera_pose(robot_x, robot_y, robot_yaw)

    detections = _neural_detections_from_nt()
    largest_contour = None
    mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)

    # Fallback path: keep classical segmentation available if neural data is unavailable.
    if len(detections) == 0:
        detections, largest_contour, mask = _segment_fuel(image)

    field_points = []
    for detection in detections:
        projected = _back_project_to_field(
            detection["x_px"], detection["y_px"], camera_translation, r_field_from_camera
        )
        if projected is None:
            continue
        field_points.append(
            {
                "x_m": projected[0],
                "y_m": projected[1],
                "x_px": detection["x_px"],
                "y_px": detection["y_px"],
                "radius_px": detection["radius_px"],
            }
        )

    xy_points = [(p["x_m"], p["y_m"]) for p in field_points]
    labels, _ = _dbscan(xy_points, DBSCAN_EPSILON_M, DBSCAN_MIN_POINTS)

    raw_cluster_points = {}
    for i, label in enumerate(labels):
        if label < 0:
            continue
        raw_cluster_points.setdefault(label, []).append((field_points[i]["x_m"], field_points[i]["y_m"]))

    raw_centroids = {raw_id: _average(points) for raw_id, points in raw_cluster_points.items()}
    raw_to_stable = _associate_stable_ids(raw_centroids)

    stable_cluster_points = {}
    for raw_id, points in raw_cluster_points.items():
        stable_id = raw_to_stable.get(raw_id)
        if stable_id is None:
            continue
        stable_cluster_points.setdefault(stable_id, []).extend(points)

    scored_clusters = []
    for stable_id, points in stable_cluster_points.items():
        centroid = _average(points)
        distance = _distance_xy((robot_x, robot_y), centroid)
        count = float(len(points))
        score = COUNT_WEIGHT * count - DISTANCE_WEIGHT * distance
        scored_clusters.append(
            {
                "stable_id": stable_id,
                "x_m": centroid[0],
                "y_m": centroid[1],
                "count": count,
                "score": score,
            }
        )

    scored_clusters.sort(key=lambda c: c["score"], reverse=True)

    cluster_rows = []
    for i, cluster in enumerate(scored_clusters[:MAX_PACKET_CLUSTERS]):
        cluster_rows.append(
            [
                float(cluster["stable_id"]),
                float(cluster["x_m"]),
                float(cluster["y_m"]),
                float(cluster["count"]),
                float(cluster["score"]),
                1.0 if i == 0 else 0.0,
            ]
        )

    # Debug visualization
    output = image.copy()
    for detection in detections:
        min_x, min_y, max_x, max_y = detection.get("bbox", (0.0, 0.0, 0.0, 0.0))
        if max_x > min_x and max_y > min_y:
            cv2.rectangle(
                output,
                (int(min_x), int(min_y)),
                (int(max_x), int(max_y)),
                (120, 120, 120),
                1,
            )
        cv2.circle(
            output,
            (int(detection["x_px"]), int(detection["y_px"])),
            int(max(2.0, detection["radius_px"])),
            (140, 140, 140),
            1,
        )
    for p in field_points:
        cv2.circle(
            output,
            (int(p["x_px"]), int(p["y_px"])),
            int(max(2.0, p["radius_px"])),
            (70, 220, 255),
            1,
        )
    cv2.putText(
        output,
        "clusters=%d points=%d" % (len(scored_clusters), len(field_points)),
        (12, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    if len(scored_clusters) > 0:
        best = scored_clusters[0]
        cv2.putText(
            output,
            "best id=%d score=%.2f @ (%.2f, %.2f)m"
            % (best["stable_id"], best["score"], best["x_m"], best["y_m"]),
            (12, 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 220, 255),
            1,
        )
    cv2.putText(
        output,
        "mask px=%d" % int(np.count_nonzero(mask)),
        (12, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (220, 220, 220),
        1,
    )

    llpython = _build_llpython(cluster_rows)
    if largest_contour is None:
        if len(detections) > 0:
            best = detections[0]
            min_x, min_y, max_x, max_y = best.get("bbox", (best["x_px"], best["y_px"], best["x_px"], best["y_px"]))
            largest_contour = np.array(
                [
                    [[int(min_x), int(min_y)]],
                    [[int(max_x), int(min_y)]],
                    [[int(max_x), int(max_y)]],
                    [[int(min_x), int(max_y)]],
                ],
                dtype=np.int32,
            )
        else:
            largest_contour = np.array([], dtype=np.int32)
    return largest_contour, output, llpython
