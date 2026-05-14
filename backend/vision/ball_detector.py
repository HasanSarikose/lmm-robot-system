import cv2
import numpy as np
import math


CAMERA_FOV_RAD = 1.8
WORLD_LIMIT_M = 7.0

# Hedefler artık top değil, yerdeki geometrik kırmızı hedefler.
# Bu değer sadece drone kamerasında beklenen hedef boyutunu filtrelemek için kullanılır.
TARGET_RADIUS_M = 0.12

# Hedef görüntü merkezine ne kadar yakınsa koordinat o kadar güvenilir.
# 0.15 = görüntü merkezinin yaklaşık %15 çevresi.
CENTER_TOLERANCE = 0.15


def _red_mask(frame, lower_sat=100, lower_val=50):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(
        hsv,
        np.array([0, lower_sat, lower_val]),
        np.array([15, 255, 255])
    )

    mask2 = cv2.inRange(
        hsv,
        np.array([165, lower_sat, lower_val]),
        np.array([180, 255, 255])
    )

    mask = mask1 | mask2

    open_kernel = np.ones((3, 3), np.uint8)
    close_kernel = np.ones((15, 15), np.uint8)
    dilate_kernel = np.ones((9, 9), np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, dilate_kernel)

    return mask


def _classify_shape(cnt):
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)

    if area <= 0 or perimeter <= 0:
        return "unknown", "red_unknown"

    approx = cv2.approxPolyDP(cnt, 0.04 * perimeter, True)
    vertices = len(approx)

    circularity = 4 * math.pi * area / (perimeter * perimeter)

    x, y, w, h = cv2.boundingRect(cnt)
    aspect_ratio = w / float(h) if h > 0 else 0

    if vertices == 3:
        return "triangle", "red_triangle"

    if vertices == 4 and 0.70 <= aspect_ratio <= 1.30:
        if 0.70 <= aspect_ratio <= 1.30:
            return "square", "red_square"
        return "square", "red_square"

    if circularity > 0.55:
        return "circle", "red_circle"

    return "unknown", "red_unknown"


def detect_from_drone(frame, drone_x, drone_y, drone_z, drone_yaw=0.0):
    """
    Drone kamerasindan kirmizi geometrik hedefleri tespit eder.

    Yeni mantik:
    - Hedefin sekli bulunur: circle, square, triangle
    - Her hedefe sabit id atanir: red_circle, red_square, red_triangle
    - Hedef kameranin merkezine yakin degilse centered=False doner
    - centered=True ise koordinat daha guvenilir kabul edilir
    """

    if frame is None:
        return []

    if drone_z is None or drone_z < 0.2:
        return []

    mask = _red_mask(frame, lower_sat=100, lower_val=50)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    img_h, img_w = frame.shape[:2]

    ground_w = 2 * drone_z * math.tan(CAMERA_FOV_RAD / 2)
    ground_h = ground_w * img_h / img_w

    expected_radius_px = (
        TARGET_RADIUS_M / (2 * drone_z * math.tan(CAMERA_FOV_RAD / 2))
    ) * img_w

    min_area = 20
    max_area = math.pi * (expected_radius_px * 5.0) ** 2

    detections = []

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if not (min_area < area < max_area):
            continue

        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            continue

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        shape, target_id = _classify_shape(cnt)

        if target_id == "red_unknown":
            continue

        # Görüntü merkezine göre normalize offset
        norm_x = (cx - img_w / 2) / (img_w / 2)
        norm_y = (cy - img_h / 2) / (img_h / 2)
        center_error = math.sqrt(norm_x * norm_x + norm_y * norm_y)

        centered = center_error <= CENTER_TOLERANCE

        # Kamera düzleminden lokal dünya offseti
        local_x = (cx - img_w / 2) / img_w * ground_w
        local_y = -(cy - img_h / 2) / img_h * ground_h

        body_dx = local_y
        body_dy = -local_x

        wx = drone_x + body_dx * math.cos(drone_yaw) - body_dy * math.sin(drone_yaw)
        wy = drone_y + body_dx * math.sin(drone_yaw) + body_dy * math.cos(drone_yaw)
        if abs(wx) > WORLD_LIMIT_M or abs(wy) > WORLD_LIMIT_M:
            continue

        det = {
            "id": target_id,
            "shape": shape,
            "x": round(wx, 2),
            "y": round(wy, 2),
            "area": round(float(area), 2),
            "px": cx,
            "py": cy,
            "center_error": round(center_error, 3),
            "centered": centered,
        }

        detections.append(det)

        print(
            f"[DRONE-DETECT] id={target_id} shape={shape} "
            f"pixel=({cx},{cy}) centered={centered} "
            f"center_error={center_error:.2f} "
            f"world=({wx:.2f},{wy:.2f}) area={area:.0f}"
        )

    # Aynı frame içinde aynı ID birden fazla çıkarsa, merkeze en yakın olanı al.
    best_by_id = {}

    for det in detections:
        target_id = det["id"]

        if target_id not in best_by_id:
            best_by_id[target_id] = det
            continue

        if det["center_error"] < best_by_id[target_id]["center_error"]:
            best_by_id[target_id] = det

    return list(best_by_id.values())


def verify_from_ika(frame):
    """
    IKA kamerasinda kirmizi hedef var mi kontrol eder.

    Donen deger:
    - found: bool
    - area: kontur alani, yaklasma gostergesi
    - offset: -1 sol, 0 merkez, +1 sag
    """

    if frame is None:
        return False, 0, 0

    mask = _red_mask(frame, lower_sat=100, lower_val=50)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return False, 0, 0

    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)

    if area <= 30:
        return False, 0, 0

    M = cv2.moments(cnt)
    if M["m00"] <= 0:
        return False, 0, 0

    cx = int(M["m10"] / M["m00"])
    img_w = frame.shape[1]

    offset = (cx - img_w / 2) / (img_w / 2)

    print(f"[IKA-CAM] Kirmizi hedef bulundu. area={area:.0f}, offset={offset:.2f}")

    return True, float(area), float(offset)