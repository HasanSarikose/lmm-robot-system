import cv2
import numpy as np
import math


def detect_from_drone(frame, drone_x, drone_y, drone_z):
    """Drone kamerasından kirmizi top tespit et ve dunya koordinatina donustur"""
    if frame is None:
        return []

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 150, 100]), np.array([8, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([172, 150, 100]), np.array([180, 255, 255]))
    mask = mask1 | mask2

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    img_h, img_w = frame.shape[:2]
    fov = 1.8
    found = []

    expected_radius_px = (0.1 / (2 * drone_z * math.tan(fov/2))) * img_w
    min_area = math.pi * (expected_radius_px * 0.3) ** 2
    max_area = math.pi * (expected_radius_px * 3.0) ** 2

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area < area < max_area:
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            if circularity < 0.65:
                continue
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                px = int(M["m10"] / M["m00"])
                py = int(M["m01"] / M["m00"])
                ground_w = 2 * drone_z * math.tan(fov / 2)
                ground_h = ground_w * img_h / img_w
                wx = drone_x + (px - img_w/2) / img_w * ground_w
                wy = drone_y - (py - img_h/2) / img_h * ground_h
                if abs(wx) > 7 or abs(wy) > 7:
                    continue
                found.append({"x": round(wx, 2), "y": round(wy, 2), "area": area})
                print(f"[DETECT] Top: piksel({px},{py}) -> dunya({wx:.1f},{wy:.1f})")

    return found


def verify_from_ika(frame):
    """IKA kamerasinda kirmizi top var mi kontrol et"""
    if frame is None:
        return False, 0, 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([15, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([165, 100, 50]), np.array([180, 255, 255]))
    mask = mask1 + mask2

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area > 30:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                img_w = frame.shape[1]
                offset = (cx - img_w / 2) / (img_w / 2)
                print(f"[IKA-CAM] BULDU! Alan={area:.0f} Offset={offset:.2f}")
                return True, area, offset

    return False, 0, 0