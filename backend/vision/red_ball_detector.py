import cv2
import numpy as np

def detect_red_ball(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 🔥 Kırmızı aralık
    lower1 = np.array([0, 100, 50])
    upper1 = np.array([15, 255, 255])

    lower2 = np.array([165, 100, 50])
    upper2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)

    mask = mask1 + mask2

    # 🔥 Noise temizleme
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 🔥 BURASI KRİTİK
    found = False
    cx, cy = None, None
    area = 0

    if len(contours) > 0:
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)

        if area > 50:
            x, y, w, h = cv2.boundingRect(cnt)

            cx = x + w // 2
            cy = y + h // 2

            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 5, (0,0,255), -1)

            cv2.putText(frame, "RED BALL",
                        (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0,255,0), 2)

            found = True

    return frame, found, cx, cy, area