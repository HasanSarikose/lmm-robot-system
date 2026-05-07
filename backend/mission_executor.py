import time
import threading

from robot_controllers import DroneCtrl, IKACtrl, ArmCtrl, ign_cmd
from vision.ball_detector import detect_from_drone, verify_from_ika

mission_log = []

def get_log():
    return mission_log


def execute_mission(llm_output, frames_dict=None):
    global mission_log
    mission_log = []

    def log(msg):
        print(msg)
        mission_log.append(msg)

    def run():
        drone = DroneCtrl()
        ika = IKACtrl()
        arm = ArmCtrl()

        log("=" * 50)
        log("GOREV BASLADI")
        log("=" * 50)

        # ADIM 1: Drone tarama
        log("[ADIM 1] Drone kalkiyor...")
        drone.takeoff(3)
        time.sleep(1)

        scan_points = [
            (-4, -2), (-2, -2), (0, -2), (2, -2), (4, -2),
            (-4, 0), (-2, 0), (0, 0), (2, 0), (4, 0),
            (-4, 2), (-2, 2), (0, 2), (2, 2), (4, 2),
            (-4, 4), (-2, 4), (0, 4), (2, 4), (4, 4),
            (-3, -1), (-1, -1), (1, -1), (3, -1),
            (-3, 1), (-1, 1), (1, 1), (3, 1),
            (-3, 3), (-1, 3), (1, 3), (3, 3),
        ]

        all_detections = []
        for i, (sx, sy) in enumerate(scan_points):
            log(f"[DRONE] Tarama {i+1}/{len(scan_points)}: ({sx}, {sy})")
            drone.goto(sx, sy, 3)
            time.sleep(3)

            if frames_dict:
                frame = frames_dict.get("drone")
                balls = detect_from_drone(frame, drone.x, drone.y, drone.z)
                all_detections.extend(balls)

        # NMS clustering
        clusters = []
        for det in all_detections:
            merged = False
            for c in clusters:
                if abs(c["x"] - det["x"]) < 1.5 and abs(c["y"] - det["y"]) < 1.5:
                    c["count"] += 1
                    c["x"] = (c["x"] * (c["count"]-1) + det["x"]) / c["count"]
                    c["y"] = (c["y"] * (c["count"]-1) + det["y"]) / c["count"]
                    merged = True
                    break
            if not merged:
                clusters.append({"x": det["x"], "y": det["y"], "count": 1})

        confirmed = [b for b in clusters if b["count"] >= 3]

        log(f"[DRONE] Tarama bitti. Ham:{len(all_detections)} Cluster:{len(clusters)} Onaylanan:{len(confirmed)}")
        for i, b in enumerate(confirmed):
            log(f"  Top {i+1}: ({b['x']:.1f}, {b['y']:.1f}) - {b['count']}x")

        if not confirmed:
            log("[GOREV] Top bulunamadi.")
            drone.land()
            return

        drone.goto(0, 0, 3)
        log("[DRONE] Koordinatlar IKA'ya aktarildi.")

        # ADIM 2: Her top icin git + topla
        for i, ball in enumerate(confirmed):
            log(f"[ADIM 2.{i+1}] Top {i+1}: ({ball['x']:.1f}, {ball['y']:.1f})")
            arm.home()
            time.sleep(0.5)

            success = ika.navigate_to(ball["x"], ball["y"])
            if success:
                log("[IKA] Bolgeye ulasildi. Kamera ile top araniyor...")

                found_ball = False
                for search in range(40):
                    if not frames_dict:
                        time.sleep(0.1)
                        continue

                    # Engel kontrolu
                    ranges = ika.read_lidar()
                    obs, _ = ika.check_front(ranges)
                    if obs:
                        ign_cmd(-0.1, 0.0)
                        time.sleep(0.5)
                        ign_cmd(0.0, 0.4)
                        time.sleep(0.5)
                        continue

                    frame = frames_dict.get("ika")
                    found, area, offset = verify_from_ika(frame)

                    if found:
                        if area > 2000:
                            ika.stop()
                            log(f"[IKA-CAM] Top tam onde! Alan={area:.0f}")
                            found_ball = True
                            break
                        else:
                            if abs(offset) > 0.15:
                                az = -0.2 if offset > 0 else 0.2
                                ign_cmd(0.1, az)
                            else:
                                ign_cmd(0.15, 0.0)
                            time.sleep(0.3)
                    else:
                        if search < 12:
                            ign_cmd(0.0, 0.3)
                        elif search < 24:
                            ign_cmd(0.0, -0.3)
                        else:
                            ign_cmd(0.1, 0.2)
                        time.sleep(0.4)
                    time.sleep(0.1)

                ika.stop()

                if found_ball:
                    log(f"[GOREV] Top {i+1} bulundu! Kol topluyor...")
                    arm.pick()
                    time.sleep(1)
                    log(f"[GOREV] Top {i+1} toplandi. Eve donus...")
                    arm.home()
                    ika.return_home()
                    arm.place()
                    log(f"[GOREV] Top {i+1} birakildi.")
                else:
                    log(f"[GOREV] Top {i+1} kamerada bulunamadi.")
            else:
                log(f"[GOREV] Top {i+1} bolgeye ulasilamadi.")

        # ADIM 3: Bitis
        drone.land()
        arm.home()
        ika.stop()
        log("=" * 50)
        log("GOREV TAMAMLANDI!")
        log("=" * 50)

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return "Gorev baslatildi"