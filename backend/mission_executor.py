import time
import threading

from robot_controllers import DroneCtrl, IKACtrl, ArmCtrl, ign_cmd
from vision.ball_detector import detect_from_drone, verify_from_ika

mission_log = []
mission_running = False


def get_log():
    return mission_log


def execute_mission(llm_output, frames_dict=None):
    global mission_log, mission_running

    if mission_running:
        mission_log.append("[SISTEM] Zaten calisan bir gorev var. Yeni gorev baslatilmadi.")
        return "Zaten calisan bir gorev var"

    mission_running = True
    mission_log = []

    def log(msg):
        print(msg)
        mission_log.append(msg)

    def approach_ball_with_ika_camera(ika, frames_dict, log):
        """
        IKA hedef bolgesine geldikten sonra,
        IKA kamerasi ile kirmizi hedefi merkezleyip yavasca yaklasir.
        """
        if not frames_dict:
            log("[IKA-CAM] Frame kaynagi yok.")
            return False

        log("[IKA-CAM] Kamera ile son yaklasma basladi.")

        for step in range(70):
            frame = frames_dict.get("ika")
            found, area, offset = verify_from_ika(frame)

            ranges = ika.read_lidar()
            obs, min_d = ika.check_front(ranges)

            if obs and min_d < 0.40:
                ika.stop()
                log(f"[IKA-CAM] Cok yakin hedef/engel: {min_d:.2f}m, duruldu.")
                return True

            if found:
                log(f"[IKA-CAM] found=True area={area:.0f} offset={offset:.2f}")

                if area > 5000:
                    ika.stop()
                    log(f"[IKA-CAM] Hedef erisim mesafesinde. Alan={area:.0f}")
                    return True

                if abs(offset) > 0.18:
                    az = -0.18 if offset > 0 else 0.18
                    ign_cmd(0.03, az)
                else:
                    ign_cmd(0.08, 0.0)

                time.sleep(0.25)

            else:
                log(f"[IKA-CAM] Hedef gorulmedi. Arama step={step}")

                if step < 15:
                    ign_cmd(0.0, 0.22)
                elif step < 30:
                    ign_cmd(0.0, -0.22)
                elif step < 50:
                    ign_cmd(0.05, 0.18)
                else:
                    ign_cmd(0.05, -0.18)

                time.sleep(0.3)

        ika.stop()
        log("[IKA-CAM] Son yaklasma basarisiz.")
        return False

    def run():
        global mission_running

        try:
            drone = DroneCtrl()
            ika = IKACtrl()
            arm = ArmCtrl()

            log("=" * 50)
            log("GOREV BASLADI")
            log("=" * 50)

            log("[ADIM 1] Drone kalkiyor...")
            drone.takeoff(3)
            time.sleep(1)

            # ID'yi detector'dan DEGIL, taranan bolgeden veriyoruz.
            # Koordinat yine kameradan merkezleme ile aliniyor.
            scan_points = [
                (4,-2),
                (0, -2),
                (4, -1),
                (-4, 1),
                (0, 1),
                (4, 1),
                (-4, 4),
                (0, 4),
                (4, 4),
            ]

            expected_target_ids = {
                "red_circle",
                "red_square",
                "red_triangle",
            }
            found_targets = {}

            for i, scan_point in enumerate(scan_points):
                sx, sy = scan_point

                log(
                    f"[DRONE] Tarama {i + 1}/{len(scan_points)}: "
                    f"arama noktasi ({sx}, {sy})"
                )

                drone.goto(sx, sy, 3)
                time.sleep(1.2)

                if not frames_dict:
                    log("[DRONE] Frame kaynagi yok.")
                    continue

                frame = frames_dict.get("drone")
                detections = detect_from_drone(frame, drone.x, drone.y, drone.z)

                if not detections:
                    log("[DRONE] Bu arama noktasinda kirmizi geometrik hedef gorulmedi.")
                    continue

                # Daha önce kaydedilmiş ID'leri tekrar işleme.
                new_detections = [
                    d for d in detections
                    if d.get("id") in expected_target_ids
                    and d.get("id") not in found_targets
                ]

                if not new_detections:
                    log("[DRONE] Gorulen hedefler daha once kaydedilmis.")
                    continue

                # Önce zaten merkezde olan hedefleri direkt kaydet.
                centered_detections = [
                    d for d in new_detections
                    if d.get("centered", False)
                ]

                for det in centered_detections:
                    target_id = det["id"]
                    found_targets[target_id] = det

                    log(
                        f"[DRONE] HEDEF BULUNDU: {target_id} "
                        f"shape={det['shape']} "
                        f"({det['x']:.2f}, {det['y']:.2f}) "
                        f"center_error={det['center_error']}"
                    )

                # Eğer bu frame'de yeni merkezlenmiş hedef bulunduysa devam et.
                if centered_detections:
                    if len(found_targets) == len(expected_target_ids):
                        log("[DRONE] Tum hedef ID'leri bulundu. Tarama erken bitiriliyor.")
                        break
                    continue

                # Merkezde değilse, kameraya en yakın yeni hedefi refine et.
                best = min(
                    new_detections,
                    key=lambda d: d.get("center_error", 999)
                )

                target_id = best["id"]

                log(
                    f"[DRONE] {target_id} goruldu ama merkezde degil. "
                    f"Refine noktasi=({best['x']:.2f}, {best['y']:.2f}) "
                    f"center_error={best['center_error']}"
                )

                drone.goto(best["x"], best["y"], 3)
                time.sleep(1.0)

                frame2 = frames_dict.get("drone")
                refined = detect_from_drone(frame2, drone.x, drone.y, drone.z)

                if not refined:
                    log(f"[DRONE] Refine sonrasi {target_id} tekrar gorulemedi.")
                    continue

                # Refine sonrasında aynı ID'yi ara.
                same_id_refined = [
                    d for d in refined
                    if d.get("id") == target_id
                ]

                if not same_id_refined:
                    nearest = min(
                        refined,
                        key=lambda d: ((d.get("x", 999) - best["x"]) ** 2 + (d.get("y", 999) - best["y"]) ** 2) ** 0.5
                    )

                    dist_to_previous = (
                        (nearest.get("x", 999) - best["x"]) ** 2 +
                        (nearest.get("y", 999) - best["y"]) ** 2
                    ) ** 0.5

                    if target_id == "red_triangle" and dist_to_previous < 0.8:
                        nearest["id"] = "red_triangle"
                        nearest["shape"] = "triangle"
                        same_id_refined = [nearest]

                        log(
                            f"[DRONE] Refine sonrasi triangle yakin hedefle dogrulandi. "
                            f"koord=({nearest['x']:.2f}, {nearest['y']:.2f}) "
                            f"mesafe={dist_to_previous:.2f}"
                        )
                    else:
                        log(f"[DRONE] Refine sonrasi {target_id} ID'si dogrulanamadi.")
                        continue
                best2 = min(
                    same_id_refined,
                    key=lambda d: d.get("center_error", 999)
                )

                if best2.get("centered", False):
                    found_targets[target_id] = best2

                    log(
                        f"[DRONE] HEDEF MERKEZLENDI: {target_id} "
                        f"shape={best2['shape']} "
                        f"({best2['x']:.2f}, {best2['y']:.2f}) "
                        f"center_error={best2['center_error']}"
                    )

                    if len(found_targets) == len(expected_target_ids):
                        log("[DRONE] Tum hedef ID'leri bulundu. Tarama erken bitiriliyor.")
                        break
                else:
                    log(
                        f"[DRONE] {target_id} hala merkezde degil. "
                        f"Kaydedilmedi. center_error={best2['center_error']}"
                    )

            confirmed = list(found_targets.values())


            for i, h in enumerate(confirmed):
                log(
                    f"  Hedef {i + 1}: id={h['id']} shape={h['shape']} "
                    f"koord=({h['x']:.2f}, {h['y']:.2f}) "
                    f"center_error={h['center_error']}"
                )
            confirmed = list(found_targets.values())

            log(f"[DRONE] Tarama bitti. Onaylanan hedef sayisi: {len(confirmed)}")

            for i, h in enumerate(confirmed):
                log(
                    f"  Hedef {i + 1}: id={h['id']} shape={h['shape']} "
                    f"koord=({h['x']:.2f}, {h['y']:.2f}) "
                    f"center_error={h['center_error']}"
                )

            if not confirmed:
                log("[GOREV] Hedef bulunamadi.")
                drone.land()
                return

            drone.goto(0, 0, 3)
            log("[DRONE] Koordinatlar IKA'ya aktarildi.")

            # ADIM 2: IKA hedeflere gider
            for i, target in enumerate(confirmed):
                target_id = target.get("id", "unknown")
                target_shape = target.get("shape", "unknown")

                log(
                    f"[ADIM 2.{i + 1}] Hedef {i + 1}: "
                    f"{target_id} / {target_shape} "
                    f"({target['x']:.2f}, {target['y']:.2f})"
                )

                arm.home()
                time.sleep(0.5)

                success = ika.navigate_to(target["x"], target["y"], stop_distance=0.9)

                if success:
                    log("[IKA] Bolgeye ulasildi. Kamera ile hedefe yaklasiliyor...")

                    found_target = approach_ball_with_ika_camera(ika, frames_dict, log)

                    if found_target:
                        arm.pick()
                        time.sleep(0.5)

                        carry_stop_event, carry_thread = arm.start_carrying_target(target_id, ika)

                        log(f"[GOREV] {target_id} icin eve donus...")
                        arm.home()
                        ika.return_home()

                        carry_stop_event.set()
                        carry_thread.join(timeout=1.0)
                        arm.place()
                        drop_positions = {
                            "red_triangle": (-2.4, -0.7),
                            "red_circle": (-2.0, -0.7),
                            "red_square": (-1.6, -0.7),
                        }

                        drop_x, drop_y = drop_positions.get(target_id, (-2.0, -0.7))
                        arm.place_target(target_id, drop_x, drop_y)

                        log(f"[GOREV] {target_id} hedefi tamamlandi.")
                    else:
                        log(f"[GOREV] {target_id} kamerada bulunamadi.")
                else:
                    log(f"[GOREV] {target_id} bolgesine ulasilamadi.")

            drone.land()
            arm.home()
            ika.stop()

            log("=" * 50)
            log("GOREV TAMAMLANDI!")
            log("=" * 50)

        except Exception as e:
            log(f"[HATA] Gorev sirasinda hata olustu: {e}")

        finally:
            mission_running = False

    t = threading.Thread(target=run, daemon=True)
    t.start()

    return "Gorev baslatildi"