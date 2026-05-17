import threading

state_lock = threading.Lock()

robot_state = {
    "ika_x": 0.0,
    "ika_y": 0.0,
    "ika_yaw": 0.0,
    "ika_odom_ready": False,

    "lidar_ranges": [],
    "lidar_ready": False,
}