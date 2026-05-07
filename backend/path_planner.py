import math
import heapq

class PathPlanner:
    def __init__(self, grid_size=0.3, arena_min=-7, arena_max=7):
        self.grid_size = grid_size
        self.arena_min = arena_min
        self.arena_max = arena_max
        self.width = int((arena_max - arena_min) / grid_size)
        self.grid = [[0] * self.width for _ in range(self.width)]

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.arena_min) / self.grid_size)
        gy = int((wy - self.arena_min) / self.grid_size)
        gx = max(0, min(self.width - 1, gx))
        gy = max(0, min(self.width - 1, gy))
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.grid_size + self.arena_min + self.grid_size / 2
        wy = gy * self.grid_size + self.arena_min + self.grid_size / 2
        return wx, wy

    def update_from_lidar(self, robot_x, robot_y, robot_yaw, ranges):
        """LiDAR verisinden grid'i guncelle"""
        n = len(ranges)
        for i, r in enumerate(ranges):
            if r <= 0.5 or r > 12:
                continue
            angle = robot_yaw + (i / n) * 2 * math.pi - math.pi
            ox = robot_x + r * math.cos(angle)
            oy = robot_y + r * math.sin(angle)
            gx, gy = self.world_to_grid(ox, oy)
            if 0 <= gx < self.width and 0 <= gy < self.width:
                self.grid[gy][gx] = 1
                # Engel etrafini da isaretele (guvenlik mesafesi)
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < self.width and 0 <= ny < self.width:
                            self.grid[ny][nx] = 1

    def astar(self, start_x, start_y, goal_x, goal_y):
        """A* ile yol bul"""
        sx, sy = self.world_to_grid(start_x, start_y)
        gx, gy = self.world_to_grid(goal_x, goal_y)

        # Hedef engel uzerindeyse en yakin bos noktaya tasi
        if self.grid[gy][gx] == 1:
            best = None
            best_dist = 999
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.width:
                        if self.grid[ny][nx] == 0:
                            d = abs(dx) + abs(dy)
                            if d < best_dist:
                                best_dist = d
                                best = (nx, ny)
            if best:
                gx, gy = best

        open_set = []
        heapq.heappush(open_set, (0, sx, sy))
        came_from = {}
        g_score = {(sx, sy): 0}
        closed = set()

        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

        while open_set:
            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if abs(cx - gx) <= 1 and abs(cy - gy) <= 1:
                # Yolu geri izle
                path = []
                current = (cx, cy)
                while current in came_from:
                    wx, wy = self.grid_to_world(current[0], current[1])
                    path.append((wx, wy))
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.width and 0 <= ny < self.width:
                    if self.grid[ny][nx] == 1:
                        continue
                    if (nx, ny) in closed:
                        continue
                    cost = 1.0 if abs(dx) + abs(dy) == 1 else 1.414
                    new_g = g_score[(cx, cy)] + cost
                    if new_g < g_score.get((nx, ny), 999):
                        g_score[(nx, ny)] = new_g
                        h = math.sqrt((nx - gx)**2 + (ny - gy)**2)
                        f = new_g + h
                        came_from[(nx, ny)] = (cx, cy)
                        heapq.heappush(open_set, (f, nx, ny))

        return []  # Yol bulunamadi