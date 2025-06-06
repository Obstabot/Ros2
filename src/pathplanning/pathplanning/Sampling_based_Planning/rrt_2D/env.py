"""
Environment for RRT_star 2D (10x10 map with JSON obstacle support)
"""

import os
import json


class Env:
    def __init__(self, origin=(0.0, 0.0), resolution=0.05):
        self.origin = origin                # 그리드 맵 기준 좌표 원점
        self.resolution = resolution        # 1 그리드당 실제 거리 (m)
        self.x_range = (0.0, 10.0)          # x축 10m
        self.y_range = (0.0, 10.0)          # y축 10m
        self.obs = self.obs_map()           # 장애물: 실수 좌표 기반 (x, y)의 set

    def obs_map(self):
        json_file = "/tmp/obstacle_positions.json"
        if not os.path.exists(json_file):
            print(f"[Env] JSON not found: {json_file}")
            return set()

        with open(json_file) as f:
            raw = json.load(f)  # ex: [[1.2, 2.5], [3.4, 7.8], ...]

        obs_set = set()
        inflation = int(0.35 / self.resolution)  # 0.25m 팽창

        for x_real, y_real in raw:
            gx = round((x_real - self.origin[0]) / self.resolution)
            gy = round((y_real - self.origin[1]) / self.resolution)

            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    nx = gx + dx
                    ny = gy + dy
                    x = nx * self.resolution + self.origin[0]
                    y = ny * self.resolution + self.origin[1]

                    if self.x_range[0] <= x <= self.x_range[1] and self.y_range[0] <= y <= self.y_range[1]:
                        obs_set.add((round(x, 2), round(y, 2)))  # 소수점 2자리로 좌표 정리

        print(f"[Env] Loaded {len(obs_set)} inflated obstacles from JSON")
        return obs_set
