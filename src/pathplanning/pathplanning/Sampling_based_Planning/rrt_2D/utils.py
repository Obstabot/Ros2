"""
utils for collision check (set-based JSON obstacles)
@author: huiming zhou (modified)
"""

import math
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from . import env
from .rrt import Node


class Utils:
    def __init__(self):
        self.env = env.Env()
        self.delta = 0.25  # 안전 여유 거리 (m)

    def is_collision(self, start, end):
        # 선분을 일정 간격으로 샘플링하여 장애물 안에 들어가는지 검사
        dist = self.get_dist(start, end)
        steps = int(dist / 0.02)  # 2cm 단위로 충돌 검사
        if steps == 0:
            return self.is_inside_obs(start)

        for i in range(steps + 1):
            x = start.x + (end.x - start.x) * i / steps
            y = start.y + (end.y - start.y) * i / steps
            if self.is_inside_obs(Node((x, y))):
                return True
        return False

    def is_inside_obs(self, node):
        rounded = (round(node.x, 2), round(node.y, 2))
        return rounded in self.env.obs

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)
