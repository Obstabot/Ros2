import math
import heapq
from planning_path_algorithm.common.env import Env

class AStar:
    def __init__(self, s_start, s_goal, heuristic_type="euclidean"):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = Env()
        self.Env.update_obs(self.Env.obs_map())
        self.u_set = self.Env.motions
        self.obs = self.Env.obs

        self.OPEN = []
        self.CLOSED = []
        self.PARENT = dict()
        self.g = dict()

    def searching(self):
        # print(f"[DEBUG] Obstacle positions: {sorted(self.obs)}")

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN, (self.f_value(self.s_start), self.s_start))

        print("Start A* Search")

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            # print(f"Expanding node : {s}")

            if s == self.s_goal:
                print("[INFO] Goal reached.")
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        if self.s_goal not in self.PARENT:
            print(f"[WARNING] Goal {self.s_goal} was not reached! It may be blocked or invalid.")
            return [], []

        return self.extract_path(self.PARENT), self.CLOSED

    def get_neighbor(self, s):
        # neighbors = [(round(s[0] + u[0]), round(s[1] + u[1])) for u in self.u_set]
        # print(f"[DEBUG] Neighbors of {s}: {neighbors}")
        return [(round(s[0] + u[0]), round(s[1] + u[1])) for u in self.u_set]

    def cost(self, s_start, s_goal):
        if self.is_collision(s_start, s_goal):
            return math.inf
        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        start_grid = int((round(s_start[0]))), int(round(s_start[1]))
        end_grid = (int(round(s_end[0]))), int(round(s_end[1]))
        print(f"start_grid:{start_grid} end_grid:{end_grid}")

        if start_grid in self.obs or end_grid in self.obs:
            print(f"collision detected from {start_grid} to {end_grid}")
            return True
        return False

    def f_value(self, s):
        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        path = [self.s_goal]
        s = self.s_goal
        while True:
            s = PARENT[s]
            path.append(s)
            if s == self.s_start:
                break
        return list(path)

    def heuristic(self, s):
        goal = self.s_goal
        if self.heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        return math.hypot(goal[0] - s[0], goal[1] - s[1])
