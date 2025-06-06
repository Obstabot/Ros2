"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from . import plotting, env


class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, heuristic_type, env_obj=None):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = env_obj if env_obj else env.Env(origin=(0.0,0.0), resolution=0.05)  # class Env

        inflation_radius = 2
        infalted_obs = env.Env.inflate_obstacles(self.Env.obs, inflation_radius)
        self.Env.update_obs(infalted_obs)

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        return self.extract_path(self.PARENT), self.CLOSED

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """
        neighbors = []
        for u in self.u_set:
            x,y = s[0]+u[0], s[1]+u[1]

            if x < 0 or x >= self.Env.x_range or y < 0 or y >= self.Env.y_range:
                continue
            if (x,y) in self.Env.obs:
                continue
            neighbors.append((x,y))

        return neighbors

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        dist = math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

        # for ox, oy in self.obs:
        #     if abs(ox - s_goal[0]) <= 2 and abs(oy - s_goal[1]) <2:
        #         dist *= 2
        #         break
        
        return dist

    def is_collision(self, s_start, s_end):
        # Δ
        dx, dy = s_end[0] - s_start[0],  s_end[1] - s_start[1]
        steps = max(abs(dx), abs(dy))
        if steps == 0:
            return (s_start in self.obs)

        for i in range(steps + 1):
            xi = int(round(s_start[0] + dx * i / steps))
            yi = int(round(s_start[1] + dy * i / steps))
            if (xi, yi) in self.obs:
                return True               # 셀 하나라도 겹치면 충돌
        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])


def main():
    start_world = (0.0, 0.0)
    goal_world  = (9.5, 9.5)

    env_obj = env.Env(origin=(0.0, 0.0), resolution=0.05)

    s_start = (
        int((start_world[0] - env_obj.origin[0]) / env_obj.resolution),
        int((start_world[1] - env_obj.origin[1]) / env_obj.resolution)
    )
    s_goal = (
        int((goal_world[0] - env_obj.origin[0]) / env_obj.resolution),
        int((goal_world[1] - env_obj.origin[1]) / env_obj.resolution)
    )

    if s_goal in env_obj.obs:
        print(f"❌ 목표점 {s_goal}이 장애물에 있습니다!")
        return

    astar = AStar(s_start, s_goal, "euclidean", env_obj=env_obj)
    plot = plotting.Plotting(s_start, s_goal, env_obj=env_obj)

    path, visited = astar.searching()

    for gx, gy in path:
        rx = gx * env_obj.resolution + env_obj.origin[0]
        ry = gy * env_obj.resolution + env_obj.origin[1]
        print(f"grid=({gx:3}, {gy:3}) -> world=({rx:5.2f}, {ry:5.2f})")

    plot.animation(path, visited, "A*")


    # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")


if __name__ == '__main__':
    main()
