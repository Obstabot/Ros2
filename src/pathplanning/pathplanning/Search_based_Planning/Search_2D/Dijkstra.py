"""
Dijkstra 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from . import plotting, env

from .Astar import AStar


class Dijkstra(AStar):
    """Dijkstra set the cost as the priority 
    """
    def __init__(self, s_start, s_goal, heuristic_type, env_obj):
        super().__init__(s_start, s_goal, heuristic_type)

    def searching(self):
        """
        Breadth-first Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        # self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (0, self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s

                    # best first set the heuristics as the priority 
                    heapq.heappush(self.OPEN, (new_cost, s_n))

        return self.extract_path(self.PARENT), self.CLOSED


def main():
    start_world = (0,0)
    goal_world=(9.5,9.5)

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
    else:
        print(f"✅ 목표점 {s_goal}은 통과 가능한 위치입니다.")

    dijkstra = Dijkstra(s_start, s_goal, 'None', env_obj)
    plot = plotting.Plotting(s_start, s_goal, env_obj=env_obj)

    path, visited = dijkstra.searching()
    plot.animation(path, visited, "Dijkstra's")  # animation generate


if __name__ == '__main__':
    main()
