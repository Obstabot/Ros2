import rclpy
from rclpy.node import Node
from planning_path_algorithm.astar.astar_algorithm import AStar
from planning_path_algorithm.common.env import Env
from planning_path_algorithm.controller.path_follower import PathFollower

def main(args=None):
    rclpy.init(args=args)

    scale = 10  # resolution 0.1m 기준
    s_start = (int(round(0.0 * scale)), int(round(0.0 * scale)))
    s_goal = (int(round(9.5 * scale)), int(round(9.5 * scale)))

    # 환경 및 알고리즘 실행
    env = Env()
    astar = AStar(s_start, s_goal, env.obs)
    # print(f"Start: {s_start}, Goal: {s_goal}")
    # print(f"Goal in obstacles? {s_goal in env.obs}")

    path, _ = astar.searching()
    print(f"[DEBUG] path: {path}")

    

    # path 좌표를 (x, y) 실수로 변환 (기본 단위는 그리드라서 실수화 필요할 수 있음)
    path = [(x / scale, y / scale) for x, y in path]

    print("Generated Path:")
    for p in path:
      print(p)


    # path_follower 노드 실행
    follower = PathFollower(path)
    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()
