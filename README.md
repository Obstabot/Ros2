### Overview
---
이 프로젝트는 Jackal 로봇이 10m × 10m 맵에서 랜덤하게 배치된 장애물을 피해 목표 지점까지 도달하는 경로를 생성하기 위해 다양한 경로 탐색 알고리즘의 성능을 비교 분석하는 것을 목적으로 한다.
A*, Dijkstra, RRT* 알고리즘을 적용하여 각 알고리즘의 경로 생성 성공률, 실제 목표 도달률, 경로 길이, waypoint 수, 탐색 시간, 실패 원인 등을 정량적으로 비교하였다.

---
### Directory structure
```
└── my_jackal_world
  └── launch
      ├── astar_path.launch.py
      ├── dijkstra_path.launch.py
      ├── rrtstar_path.launch.py
      ├── dynamics_mode.launch.py
      ├── random_mode.launch.py
      └── simple_world.launch.py
  └── models
      ├── cylinder.sdf
      ├── marker.sdf
      └── turtlebot3_waffle.urdf
  └── my_jackal_world
      ├── dynamics_obstacles
          ├── central_reset_node.py
          ├── multi_controller_launcher.py
          ├── random_dynamic_spawn.py
          └── turtlebot_controller.py
      ├── path
          ├── astar_planner_node.py
          ├── dijkstra_planner_node.py
          ├── rrtstar_planner_node.py
          └── follower_node.py
      └── random_spawn.py
  └── worlds
└── pathplanning
  └── Search-based Planning
      ├── Breadth-First Searching (BFS)
      ├── Depth-First Searching (DFS)
      ├── Best-First Searching
      ├── Dijkstra's
      ├── A*
      ├── Bidirectional A*
      ├── Anytime Repairing A*
      ├── Learning Real-time A* (LRTA*)
      ├── Real-time Adaptive A* (RTAA*)
      ├── Lifelong Planning A* (LPA*)
      ├── Dynamic A* (D*)
      ├── D* Lite
      └── Anytime D*
  └── Sampling-based Planning
      ├── RRT
      ├── RRT-Connect
      ├── Extended-RRT
      ├── Dynamic-RRT
      ├── RRT*
      ├── Informed RRT*
      ├── RRT* Smart
      ├── Anytime RRT*
      ├── Closed-Loop RRT*
      ├── Spline-RRT*
      ├── Fast Marching Trees (FMT*)
      └── Batch Informed Trees (BIT*)
```