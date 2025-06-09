import rclpy, time, json
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from pathplanning.Search_based_Planning.Search_2D.Astar import AStar
from pathplanning.Search_based_Planning.Search_2D.env import Env
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class AstarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.path_pub = self.create_publisher(Path, '/planned_path', 1)

        self.ready = False
        self.done  = False

        latch_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            Bool, '/obstacles_ready', self.cb_ready, latch_qos
        )

    def cb_ready(self, msg: Bool):
        self.get_logger().info(f"/obstacles_ready <= {msg.data}")
        if msg.data and not self.done:
            self.ready = True
            self.plan_once()

    def plan_once(self):
        if not self.ready:
            return

        start_world = (0.0, 0.0)
        goal_world = (9.5, 9.5)

        env_obj = Env(origin=(0.0, 0.0), resolution=0.05)

        s_start = (
            int((start_world[0] - env_obj.origin[0]) / env_obj.resolution),
            int((start_world[1] - env_obj.origin[1]) / env_obj.resolution)
        )
        s_goal = (
            int((goal_world[0] - env_obj.origin[0]) / env_obj.resolution),
            int((goal_world[1] - env_obj.origin[1]) / env_obj.resolution)
        )

        astar = AStar(s_start, s_goal, "euclidean", env_obj)

        t0 = time.time()
        grid_path, _ = astar.searching()
        search_time = (time.time() - t0) * 1000  # ms
        self.get_logger().info(f"A* done in {search_time:.1f} ms")

        if not grid_path:
            self.get_logger().info("No path found")
            return

        grid_path = grid_path[::-1]
        self.get_logger().info(f"[GRID]{grid_path}")

        def interpolate_path(path, resolution=0.05):
            from math import sqrt
            interpolated = []
            for i in range(len(path) - 1):
                x0, y0 = path[i]
                x1, y1 = path[i + 1]
                dx, dy = x1 - x0, y1 - y0
                dist = sqrt(dx ** 2 + dy ** 2)
                steps = max(1, int(dist / resolution))
                for j in range(steps):
                    ratio = j / steps
                    x = x0 + ratio * dx
                    y = y0 + ratio * dy
                    interpolated.append((x, y))
            interpolated.append(path[-1])
            return interpolated

        def downsample_path(path, step=5):
            return path[::step] + [path[-1]]

        smooth_path = interpolate_path(grid_path, resolution=0.2)
        # smooth_path = downsample_path(smooth_path, step=3)

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        res = astar.Env.resolution
        org = astar.Env.origin

        total_length = 0.0
        prev = None
        for gx, gy in smooth_path:
            wx, wy = gx * res + org[0], gy * res + org[1]
            pose = PoseStamped()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            if prev:
                dx = wx - prev[0]
                dy = wy - prev[1]
                total_length += (dx**2 + dy**2) ** 0.5
            prev = (wx, wy)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path published: {len(path_msg.poses)} waypoints")
        self.get_logger().info(f"경로 길이 : {total_length:.2f}m")
        self.get_logger().info(f"탐색 시간 : {search_time: .2f} ms")

        self.done = True


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AstarPlanner())
    rclpy.shutdown()


if __name__ == '__main__':
    main()