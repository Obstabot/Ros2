import rclpy, time
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from pathplanning.Sampling_based_Planning.rrt_2D.rrt_star import RrtStar
from pathplanning.Sampling_based_Planning.rrt_2D.env import Env

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class RRTStarPlanner(Node):
    def __init__(self):
        super().__init__('rrt_star_planner')
        self.path_pub = self.create_publisher(Path, '/planned_path', 1)

        self.ready = False
        self.done = False

        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(Bool, '/obstacles_ready', self.cb_ready, latch_qos)

    def cb_ready(self, msg: Bool):
        self.get_logger().info(f"/obstacles_ready <= {msg.data}")
        if msg.data and not self.done:
            self.ready = True
            self.plan_once()

    def plan_once(self):
        if not self.ready:
            return

        start = (0.0, 0.0)
        goal = (9.5, 9.5)

        env_obj = Env(origin=(0.0, 0.0), resolution=0.05)

        if (round(goal[0], 2), round(goal[1], 2)) in env_obj.obs:
            self.get_logger().info("❌ 목표점이 장애물입니다.")
            return

        planner = RrtStar(start, goal, step_len=1.0, goal_sample_rate=0.1,
                          search_radius=2.0, iter_max=3000)

        planner.env = env_obj
        planner.utils.env = env_obj
        planner.plotting.env = env_obj

        t0 = time.time()
        planner.planning()
        self.get_logger().info(f"RRT* done in {(time.time()-t0)*1000:.1f} ms")

        raw_path = planner.path[::-1][1:]  # start부터 goal까지

        if not raw_path:
            self.get_logger().info("No path found")
            return

        self.get_logger().info(f"[PATH] {raw_path}")

        # 보간 및 다운샘플링
        def interpolate_path(path, resolution=0.2):
            from math import sqrt
            interpolated = []
            for i in range(len(path)-1):
                x0, y0 = path[i]
                x1, y1 = path[i+1]
                dx, dy = x1 - x0, y1 - y0
                dist = sqrt(dx**2 + dy**2)
                steps = max(1, int(dist / resolution))
                for j in range(steps):
                    ratio = j / steps
                    x = x0 + ratio * dx
                    y = y0 + ratio * dy
                    interpolated.append((x, y))
            interpolated.append(path[-1])
            return interpolated

        def downsample_path(path, step=3):
            return path[::step] + [path[-1]]

        smooth_path = interpolate_path(raw_path, resolution=0.1)
        smooth_path = downsample_path(smooth_path, step=2)

        # Path 메시지 생성
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in smooth_path:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path published: {len(path_msg.poses)} waypoints")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RRTStarPlanner())
    rclpy.shutdown()
