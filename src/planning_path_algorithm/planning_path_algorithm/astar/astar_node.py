import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from planning_path_algorithm.astar.astar_algorithm import AStar
from planning_path_algorithm.common.env import Env
from planning_path_algorithm.controller.path_follower import PathFollower
from planning_path_algorithm.utils.scan_to_grid import scan_to_obstacle_grids

class AstarNode(Node):
    def __init__(self):
        super().__init__('astar_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.scale = 10  # 0.1m resolution
        self.env = Env()
        self.path_follower = None

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/front/scan', self.scan_callback, 10)
        self.create_timer(2.0, self.replan_path)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y

        orientation = pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

    def scan_callback(self, msg):
        new_obs = scan_to_obstacle_grids(msg, (self.x, self.y, self.yaw), self.scale)
        self.env.update_obs(new_obs)

    def replan_path(self):
        s_start = (int(round(self.x * self.scale)), int(round(self.y * self.scale)))
        s_goal = (int(round(9.5 * self.scale)), int(round(9.5 * self.scale)))

        astar = AStar(s_start, s_goal, self.env.obs)
        path, _ = astar.searching()
        path = [(x / self.scale, y / self.scale) for x, y in path]

        if path:
            if self.path_follower:
                self.path_follower.destroy_node()
            self.path_follower = PathFollower(path)
            self.get_logger().info('Path re-calculated and following')


def main(args=None):
    rclpy.init(args=args)
    node = AstarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
