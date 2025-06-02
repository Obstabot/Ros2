import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion

from planning_path_algorithm.astar.astar_algorithm import AStar
from planning_path_algorithm.common.env import Env

class PathFollower(Node):
    def __init__(self, topic_name='/jackal_velocity_controller/cmd_vel_unstamped'):
        super().__init__('path_follower')

        self.publisher = self.create_publisher(Twist, topic_name, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)

        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0
        self.path = []
        self.frame_id = 'odom'

        self.target_tolerance = 0.2
        self.max_speed = 0.3
        self.max_angular = 1.0

        self.env = Env()  # 장애물 지도 (수정 가능)
        self.scale = 10
        self.s_goal = (int(round(9.5 * self.scale)), int(round(9.5 * self.scale)))

        self.create_timer(5.0, self.update_path)     # 경로 재계산 (5초마다)
        self.create_timer(0.1, self.follow_path)     # 경로 추종

    def update_path(self):
        s_start = (int(round(self.current_position[0] * self.scale)), 
                   int(round(self.current_position[1] * self.scale)))

        astar = AStar(s_start, self.s_goal)
        path, _ = astar.searching()

        if path:
            self.path = [(x / self.scale, y / self.scale) for x, y in path][::-1]
            self.publish_path(self.path)
            self.get_logger().info(f'New path calculated from {s_start} to {self.s_goal}.')
        else:
            self.get_logger().warn('No valid path found.')

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path[::-1]:  # RViz는 시작점부터 보여주기
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_position = (pose.position.x, pose.position.y)
        orientation = pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.current_yaw = yaw

    def follow_path(self):
        if not self.path:
            return

        target = self.path[-1]
        dx = target[0] - self.current_position[0]
        dy = target[1] - self.current_position[1]
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.current_yaw)

        msg = Twist()
        if abs(angle_diff) > 0.2:
            msg.angular.z = max(-self.max_angular, min(self.max_angular, angle_diff))
        elif distance > self.target_tolerance:
            msg.linear.x = self.max_speed
        else:
            self.path.pop()

        self.publisher.publish(msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
