import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class PathFollower(Node):
    def __init__(self, path, topic_name='/jackal_velocity_controller/cmd_vel_unstamped'):
        super().__init__('path_follower')
        self.publisher = self.create_publisher(Twist, topic_name, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.path = path[::-1]  # (x, y) 좌표 리스트, 목표지점부터 시작 → 역순으로 바꿔야 pop이 효율적
        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0
        self.timer = self.create_timer(0.1, self.follow_path)

        self.target_tolerance = 0.2
        self.max_speed = 0.3
        self.max_angular = 1.0

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_position = (pose.position.x, pose.position.y)

        orientation = pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        self.current_yaw = yaw
        # self.get_logger().info(f"ODOM update: pos={self.current_position}, yaw={self.current_yaw:.2f}")

    def follow_path(self):
        if not self.path:
            self.get_logger().info('Path complete.')
            self.publisher.publish(Twist())  # stop
            return

        target = self.path[-1]
        dx = target[0] - self.current_position[0]
        dy = target[1] - self.current_position[1]
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.current_yaw)

        # self.get_logger().info(
        #   f"Target: {target}, Pos: {self.current_position}, "
        #   f"Dist: {distance:.2f}, AngleDiff: {angle_diff:.2f}"
        # )

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
