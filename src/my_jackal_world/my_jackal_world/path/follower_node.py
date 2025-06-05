#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from typing import List, Tuple

class PathFollower(Node):
    """
    ┌─────────────────────────────────────────────┐
    │ Pure-Pursuit 기반 경로 추종 노드             │
    │  * /planned_path 구독                       │
    │  * /odom      구독                          │
    │  * /jackal_velocity_controller/cmd_vel_unstamped 발행 │
    └─────────────────────────────────────────────┘
    """
    def __init__(self):
        super().__init__('path_follower')

        # ----- 파라미터 -----
        self.declare_parameter('lookahead',     0.4)   # [m]
        self.declare_parameter('linear_speed',  0.3)   # [m/s]
        self.declare_parameter('angular_gain',  2.0)   # [rad/s per rad]
        self.lookahead = self.get_parameter('lookahead').value
        self.v         = self.get_parameter('linear_speed').value
        self.k_ang     = self.get_parameter('angular_gain').value

        # ----- 토픽 -----
        self.cmd_pub = self.create_publisher(
            Twist,
            '/jackal_velocity_controller/cmd_vel_unstamped',
            10)

        self.sub_path = self.create_subscription(
            Path,
            '/planned_path',
            self.cb_path,
            1)

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.cb_odom,
            10)

        self.path_pts: list[tuple[float,float]] = []
        self.idx: int = 0          # [(x, y), …]  world coords

    # ============ Path 콜백 ============
    def cb_path(self, msg: Path):
        self.path_pts = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.idx = 0
        self.get_logger().info(f"[Follower] 새 경로 수신 ({len(self.path_pts)} pts)")

    # ============ Odometry 콜백 =========
    def cb_odom(self, msg: Odometry):
        if not self.path_pts:  # 경로 없으면 정지
            return
        
        # 로봇 현재 위치·방향
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        _, _, yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w])

        # ---- look-ahead 포인트 선택 ----
        target = None
        while self.idx < len(self.path_pts):
            tx, ty = self.path_pts[self.idx]
            if math.hypot(tx - px, ty - py) > self.lookahead:
                target = (tx, ty)
                break
            self.idx += 1

        # 경로 끝에 도달
        if target is None:
            self.cmd_pub.publish(Twist())        # 정지
            self.path_pts.clear()
            self.get_logger().info("[Follower] 목표 도달, 경로 완료")
            return

        # ---- 제어 계산 (Pure-Pursuit) ----
        angle_to = math.atan2(target[1] - py, target[0] - px)
        ang_err  = self._normalize(angle_to - yaw)

        cmd = Twist()
        cmd.linear.x  = self.v
        cmd.angular.z = self.k_ang * ang_err
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _normalize(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathFollower())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
