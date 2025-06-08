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
        self.declare_parameter('angular_gain',  3.5)
        self.declare_parameter('target_tolerance', 0.1)   # [rad/s per rad]
        self.lookahead = self.get_parameter('lookahead').value
        self.v         = self.get_parameter('linear_speed').value
        self.k_ang     = self.get_parameter('angular_gain').value
        self.tol = self.get_parameter('target_tolerance').value

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
        last_x, last_y = self.path_pts[-1]
        dist_to_goal = math.hypot(px - last_x, py - last_y)
        angle_to_goal = math.atan2(last_y - py, last_x - px)
        yaw_err = abs(self._normalize(angle_to_goal - yaw))

        if dist_to_goal < 0.2 and yaw_err <0.5:
            self.get_logger().info("[Follower] 목표 도달")
            self.cmd_pub.publish(Twist())
            self.path_pts.clear()
            return

        if target is None:
            for i in range(len(self.path_pts)):
                tx, ty = self.path_pts[i]
                if math.hypot(tx - px, ty - py) > self.lookahead:
                    target = (tx, ty)
                    self.idx = i
                    self.get_logger().warn(f"[Follower] 경로 이탈 → index 재탐색 {i}")
                    break
        if target is None:
            self.get_logger().warn("[Follower] 목표점 찾기 실패")
            self.cmd_pub.publish(Twist())
            return

        # ---- 제어 계산 (Pure-Pursuit) ----
        angle_to = math.atan2(target[1] - py, target[0] - px)
        ang_err  = self._normalize(angle_to - yaw)

        cmd = Twist()
        cmd.linear.x  = self.v * max(0.1,1.0 - abs(ang_err))

        if abs(ang_err) > math.pi /2 :
            cmd.linear.x = 0.0

        max_ang = 1.0
        cmd.angular.z = max(-max_ang, min(max_ang, self.k_ang * ang_err))
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
