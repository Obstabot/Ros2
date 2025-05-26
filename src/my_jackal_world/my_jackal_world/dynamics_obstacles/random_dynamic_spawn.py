import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import random
from geometry_msgs.msg import Quaternion
import math
import os
from ament_index_python.packages import get_package_share_directory

# 경계 설정
MIN_X, MAX_X = 1.0, 9.0
MIN_Y, MAX_Y = 1.0, 9.0

# 자칼 로봇 위치 피하기
START_POS = (0.5, 0.5)
GOAL_POS = (9.5, 9.5)

# 장애물 반지름에 해당하는 안전 거리 (사이즈 고려)
OBSTACLE_RADIUS = 0.15  # box: 0.3 / 2
SAFE_DISTANCE = 1.0

class RandomDynamicSpawner(Node):
    def __init__(self):
        super().__init__('random_dynamic_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        self.spawn_goal_marker()
        self.spawn_obstacles()

    def spawn_goal_marker(self):
        try:
            package_share_directory = get_package_share_directory('my_jackal_world')
            marker_path = os.path.join(package_share_directory, 'models', 'marker.sdf')

            with open(marker_path, 'r') as file:
                marker_xml = file.read()

            req = SpawnEntity.Request()
            req.name = 'goal_marker'
            req.xml = marker_xml
            req.robot_namespace = req.name
            req.initial_pose.position.x = GOAL_POS[0]
            req.initial_pose.position.y = GOAL_POS[1]
            req.initial_pose.position.z = 0.0
            req.initial_pose.orientation.w = 1.0

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f"Spawned goal marker at {GOAL_POS}")

        except Exception as e:
            self.get_logger().error(f"Failed to spawn goal marker: {e}")

    def is_position_valid(self, new_pos, existing_positions):
        if math.dist(new_pos, START_POS) < (OBSTACLE_RADIUS + SAFE_DISTANCE):
            return False
        if math.dist(new_pos, GOAL_POS) < (OBSTACLE_RADIUS + SAFE_DISTANCE):
            return False
        for existing in existing_positions:
            if math.dist(new_pos, existing) < (2 * OBSTACLE_RADIUS):
                return False
        return True

    def spawn_obstacles(self):
        try:
            model_path = os.path.join(
                get_package_share_directory('my_jackal_world'),
                'models',
                'turtlebot3_waffle.urdf'
            )   

            with open(model_path, 'r') as file:
                base_urdf = file.read()
            
            positions = []

            for i in range(2):  # 5개 동적 장애물
                while True:
                    # ⛔ 경계 바깥 튀는 것 방지: 여유 거리 OBSTACLE_RADIUS를 뺌
                    x = random.uniform(MIN_X + OBSTACLE_RADIUS, MAX_X - OBSTACLE_RADIUS)
                    y = random.uniform(MIN_Y + OBSTACLE_RADIUS, MAX_Y - OBSTACLE_RADIUS)
                    pos = (x, y)

                    if self.is_position_valid(pos, positions):
                        positions.append(pos)
                        break
                model_name = f'turtlebot_{i+1}'

                req = SpawnEntity.Request()
                req.name = model_name
                req.xml = base_urdf
                req.robot_namespace = model_name
                req.initial_pose.position.x = x + 0.064
                req.initial_pose.position.y = y
                req.initial_pose.position.z = 0.3

                yaw = 0.0 if i < 2 else math.pi  # 앞 2개는 오른쪽(x+), 뒤 3개는 왼쪽(x-)
                req.initial_pose.orientation.x = 0.0
                req.initial_pose.orientation.y = 0.0
                req.initial_pose.orientation.z = math.sin(yaw / 2)
                req.initial_pose.orientation.w = math.cos(yaw / 2)

                future = self.cli.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                self.get_logger().info(f"Spawned TurtleBot {model_name} at ({x:.2f}, {y:.2f})")

        except Exception as e:
            self.get_logger().error(f"Failed to spawn TurtleBots: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomDynamicSpawner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
