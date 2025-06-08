import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import math
import random
import os
from ament_index_python.packages import get_package_share_directory
import json
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 경계 설정
MIN_X, MAX_X = 0.0, 10.0
MIN_Y, MAX_Y = 0.0, 10.0

# 로봇 시작 위치, 목표 위치
START_POS = (0.0, 0.0)
GOAL_POS = (9.5,9.5)

# 설정 값
OBSTACLE_RADIUS = 0.2
SAFE_DISTANCE = 1.0

class RandomObstacleSpawner(Node):
    def __init__(self):
        super().__init__('random_obstacle_spawner')

        self.declare_parameter('obstacle_num', 10)
        self.num_obstacles = self.get_parameter('obstacle_num').value

        latch_qos = QoSProfile(
            depth = 1,
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.ready_pub = self.create_publisher(Bool, '/obstacles_ready',latch_qos)

        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_entity service...')

        # self.spawn_goal_marker()
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
            req.robot_namespace = ''
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
            package_share_directory = get_package_share_directory('my_jackal_world')
            model_path = os.path.join(package_share_directory, 'models', 'cylinder.sdf')

            with open(model_path, 'r') as file:
                sdf_xml = file.read()

            obstacle_positions = []

            while len(obstacle_positions) < self.num_obstacles:
                rand_x = random.uniform(MIN_X, MAX_X)
                rand_y = random.uniform(MIN_Y, MAX_Y)
                candidate_pos = (rand_x, rand_y)

                if self.is_position_valid(candidate_pos, obstacle_positions):
                    obstacle_positions.append(candidate_pos)

                    req = SpawnEntity.Request()
                    req.name = f'obstacle_{len(obstacle_positions)}'
                    req.xml = sdf_xml
                    req.robot_namespace = ''
                    req.initial_pose.position.x = rand_x
                    req.initial_pose.position.y = rand_y
                    req.initial_pose.position.z = 0.0
                    req.initial_pose.orientation.w = 1.0

                    future = self.cli.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    self.get_logger().info(f"Spawned obstacle at ({rand_x:.2f}, {rand_y:.2f})")

            with open('/tmp/obstacle_positions.json', 'w') as f:
                json.dump(obstacle_positions,f)
            self.get_logger().info(f"Saved {len(obstacle_positions)} obstacle positions to /tmp/obstacle_positions.json")

            self.ready_pub.publish(Bool(data=True))
            self.get_logger().info("🔔 Published /obstacles_ready=True")

        except Exception as e:
            self.get_logger().error(f"Failed to spawn obstacles: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomObstacleSpawner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
