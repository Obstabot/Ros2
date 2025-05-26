import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
import math
import sys

MIN_X, MAX_X = 0.5, 9.5
SPEED = 1.0

class TurtlebotController(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_controller')
        self.robot_name = robot_name
        self.robot_speed = SPEED if int(robot_name[-1]) <= 2 else -SPEED
        self.model_position = None

        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.reset_pub = self.create_publisher(String, '/reset_cmd', 10)
        self.create_subscription(ModelStates, '/model_states', self.state_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def state_callback(self, msg):
        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            x = msg.pose[idx].position.x
            y = msg.pose[idx].position.y
            self.model_position = (x, y)

    def publish_velocity(self):
        if self.model_position is None:
            return

        x, y = self.model_position

        if self.robot_speed > 0 and x >= MAX_X:
            self.robot_speed *= -1
            self.reset_orientation(MAX_X, y, math.pi)
            return

        if self.robot_speed < 0 and x <= MIN_X:
            self.robot_speed *= -1
            self.reset_orientation(MIN_X, y, 0.0)
            return

        twist = Twist()
        twist.linear.x = self.robot_speed
        self.cmd_vel_pub.publish(twist)

    def reset_orientation(self, x, y, yaw):
        msg = String()
        msg.data = f"{self.robot_name} {x:.2f} {y:.2f} {yaw:.2f}"
        self.reset_pub.publish(msg)
        self.get_logger().info(f"📤 Reset published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("❌ 로봇 이름이 필요합니다. 예: turtlebot_1")
        return
    robot_name = sys.argv[1]
    node = TurtlebotController(robot_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
