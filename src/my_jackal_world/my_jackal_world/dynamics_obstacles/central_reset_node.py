import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from my_jackal_world.msg import ResetOrientation
from rclpy.qos import QoSProfile
from collections import deque
import math

class CentralResetService(Node):
    def __init__(self):
        super().__init__('central_reset_service')
        self.queue = deque()
        self.processing = False

        self.create_subscription(
            ResetOrientation,
            '/reset_orientation',
            self.queue_request,
            QoSProfile(depth=10)
        )

        self.client = self.create_client(SetModelState, '/set_model_state')

        self.timer = self.create_timer(0.2, self.process_queue)

    def queue_request(self, msg):
        self.queue.append(msg)
        self.get_logger().info(f'📩 Queued reset for {msg.robot_name}')

    def process_queue(self):
        if not self.queue or self.processing or not self.client.service_is_ready():
            return

        msg = self.queue.popleft()
        self.processing = True

        req = SetModelState.Request()
        state = ModelState()
        state.model_name = msg.robot_name
        state.pose.position.x = msg.x
        state.pose.position.y = msg.y
        state.pose.position.z = 0.0

        qz = math.sin(msg.yaw / 2)
        qw = math.cos(msg.yaw / 2)

        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw

        state.twist.linear.x = 0.0
        state.twist.angular.z = 0.0
        state.reference_frame = 'world'

        req.model_state = state
        future = self.client.call_async(req)
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('✅ Model reset successfully.')
            else:
                self.get_logger().warn(f'⚠️ Model reset failed: {result.status_message}')
        except Exception as e:
            self.get_logger().error(f'❌ Service call failed: {str(e)}')
        self.processing = False

def main(args=None):
    rclpy.init(args=args)
    node = CentralResetService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
