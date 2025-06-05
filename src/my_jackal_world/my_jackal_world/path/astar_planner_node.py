import rclpy, time
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from pathplanning.Search_based_Planning.Search_2D.Astar import AStar
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class AstarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.path_pub = self.create_publisher(Path, '/planned_path', 1)

        self.ready = False
        self.done  = False

        latch_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            Bool, '/obstacles_ready', self.cb_ready, latch_qos
        )

    def cb_ready(self, msg: Bool):
        self.get_logger().info(f"/obstacles_ready <= {msg.data}")
        if msg.data and not self.done:
            self.ready = True
            self.plan_once()

    def plan_once(self):
        if not self.ready:
            return
        
        start_grid = (0, 0)
        goal_grid  = (95, 95)

        astar = AStar(start_grid, goal_grid, "euclidean")

        t0 = time.time()
        grid_path, _ = astar.searching()
        self.get_logger().info(f"A* done in {(time.time()-t0)*1000:.1f} ms")

        if not grid_path:
            self.get_logger().info("No path found")
            return

        # Path 메시지 생성

        grid_path = grid_path[::-1][1:]
        self.get_logger().info(f"[GRID]{grid_path}")

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        res = astar.Env.resolution
        org = astar.Env.origin

        for gx, gy in grid_path:
            pose = PoseStamped()
            pose.pose.position.x = gx * res + org[0]
            pose.pose.position.y = gy * res + org[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path published: {len(path_msg.poses)} waypoints")
        self.done = True

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AstarPlanner())
    rclpy.shutdown()
