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

        def interpolate_path(path, resolution=0.05):
            from math import sqrt
            interpolated = []
            for i in range(len(path)-1):
                x0, y0 = path[i]
                x1, y1 = path[i + 1]
                dx, dy = x1 - x0, y1 - y0
                dist = sqrt(dx**2+dy**2)
                steps = max(1, int(dist / resolution))
                for j in range(steps):
                    ratio = j/steps
                    x = x0 + ratio * dx
                    y = y0 + ratio * dy
                    interpolated.append((x,y))
            interpolated.append(path[-1])
            return interpolated
        
        smooth_path = interpolate_path(grid_path, resolution=0.1)

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        res = astar.Env.resolution
        org = astar.Env.origin

        for gx, gy in smooth_path:
            pose = PoseStamped()
            pose.pose.position.x = gx * res + org[0] + 0.05
            pose.pose.position.y = gy * res + org[1] + 0.05
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path published: {len(path_msg.poses)} waypoints")
        self.done = True

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AstarPlanner())
    rclpy.shutdown()
