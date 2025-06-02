# scan_to_grid.py
import math

def scan_to_obstacle_grids(scan_msg, robot_pose, scale):
    """
    Converts LaserScan to grid-based obstacle set
    :param scan_msg: sensor_msgs/LaserScan
    :param robot_pose: (x, y, yaw)
    :param scale: number of grid cells per meter
    :return: set of (grid_x, grid_y)
    """
    obs_set = set()
    angle = scan_msg.angle_min
    x, y, yaw = robot_pose

    for r in scan_msg.ranges:
        if scan_msg.range_min < r < scan_msg.range_max:
            obs_x = x + r * math.cos(yaw + angle)
            obs_y = y + r * math.sin(yaw + angle)
            gx = int(round(obs_x * scale))
            gy = int(round(obs_y * scale))
            obs_set.add((gx, gy))
        angle += scan_msg.angle_increment

    return obs_set
