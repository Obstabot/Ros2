import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_jackal_world.dynamics_obstacles.turtlebot_controller import TurtlebotController
# from my_jackal_world.dynamics_obstacles.turtlebot_2_controller import TurtlebotController as TB2
# from my_jackal_world.dynamics_obstacles.turtlebot_3_controller import TurtlebotController as TB3
# from my_jackal_world.dynamics_obstacles.turtlebot_4_controller import TurtlebotController as TB4
# from my_jackal_world.dynamics_obstacles.turtlebot_5_controller import TurtlebotController as TB5


def main():
  rclpy.init()

  # tb1 = TB1('turtlebot_1')
  # tb2 = TB2('turtlebot_2')
  # tb3 = TB3('turtlebot_3')
  # tb4 = TB4('turtlebot_4')
  # tb5 = TB5('turtlebot_5')

  # executor = MultiThreadedExecutor()
  # executor.add_node(tb1)
  # executor.add_node(tb2)
  # executor.add_node(tb3)
  # executor.add_node(tb4)
  # executor.add_node(tb5)

  # try:
  #   executor.spin()
  # finally:
  #   for node in [tb1, tb2, tb3, tb4, tb5]:
  #     node.destroy_node()
  #   rclpy.shutdown()

  bots = [TurtlebotController(f'turtlebot_{i}' for i in range(1,6))]
  executor = MultiThreadedExecutor()

  for bot in bots:
    executor.add_node(bot)

  try:
    executor.spin()
  finally:
    for bot in bots:
      bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()