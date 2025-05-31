"""
Env 2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = 51  # size of background
        self.y_range = 31
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
      import json
      raw_obs = set()
      try:
        with open('/tmp/obstacle_positions.json', 'r') as f:
          obstacle_centers = json.load(f)
          print(f"Loaded {len(obstacle_centers)} obstacle centers from file")
      except FileNotFoundError:
        print("[WARNING] No obstacle file found. Using empty map.")
        return set()

      obstacle_radius = 0.2
      buffer = 0.1
      resolution = 0.1
      scale = int(1/resolution)

      for ox, oy in obstacle_centers:
        ox_i, oy_i = int(round(ox*scale)), int(round(oy*scale))
        range_cells = int((obstacle_radius + buffer)*scale)

        for dx in range(-range_cells, range_cells + 1):
          for dy in range(-range_cells, range_cells + 1):
            raw_obs.add((ox_i + dx, oy_i + dy))

      print(f"[DEBUG] Total {len(raw_obs)} obstacle grid cells generated.")

      return raw_obs