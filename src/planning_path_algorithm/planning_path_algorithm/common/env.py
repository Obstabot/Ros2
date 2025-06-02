# env.py
class Env:
    def __init__(self):
        self.x_range = 100  # arbitrary large map
        self.y_range = 100
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = set()

    def update_obs(self, obs):
        self.obs = obs
