# env.py 의 맨 위쯤에 추가
import os
print(f"[DEBUG] *** 사용 중인 env.py 경로: {os.path.abspath(__file__)}")


class Env:
    def __init__(self, origin=(0.0, 0.0), resolution=0.05):
        self.origin = origin          # (x0, y0) 실제 좌표에서 그리드 (0,0)이 될 위치
        self.resolution = resolution  # 1 grid → 실제 몇 m
        self.x_range = int(10.0 / resolution)             # 격자 맵 크기
        self.y_range = int(10.0 / resolution)
        self.motions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,1),(1,-1),(-1,1)]
        # self.motions = [(-1,0), (1,0), (0,-1), (0,1)]
        self.obs = self.obs_map()     # 셋으로 저장

    def update_obs(self, obs_set):
        """외부에서 장애물 set을 다시 넣어줄 때 사용"""
        self.obs = obs_set

    def obs_map(self):
        """
        /tmp/obstacle_positions.json 파일을 읽어
        실수 좌표 → 그리드 좌표(set) 로 변환
        """
        import json, os

        json_file = "/tmp/obstacle_positions.json"
        if not os.path.exists(json_file):
            print(f"[Env] JSON not found: {json_file}")
            return set()                       # 빈 맵 반환

        with open(json_file) as f:
            raw = json.load(f)                 # [[x, y], …]  (실수[m])

        obs_set = set()

        inflation = int(0.25 / self.resolution)
        for x_real, y_real in raw:
            gx = round((x_real - self.origin[0]) / self.resolution)
            gy = round((y_real - self.origin[1]) / self.resolution)
            for dx in range(-inflation, inflation+1):
                for dy in range(-inflation, inflation+1):
                    nx, ny = gx+dx, gy+dy
            # 맵 범위 안만 추가
                    if 0<=nx<self.x_range and 0<=ny<self.y_range:
                        obs_set.add((nx, ny))
        
        obs_set = Env.remove_narrow_passages(obs_set, self.resolution, min_clearance_m=0.7)

        print(f"[Env] Loaded {len(obs_set)} obstacles from JSON")
        return obs_set
    
    def inflate_obstacles(obs_set, inflation=1):
        inflated = set()
        for x,y in obs_set:
            for dx in range(-inflation, inflation+1):
                for dy in range(-inflation, inflation+1):
                    inflated.add((x+dx, y+dy))
        return inflated

    @staticmethod
    def remove_narrow_passages(obs_set, resolution, min_clearance_m=0.7):
        """장애물 간격이 너무 좁은 구역에 추가 장애물 삽입"""
        min_dist = int(min_clearance_m / resolution)  # grid 단위 거리
        blocked = set()

        for ox, oy in obs_set:
            for dx in range(-min_dist, min_dist + 1):
                for dy in range(-min_dist, min_dist + 1):
                    if (dx, dy) == (0, 0):
                        continue
                    if (ox + dx, oy + dy) in obs_set:
                        # 두 장애물이 너무 가까우면 그 중간을 막는다
                        mx = (ox + ox + dx) // 2
                        my = (oy + oy + dy) // 2
                        blocked.add((mx, my))

        return obs_set | blocked
