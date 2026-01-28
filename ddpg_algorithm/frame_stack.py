import numpy as np
from collections import deque
# 최근 4장의 사진을 스택 형태로 쌓아서 반환하는 로작
# 차량의 경우 단일 이미지로 파악하기 힘든 속도, 뱡향 등의 정보를 제공하기 위해 사용
class FrameStack:
    def __init__(self, k):
        self.k = k
        self.frames_img = deque([], maxlen=k)
        self.frames_sensor = deque([], maxlen=k)

    def reset(self, obs):
        self.frames_img.clear()
        self.frames_sensor.clear()
        for _ in range(self.k):
            self.frames_img.append(obs['image'])
            self.frames_sensor.append(obs['sensors'])
        return self._get_state()

    def step(self, obs):
        self.frames_img.append(obs['image'])
        self.frames_sensor.append(obs['sensors'])
        return self._get_state()

    def _get_state(self):
        # image: (k, H, W, 3) -> (H, W, 3 * k) or (k, H, W, 3)
        # Agent expects (Batch, 12, 64, 64). 
        # Here we return (k, 64, 64, 3) for image.
        # Sensors: (k, sensor_dim)
        return {
            'image': np.stack(self.frames_img, axis=0),
            'sensors': np.stack(self.frames_sensor, axis=0)
        }