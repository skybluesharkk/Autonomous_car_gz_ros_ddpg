import numpy as np
# ou noise를 통해 탐험의 역할을 수행할 수 있도록 함.
# 들어가는 값들은 모두 논문에 있는 값들을 그대로 사용, 시간 단위는 1로 사용.
class OuNoise():
    def __init__(self,action_size=2,mu=0.0,theta=0.15,sigma=0.2,dt=1): # 평균은 0.0으로 설정
        self.action_size = action_size
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.dt = dt # 수식에 있는 시간 단위, 1로 단위를 설정함. 파이썬 코드 상에서 1스텝이 1이라고 둬도 될 것 같아서,,
        self.reset()

    def reset(self):
        self.current_noise = np.ones(self.action_size)*self.mu # mu평균에 맞춰서 초기화시키기.

    def sample(self):
        dx = self.theta * (self.mu - self.current_noise)*self.dt +self.sigma*(np.random.randn(self.action_size)) # 무작위성을 더해야 하는데, 평균 0 , 표준편차 1인 분포에서 내 액션의 차원에 맞게 무작위숫자 뽑기.
        self.current_noise += dx
        return self.current_noise
