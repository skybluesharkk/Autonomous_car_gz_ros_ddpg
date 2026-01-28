from actor import Actor
from critic import Critic
from frame_stack import FrameStack
from ou_noise import OuNoise
from replay_buffer import ReplayBuffer
from rclpy.node import Node 
import time
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

# [디버깅] NaN 발생 위치 추적
torch.autograd.set_detect_anomaly(True)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class DDPGAgent(Node): # 실험에 필요한 하이퍼 파라미터 값 한군데에서 관리할 것이기 때문에 노드로 선언함.
    def __init__(self):
        super().__init__('ddpg_agent')
        self.declare_parameter('actor_lr', 1e-6)  # 1e-5 -> 1e-6 (NaN 방지)
        self.declare_parameter('critic_lr', 1e-5)  # 5e-5 -> 1e-5 (NaN 방지)
        self.declare_parameter('critic_l2_decay', 1e-4)
        self.declare_parameter('gamma', 0.99)
        self.declare_parameter('tau', 1e-3)
        self.declare_parameter('theta', 0.15)
        self.declare_parameter('sigma', 0.2)
        self.declare_parameter('mu', 0.0)
        self.declare_parameter('batch_size', 64)
        self.declare_parameter('max_buffer_size', 100000)

        self.actor_lr = self.get_parameter('actor_lr').value
        self.critic_lr = self.get_parameter('critic_lr').value
        self.critic_l2_decay = self.get_parameter('critic_l2_decay').value
        self.gamma = self.get_parameter('gamma').value
        self.tau = self.get_parameter('tau').value
        self.theta = self.get_parameter('theta').value
        self.sigma = self.get_parameter('sigma').value
        self.mu = self.get_parameter('mu').value
        self.batch_size = self.get_parameter('batch_size').value
        self.max_buffer_size = self.get_parameter('max_buffer_size').value

        # Actor 와 Actor_target 망 선언
        # 초기화를 먼저하고 해당 가중치를 똑같이 로드해서 사용한다.
        self.Actor = Actor().to(device)
        self.Actor_target = Actor().to(device)
        self.Actor.reset_parameters()
        self.Actor_target.load_state_dict(self.Actor.state_dict())

        # Critic도 마찬가지로 작업해줌.
        self.Critic = Critic(2).to(device)
        self.Critic_target = Critic(2).to(device)
        self.Critic.reset_parameters()
        self.Critic_target.load_state_dict(self.Critic.state_dict())
        
        # 논문에 명시된 대로 아담 옵티마이저를 적절한 학습률로 사용
        self.actor_optimizer = torch.optim.Adam(self.Actor.parameters(), lr=self.actor_lr)
        self.critic_optimizer = torch.optim.Adam(self.Critic.parameters(), lr=self.critic_lr, weight_decay=self.critic_l2_decay)

        self.OU = OuNoise()
        # 센서 데이터도 4프레임 스택이므로 (4, 128) 로 변경
        self.replay_buffer = ReplayBuffer(self.max_buffer_size, self.batch_size, (4, 64, 64, 3), (4, 128), (2,))
        
        self.check_nan_weights()

    def check_nan_weights(self):
        print("--- Checking Weights for NaNs ---")
        for name, param in self.Actor.named_parameters():
            if torch.isnan(param).any():
                print(f"CRITICAL: Actor param {name} is NaN!")
        for name, param in self.Critic.named_parameters():
            if torch.isnan(param).any():
                print(f"CRITICAL: Critic param {name} is NaN!")
        print("--- Weight Check Complete ---")

    def get_action(self, state):
        # 이미지 전처리: (4, 64, 64, 3) -> (1, 12, 64, 64)
        # Permute로 (4, 3, 64, 64) 형태로 변경 -> Reshape으로 시간 축과 채널 축을 합침 (12, 64, 64) -> Unsqueeze로 배치 차원 추가 (1, 12, 64, 64)
        state_img = torch.FloatTensor(state['image']).permute(0, 3, 1, 2).reshape(12, 64, 64).unsqueeze(0).to(device)
        
        # 이미지 데이터 디버그
        if torch.isnan(state_img).any():
             print("CRITICAL: Input Image has NaN!")
             print(state_img)
        
        # 센서 데이터 전처리: (4, 128) -> (512,) -> (1, 512)
        
        # 센서 데이터 전처리: (4, 128) -> (512,) -> (1, 512)
        # 4개의 프레임을 모두 펼쳐서 입력으로 사용
        state_sensor = torch.FloatTensor(state['sensors']).flatten().unsqueeze(0).to(device)

        # 학습 안하고 값만 뱉게.
        self.Actor.eval() # BatchNorm 오류 방지 (Batch size 1일 때 eval 모드 필수)
        if np.isnan(state['sensors']).any():
               print("STATE HAS NaN")
        with torch.no_grad():
            action = self.Actor(state_img, state_sensor)
        if torch.isnan(action).any():
            print("ACTOR OUTPUT NaN")
        self.Actor.train() # 다시 학습 모드로 복구
        
        # action은 gpu에 올리고 연산했고, ou는 cpu에 있으니까 다시 내려서 더하기.
        raw_action = action.cpu().numpy()[0] 
        action_with_noise = raw_action + self.OU.sample()
        
        steering = np.clip(action_with_noise[0], -0.6, 0.6)
        throttle = np.clip(action_with_noise[1], -1.0, 1.0)
        
        final_action = np.array([steering, throttle])
        return final_action

    def soft_update_target(self):
        for target_param, online_param in zip(self.Actor_target.parameters(), self.Actor.parameters()):
            target_param.data.copy_(self.tau * online_param.data + (1 - self.tau) * target_param.data)
        for target_param, online_param in zip(self.Critic_target.parameters(), self.Critic.parameters()):
            target_param.data.copy_(self.tau * online_param.data + (1 - self.tau) * target_param.data)
 

    def train_model(self):
        # replay memory에서 샘플 뽑기
        state_img, state_sensor, action, reward, next_state_img, next_state_sensors, done = self.replay_buffer.sample()

        # numpy 배열로 뽑은 값들 텐서로 변환하고 gpu로 보내기
        # (Batch, 4, 64, 64, 3) -> (Batch, 12, 64, 64) 로 변환
        state_img = torch.FloatTensor(state_img).permute(0, 1, 4, 2, 3).reshape(-1, 12, 64, 64).to(device)
        
        # (Batch, 4, 128) -> (Batch, 512) 로 변환 (Flatten)
        state_sensor = torch.FloatTensor(state_sensor).reshape(-1, 512).to(device)
        
        action = torch.FloatTensor(action).to(device)

        # reward랑 done은 실수 하나만 있기 때문에 차원을 맞춰주기. (Batch,1)
        reward = torch.FloatTensor(reward).to(device)
        done = torch.FloatTensor(done).to(device)

        # (Batch, 4, 64, 64, 3) -> (Batch, 12, 64, 64) 로 변환
        next_state_img = torch.FloatTensor(next_state_img).permute(0, 1, 4, 2, 3).reshape(-1, 12, 64, 64).to(device)
        
        # (Batch, 4, 128) -> (Batch, 512) 로 변환 (Flatten)
        next_state_sensors = torch.FloatTensor(next_state_sensors).reshape(-1, 512).to(device)

        # [안전장치] 입력 데이터에 NaN이 있으면 학습 스킵
        if (torch.isnan(state_img).any() or torch.isnan(state_sensor).any() or 
            torch.isnan(next_state_img).any() or torch.isnan(next_state_sensors).any()):
            print("WARNING: NaN in input data, skipping train step")
            return 0.0, 0.0, 0.0

        with torch.no_grad(): # 학습이 아닌 계산만을 하는 과정인데, 이 문구가 없으면 파이토치가 미분을 위해 메모리에 저장하게 되며 메모리가 낭비됨
            next_actions = self.Actor_target(next_state_img, next_state_sensors)
            next_q = self.Critic_target(next_state_img, next_state_sensors, next_actions)
            target_q = reward + (1 - done) * self.gamma * next_q # 만약 마지막 스텝이면 그 다음은 없으니까.
            
            target_q = torch.clamp(target_q, -50.0, 50.0)
        
        # Online Critic으로 구하는 지금 상태의 Q 값
        q = self.Critic(state_img, state_sensor, action)

        # 논문에 있는 MSEloss 대신 안정적인 Huber Loss 사용
        criterion = nn.SmoothL1Loss()
        critic_loss = criterion(q, target_q)

        # [안전장치] loss가 NaN이면 학습 스킵 및 경고
        if torch.isnan(critic_loss):
            print("CRITICAL: Critic loss is NaN! Skipping update.")
            return 0.0, 0.0, 0.0

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        # Gradient Clipping (NaN 방지) - 매우 보수적
        torch.nn.utils.clip_grad_norm_(self.Critic.parameters(), max_norm=0.01)
        self.critic_optimizer.step()

        # q 값과 action을 사용하여 업데이트
        # 이전에 버퍼에 저장했던 액션들은 이미 OU noise가 같이 불어서 저장된 것들이기 때문에 다시 해줘야 함.
        action_without_noise = self.Actor(state_img, state_sensor) 
        # ddpg는 q값을 최대화해야하고, 이는 loss를 최소화해야 하는거니까 앞에 - 붙임.
        actor_loss = -self.Critic(state_img, state_sensor, action_without_noise).mean()

        # [안전장치] actor loss가 NaN이면 학습 스킵
        if torch.isnan(actor_loss):
            print("CRITICAL: Actor loss is NaN! Skipping update.")
            return 0.0, critic_loss.item(), q.mean().item()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        # Gradient Clipping (NaN 방지) - 매우 보수적
        torch.nn.utils.clip_grad_norm_(self.Actor.parameters(), max_norm=0.01)
        self.actor_optimizer.step()

        # [안전장치] 학습 후 가중치에 NaN이 있으면 경고
        for name, param in self.Actor.named_parameters():
            if torch.isnan(param).any():
                print(f"CRITICAL: Actor {name} became NaN after update!")
                break

        # soft update 까지라고 마무리.
        self.soft_update_target()

        return actor_loss.item(), critic_loss.item(), q.mean().item()
        

    def save_model(self, path='./ddpg_model'):
        import os
        # 저장 경로의 디렉토리가 없으면 생성 (RuntimeError 방지)
        directory = os.path.dirname(path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
            
        current_time = time.strftime('%Y%m%d_%H%M%S')

        torch.save({
            'Actor': self.Actor.state_dict(),
            'Critic': self.Critic.state_dict(),
            'Actor_target': self.Actor_target.state_dict(),
            'Critic_target': self.Critic_target.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict()
        }, path+'_'+current_time+'_.pth')

    def record_check_point(self, path='./ddpg_model', episode=0):
        # 특정 에피소드때 마다 체크포인트 중간 저장
        import os
        # 저장 경로의 디렉토리가 없으면 생성 (RuntimeError 방지)
        directory = os.path.dirname(path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
            
        current_time = time.strftime('%Y%m%d_%H%M%S')
        filename = f"{path}_{current_time}_ep_{episode}.pth"
        torch.save({
            'Actor': self.Actor.state_dict(),
            'Critic': self.Critic.state_dict(),
            'Actor_target': self.Actor_target.state_dict(),
            'Critic_target': self.Critic_target.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict()
        }, filename)
