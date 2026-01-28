import random
import numpy as np
# 논문에 적힌 트랜지션 순서('state','action','reward','next_state')) 
# 기존에는 네임드 튜플과 deque를 활용하여 작성하였었으나 해당 방식은 연산 속도가 많이 느리다고 하여서 이번에는 수정하여 진행하였음.

class ReplayBuffer():
    def __init__(self,max_size,batch_size,image_shape,sensors_dim,action_dim):
        self.max_size = max_size
        self.batch_size = batch_size
        self.state_img = np.zeros((max_size,*image_shape),dtype=np.float32)
        self.state_sensor = np.zeros((max_size,*sensors_dim),dtype=np.float32)
        self.action = np.zeros((max_size,*action_dim),dtype=np.float32)
        self.reward = np.zeros((max_size,1),dtype=np.float32)
        self.next_state_img = np.zeros((max_size,*image_shape),dtype=np.float32)
        self.next_state_sensors = np.zeros((max_size,*sensors_dim),dtype=np.float32)
        self.done = np.zeros((max_size,1),dtype=np.float32)
        self.cnt=0

    def push(self,state,action, reward,next_state,done):
        index = self.cnt % self.max_size # deque로 구현한게 아니기 때문에 선입선출을 구현하기 위해 max_size를 넘어가면 나머지 연산으로 앞에꺼부터 다시 덮어줘야 한다.
        self.state_img[index] = state['image']
        self.state_sensor[index] = state['sensors']
        self.action[index] = action
        self.reward[index] = reward
        self.next_state_img[index] = next_state['image']
        self.next_state_sensors[index] = next_state['sensors']
        self.done[index] = done

        self.cnt+=1
        
    def sample(self):
        max_choice_len = min(self.cnt,self.max_size)# 넣을때마다 카운트를 올린다고 할 때, 내가 몇 개까지 중에 뽑을 수 있는지.
        batch = np.random.choice(max_choice_len,self.batch_size,replace=False ) # replace=False하면 중복 없이 뽑기 가능, 범위 안에서 배치사이즈만큼 [2,3,6]같은 형식으로 나옴.
        return ( # [batch] 로 하면 (64,64,3) 짜리가 예를들어 128이면 (128,64,64,3)과 같은 식으로 한번에 나옴.
            self.state_img[batch],
            self.state_sensor[batch],
            self.action[batch],
            self.reward[batch],
            self.next_state_img[batch],
            self.next_state_sensors[batch],
            self.done[batch]
        )