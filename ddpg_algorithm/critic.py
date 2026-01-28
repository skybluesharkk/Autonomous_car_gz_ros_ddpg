# 액터와 동일한 흐름의 신경망을 구성하고, 중간 층에 액션도 같이 넣을 수 있도록 구성하자.
# 액터에 주석을 적은 부분들은 삭제 처리.
import torch
import torch.nn as nn
import torch.nn.functional as F

class Critic(torch.nn.Module):
    def __init__(self,action_size):
        super(Critic,self).__init__()
        # image 입력 먼저
        # image 입력 먼저
        self.conv1 = nn.Conv2d(12,32,kernel_size=8,stride=4)
        self.ln1 = nn.GroupNorm(1, 32)
        self.conv2 = nn.Conv2d(32,32,kernel_size=3,stride=2)
        self.ln2 = nn.GroupNorm(1, 32)
        self.conv3 = nn.Conv2d(32,32,kernel_size=3,stride=1)
        self.ln3 = nn.GroupNorm(1, 32)

        self.mlp1 = nn.Linear(512,400)
        self.ln_mlp1 = nn.LayerNorm(400)
        self.mlp2 = nn.Linear(400,300)
        # 논문에서 두가지 버전의 실험에 대해 액션 스페이스가 들어갈 층을 따로 구분하고 있음.
        # 저차원의 경우 : 두번째 은닉층에서 액션을 포함
        # 이미지의 경우 : fully-connected전까지 포함되면 안됨.
        # 이 실험에서는 저차원과 이미지 둘 다 합쳐서 하나의 상태로 활용하고 있고, 따라서 fc1에 세개를 함께 넣는 방식으로 구현하였음.
        self.fc1 = nn.Linear(1100+ action_size,200)  # image 800 + law_dim 300 + action 2
        self.fc2 = nn.Linear(200,200)   # 논문에서 200개의 유닛을 가진 fc로 구성되었다고함.
        self.ln_fc2 = nn.LayerNorm(200)
        self.value_output = nn.Linear(200,1) # 최종 q 값

    # [주석 처리] 논문 방식 초기화 - 우리 환경과 스케일이 안 맞아서 비활성화
    # def init_hidden(self, layer):
    #     if isinstance(layer, nn.Linear):
    #         f = layer.in_features
    #     elif isinstance(layer, nn.Conv2d):
    #         f = layer.in_channels * layer.kernel_size[0] * layer.kernel_size[1]
    #     elif isinstance(layer, (nn.BatchNorm2d, nn.BatchNorm1d, nn.GroupNorm, nn.LayerNorm)):
    #         return
    #     limit = 1 / (f**0.5)
    #     nn.init.uniform_(layer.weight, -limit, limit)
    #     if layer.bias is not None:
    #         nn.init.uniform_(layer.bias, -limit, limit)

    def reset_parameters(self):
        # PyTorch 기본 초기화 사용 (Kaiming uniform - 더 안정적)
        pass
        # [주석 처리] 논문 방식 초기화
        # self.init_hidden(self.conv1)
        # self.init_hidden(self.conv2)
        # self.init_hidden(self.conv3)
        # self.init_hidden(self.mlp1)
        # self.init_hidden(self.mlp2)
        # self.init_hidden(self.fc1)
        # self.init_hidden(self.fc2)
        # nn.init.uniform_(self.value_output.weight, -3e-3, 3e-3)
        # nn.init.uniform_(self.value_output.bias, -3e-3, 3e-3)

    def forward(self,img,sensor,action):
        x = F.relu(self.ln1(self.conv1(img)))
        x = F.relu(self.ln2(self.conv2(x)))
        x = F.relu(self.ln3(self.conv3(x)))
        x = torch.flatten(x,1) # 하나로 펴기. 800 차원

        y = F.relu(self.ln_mlp1(self.mlp1(sensor)))
        y = F.relu(self.mlp2(y)) # 300차원

        combined_input = torch.cat([x,y,action],dim=1) # 옆으로 붙여서 1100차원 만들기 + action
        z = F.relu(self.fc1(combined_input))
        z = F.relu(self.ln_fc2(self.fc2(z)))
        z = self.value_output(z) # q값을 제공해야 하기 때문에 별도의 활성화 함수 사용 안함.

        return z

