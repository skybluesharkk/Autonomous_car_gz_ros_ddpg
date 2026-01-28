import torch
import torch.nn as nn
import torch.nn.functional as F

# 액터 신경망 구현. 액터는 이미지와 센서값을 각각 다른 신경망으로 받은 다음에 하나로 합쳐서 액션을 뽑아내야함.
class Actor(torch.nn.Module):
    def __init__(self):
        super(Actor,self).__init__()
        # image 입력 먼저
        # torch.nn.Conv2d(in_channels, out_channels, kernel_size, stride=1, padding=0, dilation=1, groups=1, bias=True, padding_mode='zeros', device=None, dtype=None)[source]

        self.conv1 = nn.Conv2d(12,32,kernel_size=8,stride=4)
        self.ln1 = nn.GroupNorm(1, 32)
        self.conv2 = nn.Conv2d(32,32,kernel_size=3,stride=2)
        self.ln2 = nn.GroupNorm(1, 32)
        self.conv3 = nn.Conv2d(32,32,kernel_size=3,stride=1)
        self.ln3 = nn.GroupNorm(1, 32)

        # linear input size 32*5*5 = 800 # 직접 계산한 이미지를 flatten한 값.
        # 논문에서 실험한 숫자들을 그대로 따라서 400,300의 유닛을 가지도록 하며, 입력 값은 내가 정의한 센서값 128 * 4프레임 = 512임.
        self.mlp1 = nn.Linear(512,400)
        self.ln_mlp1 = nn.LayerNorm(400)
        self.mlp2 = nn.Linear(400,300)
        # mlp2 다음은 concat이라 바로 BN 적용이 애매할 수 있지만, 활성화 함수 전에는 보통 넣음.
        
        self.fc1 = nn.Linear(1100,200)  # image크기800 + 센서 크기 300 마지막에 액션 정하기 전에는 합치기.
        self.ln_fc1 = nn.LayerNorm(200)
        self.fc2 = nn.Linear(200,200)   # 논문에서 200개의 유닛을 가진 fc로 구성되었다고함.
        self.ln_fc2 = nn.LayerNorm(200)
        self.action_output = nn.Linear(200,2)

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
        # nn.init.uniform_(self.action_output.weight, -3e-3, 3e-3)
        # nn.init.uniform_(self.action_output.bias, -3e-3, 3e-3)

    def forward(self, img, sensor):
        # 1. Conv (이미지 처리)
        x = F.relu(self.ln1(self.conv1(img)))
        x = F.relu(self.ln2(self.conv2(x)))
        x = F.relu(self.ln3(self.conv3(x)))
        x = torch.flatten(x, 1)

        # 2. Sensor MLP (센서 처리)
        y = F.relu(self.ln_mlp1(self.mlp1(sensor)))
        y = F.relu(self.mlp2(y))

        # 3. Combine (합치기)
        combined = torch.cat([x, y], dim=1)
        z = F.relu(self.ln_fc1(self.fc1(combined)))
        z = F.relu(self.ln_fc2(self.fc2(z)))
        
        out = self.action_output(z)
        action = torch.tanh(out)
        
        steering = action[:, 0] * 0.6
        velocity = action[:, 1]
        
        return torch.stack([steering, velocity], dim=1)
