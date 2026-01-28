from geometry_msgs.msg import PointStamped # 타임스탬프가 포함된 3D 좌표 메시지 형식을 가져옴
from geometry_msgs.msg import Twist # 선속도와 각속도를 제어하기 위한 메시지 형식을 가져옴

import rclpy # ROS2의 파이썬 클라이언트 라이브러리를 가져옴
from rclpy.node import Node # Node작성을 위한 기본 클래스를 가져옴

from custom_interfaces.srv import Reset 
# ros2 topic info <topic_name> 을 통해 확인한 타입을 활용
from std_msgs.msg import String 
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from rosgraph_msgs.msg import Clock
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import Log
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Contacts

import math
import cv2
import numpy as np
import random


MAP_WALL_SIZE = {'width':2,'height':28}
RADIUS = 1.5

class respawn_pose_node:
    def __init__(self,map_w,map_h):

        self.MAP_WIDTH = map_w
        self.MAP_HEIGHT = map_h

        self.info_of_obstacles = [
            # 로봇의 중심점 x,y 그 다음 각 축별로 변화량 (장애물을 크게 만들것이니 RADIUS 값을 더해줘야 함)
            # dx = w/2 + R, dy = h/2 + R ros의 좌표가 x,y가 반대기는 한데, 수식으로 정리하면 이런 느낌
            {'x': 7.5,'y': 7.5, 'dx': 1.0 + RADIUS, 'dy':1.0 + RADIUS }, # box1
            {'x': 7.0,'y': -10.0, 'dx': 1.0 + RADIUS, 'dy':1.0 + RADIUS }, # box2
            {'x': -4.0,'y': 0, 'dx': 1.0 + RADIUS, 'dy':1.0 + RADIUS }, # box3
            {'x': -4.0,'y': -7.0, 'dx': 1.0 + RADIUS, 'dy':1.0 + RADIUS }, # box4
            {'x': -8.0,'y': 8.0, 'dx': 1.0 + RADIUS, 'dy':1.0 + RADIUS }, # box5
            {'x': 5.0,'y': -3.0, 'dx': 0.5 + RADIUS, 'dy':4.0 + RADIUS }, # wall1
            {'x': 0.0,'y': 3.0, 'dx': 0.5 + RADIUS, 'dy':4.0 + RADIUS }, # wall2
        ]

    def get_safe_pose(self,use_random=True):
        d_map_wall_size = MAP_WALL_SIZE['width'] / 2.0 # map 구성 시 바깥 가장자리 벽들의 중심이 월드맵의 선에 걸쳐있기 때문에 안쪽으로 절반만큼 겹침
        width_limit = self.MAP_WIDTH / 2.0 - d_map_wall_size - RADIUS # 추가로 벽 안쪽에 바로 중심 점이 소환될 시 바깥 벽이랑 겹치기 때문에 반지름만큼 더 뺴줌
        height_limit = self.MAP_HEIGHT / 2.0 - d_map_wall_size - RADIUS

        if use_random == False:
            return (-10.0,0.0,0.0) # 기본 소환 좌표. 만약 시연등의 사유로 랜덤 좌표 안할때
        
        while True:
            x = random.uniform(-width_limit, +width_limit)
            y = random.uniform(-height_limit, +height_limit)

            conflict = False 
            for obstacle in self.info_of_obstacles:
                if abs(x - obstacle['x']) < obstacle['dx'] and abs(y - obstacle['y']) < obstacle['dy'] :
                    conflict = True # 목록에 있는 장애물 중 하나와 겹친다고 판단
                    break # for 문 나가서 다시 만들기
            # 겹치는 장애물이 없다면
            if conflict == False:
                # 목표지점과 너무 가까이 생성되는것을 피하기 위해 아래 로직을 추가함. 3m기준.
                if math.hypot(x - 10.0, y - 0.0) < 3.0:
                    continue

                return (x,y,0.05) # 바퀴의 크기를 생각했을때 지면과 딱 붙여도 상관없을 것 같은데 혹시 몰라서 조금 띄움

class EnvNode(Node):

    def __init__(self):
        # 노드 이름 정의하기
        super().__init__('EnvNode')
        
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('img_width', 64)
        self.declare_parameter('img_height', 64)
        self.declare_parameter('map_width', 30.0)
        self.declare_parameter('map_height', 30.0)
        self.declare_parameter('max_step', 1000)
        self.declare_parameter('max_episode', 1000)
        self.MAX_RANGE = self.get_parameter('max_range').value
        self.IMG_WIDTH = self.get_parameter('img_width').value
        self.IMG_HEIGHT = self.get_parameter('img_height').value
        self.MAP_WIDTH = self.get_parameter('map_width').value
        self.MAP_HEIGHT = self.get_parameter('map_height').value
        self.MAX_STEP = self.get_parameter('max_step').value
        self.MAX_EPISODE = self.get_parameter('max_episode').value
        # 인스턴스화 해서 사용
        self.respawn_generator = respawn_pose_node(self.MAP_WIDTH,self.MAP_HEIGHT)

        self.cli_env_reset = self.create_client( Reset, 'reset') # reset 해주는 서비스 들어가야함 reset_interface라는 양식으로 reset에게 주문해야함.
        # 아래는 파이썬 환경이 구독해와야 할 토픽 목록임. 각각에 대해 구독자를 생성하면 될 듯 기본적으로 1개만 저장, 충돌만 중요하니까 10개 저장
        # LiDAR
        self.sub_lidar = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 1) # 어떤 형식으로,어떤이름으로 받아서, 뭐를 실행할거고, 몇개까지 큐에 저장할건지
        # camera
        self.sub_camera = self.create_subscription(Image,'/camera',self.camera_callback,1)
        # camera_info
        # self.sub_camera_info = self.create_subscription(CameraInfo,'/camera_info',self.camera_info_callback,10)
        # 학습에는 필요 없고 추후 foxglove에서 시각화할 때만 필요해서 주석처리함. 브릿지를 통해서 뿌리기는 계속 뿌림.
        # imu
        self.sub_imu = self.create_subscription(Imu,'/imu',self.imu_callback,1)
        # collision (차체, 카메라, 타이어 4개 각각 별도 콜백으로 구독)
        self.sub_collision_chassis = self.create_subscription(Contacts, '/collision_chassis', self.collision_chassis_callback, 10)
        self.sub_collision_camera = self.create_subscription(Contacts, '/collision_camera', self.collision_camera_callback, 10)
        self.sub_collision_tire_lf = self.create_subscription(Contacts, '/collision_tire_lf', self.collision_tire_lf_callback, 10)
        self.sub_collision_tire_rf = self.create_subscription(Contacts, '/collision_tire_rf', self.collision_tire_rf_callback, 10)
        self.sub_collision_tire_lr = self.create_subscription(Contacts, '/collision_tire_lr', self.collision_tire_lr_callback, 10)
        self.sub_collision_tire_rr = self.create_subscription(Contacts, '/collision_tire_rr', self.collision_tire_rr_callback, 10)
        # model/car/odometry -> 신경망에 입력으로 넣어야 하는 값, 추정된 좌표
        self.sub_model_car_odometry = self.create_subscription(Odometry,'/model/car/odometry',self.model_car_odometry_callback,1)
        # model/car/tf 값은 사용안함. 직접 찍어보니 odometry랑 중복된 값임
        # self.sub_model_car_tf = self.create_subscription(TFMessage,'/model/car/tf',self.model_car_tf_callback,10)
        # /world/my_car_world/pose/info -> 실제 보상함수에서 '목표 지점과 얼마나 가까운지' 계산을 위해 사용되는 값.
        self.sub_world_pose = self.create_subscription(TFMessage,'/world/my_car_world/pose/info',self.world_car_pose_callback,1)

        # 발행자 만들기
        # /cmd_vel (기존 /cmd/vel 오타 수정)
        self.pub_cmd_vel = self.create_publisher(Twist,'/cmd_vel',10)

        self.req = Reset.Request() # service server에 보낼 리퀘스트를 미리 만들어둠
        # goal 위치 지점
        self.goal_position = [10,0,0]
        # 최신 라이다 값 - callback에서 계산해서 저장
        self.latest_lidar = None
        self.latest_image = None
        self.latest_imu   = None
        self.collision_flag = 0.0
        self.collision_sensor = None  # 어떤 센서에서 충돌했는지 저장
        self.is_arrived_at_goal = 0.0
        
        # Stuck 감지 변수
        self.stuck_count = 0
        self.STUCK_THRESHOLD = 20 # 2초 (20 * 0.1s) 동안 못 움직이면 Stuck 판정
        
        self.latest_odometry_info = None
        self.world_car_pose = None # world 기준으로 차가 지금 어디 있는 지 현재 위치
        self.prev_dist = None
        self.steps_done = 0

    def lidar_callback(self,msg):
        # inf라고 나오는 거는 최대 거리 10으로 설정해야 함.
        all_lidar_data = msg.ranges
        num_of_lidar = len(all_lidar_data)
        preprocessed_lidar_data = [0.0] * num_of_lidar

        # inf 로 나오는 것들 먼저 최대 거리 상수 10으로 바꾸기
        for i in range(num_of_lidar): 
            if math.isinf(all_lidar_data[i]):
                preprocessed_lidar_data[i] = self.MAX_RANGE
            else:
                preprocessed_lidar_data[i] = all_lidar_data[i]

        # 1도 당 하나의 데이터로 360도가 들어오는데, 이는 연산량이 너무 많아질 것 같아서 3도 당 1개로 다운샘플링 수행
        compressed_num_of_lidars = int(num_of_lidar/ 3)
        compressed_lidars_data = [0.0] * compressed_num_of_lidars

        # 3도 중에서 제일 작은 값으로 해서 장애물과 가까운 거리 값으로 넣기. 정규화도 같이 수행하기
        for i in range(compressed_num_of_lidars):
            compressed_lidars_data[i] = min(preprocessed_lidar_data[i*3:(i+1)*3]) / self.MAX_RANGE  
        
        # Lidar 데이터도 NaN 방지 처리 추가
        self.latest_lidar = np.nan_to_num(compressed_lidars_data, nan=0.0)

    def camera_callback(self,msg):
        # 이미지 받은 거 리사이징하기
        # ddpg 논문에 나와있던 대로 64*64로 할거임 cnn에서 나중에 flattern하기

        # uint8로 데이터가 들어옴
        # 정수형이니까 변환 및 255로 나눠서 정규화
        raw_img_data = np.frombuffer(msg.data, dtype=np.uint8)

        reshaped_img_data = raw_img_data.reshape(msg.height, msg.width, 3)
        resized_image_data = cv2.resize (reshaped_img_data,(self.IMG_WIDTH,self.IMG_HEIGHT))
        
        resized_image_data = resized_image_data.astype(np.float32) / 255.0
        # 이미지 데이터 NaN 방지 처리
        self.latest_image = np.nan_to_num(resized_image_data, nan=0.0)

    def imu_callback(self, msg):
        # 1. 원본 데이터 NaN/Inf 체크 (가제보 오류 방지)
        raw_x = np.nan_to_num(msg.linear_acceleration.x, nan=0.0, posinf=0.0, neginf=0.0)
        raw_y = np.nan_to_num(msg.linear_acceleration.y, nan=0.0, posinf=0.0, neginf=0.0)
        raw_ang_z = np.nan_to_num(msg.angular_velocity.z, nan=0.0, posinf=0.0, neginf=0.0)

        # 2. 충돌 감지 (임계값 20.0 정도로 유지)
        if abs(raw_x) > 20.0 or abs(raw_y) > 20.0:
            self.collision_flag = 1.0
            self.collision_sensor = f'imu (acc_x={raw_x:.1f}, acc_y={raw_y:.1f})'

        # 3. [핵심] 정규화 및 타이트한 클리핑
        # 다른 센서(이미지, 라이다)가 0~1 사이이므로 IMU도 절대 1.0을 넘지 않게 합니다.
        lin_acc_x = np.clip(raw_x / 20.0, -1.0, 1.0) 
        lin_acc_y = np.clip(raw_y / 20.0, -1.0, 1.0)
        ang_vel_z = np.clip(raw_ang_z / 1.0, -1.0, 1.0)

        self.latest_imu = [ang_vel_z, lin_acc_x, lin_acc_y]
        
    def lidar_callback(self,msg):
        self.latest_lidar_raw = msg.ranges # Raw data 보관 (Stuck 감지용)
        # inf라고 나오는 거는 최대 거리 10으로 설정해야 함.
        all_lidar_data = msg.ranges
        num_of_lidar = len(all_lidar_data)
        preprocessed_lidar_data = [0.0] * num_of_lidar

        # inf 로 나오는 것들 먼저 최대 거리 상수 10으로 바꾸기
        for i in range(num_of_lidar): 
            if math.isinf(all_lidar_data[i]):
                preprocessed_lidar_data[i] = self.MAX_RANGE
            else:
                preprocessed_lidar_data[i] = all_lidar_data[i]

        # 1도 당 하나의 데이터로 360도가 들어오는데, 이는 연산량이 너무 많아질 것 같아서 3도 당 1개로 다운샘플링 수행
        compressed_num_of_lidars = int(num_of_lidar/ 3)
        compressed_lidars_data = [0.0] * compressed_num_of_lidars

        # 3도 중에서 제일 작은 값으로 해서 장애물과 가까운 거리 값으로 넣기. 정규화도 같이 수행하기
        for i in range(compressed_num_of_lidars):
            compressed_lidars_data[i] = min(preprocessed_lidar_data[i*3:(i+1)*3]) / self.MAX_RANGE  
        
        # Lidar 데이터도 NaN 방지 처리 추가
        self.latest_lidar = np.nan_to_num(compressed_lidars_data, nan=0.0)
    # [주석 처리] 기존 통합 콜백 방식
    # def collision_callback(self, msg, sensor_name='unknown'):
    #     contact_lists = msg.contacts
    #     for contact in contact_lists:
    #         contact1_name = contact.collision1.name
    #         contact2_name = contact.collision2.name
    #         print(f"[DEBUG CONTACT from {sensor_name}] {contact1_name} <-> {contact2_name}")
    #         if 'car' in contact1_name or 'car' in contact2_name:
    #             if 'ground_plane' in contact1_name or 'ground_plane' in contact2_name:
    #                 continue
    #             self.get_logger().error(f"!!! 충돌 발생 [{sensor_name}] !!! {contact1_name} <-> {contact2_name}")
    #             self.collision_flag = 1.0
    #             break

    def collision_chassis_callback(self, msg):
        if msg.contacts:
            self.collision_flag = 1.0
            self.collision_sensor = 'chassis'

    def collision_camera_callback(self, msg):
        if msg.contacts:
            self.collision_flag = 1.0
            self.collision_sensor = 'camera'

    def collision_tire_lf_callback(self, msg):
        for contact in msg.contacts:
            c1 = contact.collision1.name
            c2 = contact.collision2.name
            if 'ground_plane' in c1 or 'ground_plane' in c2:
                continue  # 지면 접촉 무시
            self.collision_flag = 1.0
            self.collision_sensor = 'tire_lf'
            break

    def collision_tire_rf_callback(self, msg):
        for contact in msg.contacts:
            c1 = contact.collision1.name
            c2 = contact.collision2.name
            if 'ground_plane' in c1 or 'ground_plane' in c2:
                continue  # 지면 접촉 무시
            self.collision_flag = 1.0
            self.collision_sensor = 'tire_rf'
            break

    def collision_tire_lr_callback(self, msg):
        for contact in msg.contacts:
            c1 = contact.collision1.name
            c2 = contact.collision2.name
            if 'ground_plane' in c1 or 'ground_plane' in c2:
                continue  # 지면 접촉 무시
            self.collision_flag = 1.0
            self.collision_sensor = 'tire_lr'
            break

    def collision_tire_rr_callback(self, msg):
        for contact in msg.contacts:
            c1 = contact.collision1.name
            c2 = contact.collision2.name
            if 'ground_plane' in c1 or 'ground_plane' in c2:
                continue  # 지면 접촉 무시
            self.collision_flag = 1.0
            self.collision_sensor = 'tire_rr'
            break

    def model_car_odometry_callback(self, msg):
        # 과제에서 estimated pose and velocity를 사용하라고 하고 있음.
        
        # 추정된 위치 좌표
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        
        # 정규화하기
        norm_x = np.clip(odom_x / 15.0, -1.0, 1.0)
        norm_y = np.clip(odom_y / 15.0, -1.0, 1.0)

        # 추정된 orientation. 이게 방향을 의미한다고 함. 같이 써야 될 거 같음.
        ori_x = msg.pose.pose.orientation.x
        ori_y = msg.pose.pose.orientation.y
        ori_z = msg.pose.pose.orientation.z
        ori_w = msg.pose.pose.orientation.w

        # orientation은 직관적이지 않아 yaw좌표로 변환해야 한다고 함.
        siny_cosp = 2 * (ori_w * ori_z + ori_x * ori_y)
        cosy_cosp = 1 - 2 * (ori_y * ori_y + ori_z * ori_z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp) # 출력이 라디안 범위로 나온다고 하여 정규화를 추가함.

        normalized_odom_yaw = odom_yaw / math.pi # -1.0 ~ 1.0 의 범위로 정규화

        # 추정된 선가속도와 각속도, IMU에서 이미 넣어주고 있어서 겹치는 거 같지만 과제에 velocity를 넣어라고 명시하고 있어서 추가함.
        odom_linear_vel = msg.twist.twist.linear.x
        odom_angular_vel = msg.twist.twist.angular.z

        # 물리값 클리핑 (안전장치)
        odom_linear_vel = np.nan_to_num(odom_linear_vel, nan=0.0)
        odom_angular_vel = np.nan_to_num(odom_angular_vel, nan=0.0)

        norm_linear_vel = np.clip(odom_linear_vel / 1.0, -1.1, 1.1)
        norm_angular_vel = np.clip(odom_angular_vel / 0.5, -1.1, 1.1)

        self.latest_odometry_info = [norm_x,norm_y,normalized_odom_yaw ,norm_linear_vel,norm_angular_vel] # pose와 velocity 정보 모두 함께 담아 저장

    #def model_car_tf_callback(self,msg): -> 사용안함.
      #  pass

    def world_car_pose_callback(self, msg):
        # world관점에서 정확하게 car가 어디있는지를 알아야 골에 가까울수록 긍정보상을 줄 수 있음.
        for transform in msg.transforms: # 여러 데이터를 받으므로 그 중에 child_frame_id가 car인 것을 찾음
            if transform.child_frame_id == 'car': 
                world_car_pose_x = transform.transform.translation.x # world 기준 car 위치
                world_car_pose_y = transform.transform.translation.y
                world_car_pose_z = transform.transform.translation.z

                self.world_car_pose = [world_car_pose_x,world_car_pose_y,world_car_pose_z]
                break

    def send_request(self): # 요청 보내는 함수. 환경 리셋해달라고.
        # 서비스가 준비될 때까지 대기
        while not self.cli_env_reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset 서비스를 기다리는 중...')

        # 랜덤 좌표 생성
        x, y, z = self.respawn_generator.get_safe_pose(use_random=True)

        self.req.x = x
        self.req.y = y
        self.req.z = z

        self.get_logger().info(f'랜덤한 좌표에 자동차 생성: ({x:.2f}, {y:.2f},{z:.2f})')

        return self.cli_env_reset.call_async(self.req)

    # callback으로 받은 상태들을 하나로 모아서 state를 만드는 함수
    def get_observation(self):
        if (self.latest_image is None or self.latest_imu is None or
            self.latest_lidar is None or self.latest_odometry_info is None):
            self.get_logger().warn('센서 값 중에 아직 안 들어온 것이 있음.')
        sensor_data = np.concatenate([
            np.array(self.latest_lidar,dtype=np.float32), # sensor에 들어오는 값은 float64인데, 사진도 32로 했고 replay buffer에 쌓이는 걸 고려했을때 32로 함.
            np.array(self.latest_imu,dtype=np.float32),
            np.array(self.latest_odometry_info,dtype=np.float32)
        ])
        
        # 마지막 안전장치: 합쳐진 데이터에서도 혹시 모를 NaN 제거
        sensor_data = np.nan_to_num(sensor_data, nan=0.0)

        state = {
            'image':self.latest_image,
            'sensors':sensor_data
            }

        return state
    
    # reward를 계산하는 함수
    def compute_dist(self):
        if self.world_car_pose is None:
            return 23.0 # 만약 None이라면 제일 먼 거리 값 넣기 
        now_car_pose = self.world_car_pose
        pose_x = (now_car_pose[0] - self.goal_position[0])
        pose_y = (now_car_pose[1] - self.goal_position[1])
        pose_z = (now_car_pose[2] - self.goal_position[2])
        
        computed_dist = math.sqrt(pose_x*pose_x + pose_y*pose_y + pose_z*pose_z)

        return computed_dist
    
    def compute_reward(self):
        # 목표와 가까울 때 양의 보상
        # 충돌감지는 끝내기 큰 벌점과 함께
        # 이외의 경우는 내가 정의한 보상 함수에 의해 점수를 줘야함

        if self.collision_flag == True:
            reward = -0.1
            return reward
        
        dist = self.compute_dist()

        if dist < 0.5: # 내가 만든 차체를 덮는 원의 지름을 1.5로 할 시, 해당 만큼 가깝게 도착하면 목표지점에 도착한걸로 판단하였음.
            self.is_arrived_at_goal = True # 목표에 도달했다고 표시 남김.
            reward = 0.2 
            return reward # 큰 양의 보상
        # 충돌로 종료되거나 목표 도달로 양의 보상을 받는 경우가 아니라면 보상함수로 계산해야 함
        if self.prev_dist != None:
            reward_dist = (self.prev_dist - dist) * 0.5 # 거리 줄어든 만큼 보상 (스케일 유지하되 전체적으로 작아짐)
            
            # 물리 엔진 글리치로 인해 거리가 순간적으로 튀면 보상이 폭발하므로 클리핑 (안전장치)
            reward_dist = np.clip(reward_dist, -2.0, 2.0)
        else:
            reward_dist = 0

        self.prev_dist = dist

        time_penalty = -0.005 # -0.01 -> -0.05 (시간 낭비 벌점 강화)

        total_reward = reward_dist + time_penalty
        
        total_reward = np.clip(total_reward, -1.0, 1.0)

        return total_reward

    # 종료조건 확인 함수
    # 충돌해서 끝나거나, 스텝 수가 맥스가 되었거나, 골인지점과 가까워서 도착했다고 판단할 떄
    def check_done(self):
        rclpy.spin_once(self, timeout_sec=0) 
        
        # 1. Contact sensor 충돌 감지
        if self.collision_flag:
            self.get_logger().error(f'[종료] 충돌! 센서={self.collision_sensor}, step={self.steps_done}')
            return 1.0
        
        # 2. 라이다 기반 근접 충돌 감지 (contact sensor 백업용)
        if self.latest_lidar_raw is not None:
            valid_ranges = [r for r in self.latest_lidar_raw if not math.isinf(r) and r > 0]
            if valid_ranges:
                min_dist = min(valid_ranges)
                if min_dist < 0.25:
                    self.get_logger().error(f'[종료] 라이다 근접 충돌! min_dist={min_dist:.2f}m, step={self.steps_done}')
                    return 1.0

        # 3. 목표 도달
        if self.is_arrived_at_goal:
            self.get_logger().info(f'[종료] 목표 도달! step={self.steps_done}')
            return 1.0
        
        # 4. 최대 스텝
        if self.steps_done == self.MAX_STEP:
            self.get_logger().warn(f'[종료] 최대 스텝({self.MAX_STEP}) 도달')
            return 1.0
        
        return 0.0
    
    # ddpg agent 에게서 받은 action으로 진행
    def step(self,action): 
        steering_angle = float(action[0]) # Actor에서 이미 0.6 배수가 적용됨
        linear_velocity = float(action[1]) # 그 다음 속도

        move_msg = Twist()
        move_msg.linear.x = linear_velocity
        move_msg.angular.z = steering_angle
        self.pub_cmd_vel.publish(move_msg) # publisher로 전송
        
        # Stuck 감지 로직 제거 (오작동 방지)
        # 사용자 요청으로 제거함.

        rclpy.spin_once(self,timeout_sec=0.1) # 결과를 받을때 까지 물리엔진을 기다려줄 시간이 필요하다고 함.

        next_state = self.get_observation()
        reward = self.compute_reward()
        done = self.check_done()

        self.steps_done += 1

        return next_state,reward,done
        
    # 이 밑으로 이제 gym의 메서드들을 구현해야함.
    def reset(self):
        self.pub_cmd_vel.publish(Twist()) # Twist의 메시지를 인자 없이 생성해서 두 값 다 0,0으로 초기화.
        # 랜덤 위치 다시 받아오기
        reset_state = self.send_request()
        # 물리 엔진에서 해당하는 작업이 완료될때까지 대기 (타임아웃 5초)
        rclpy.spin_until_future_complete(self, reset_state, timeout_sec=5.0)
        if not reset_state.done():
            self.get_logger().warn('Reset 서비스 응답 타임아웃')
        else:
            self.get_logger().info('Reset 서비스 응답 수신 완료')

        # 센서 데이터가 도착할 때까지 spin
        timeout = 10.0  # 최대 10초 대기
        start_time = self.get_clock().now()
        last_log_time = start_time

        while True:
            rclpy.spin_once(self, timeout_sec=0.1)

            # 모든 센서 데이터가 도착했는지 확인
            if (self.latest_image is not None and
                self.latest_imu is not None and
                self.latest_lidar is not None and
                self.latest_odometry_info is not None and
                self.world_car_pose is not None):
                self.get_logger().info('모든 센서 데이터 수신 완료')
                break

            # 1초마다 센서 상태 로그
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed - (last_log_time - start_time).nanoseconds / 1e9 >= 1.0:
                missing = []
                if self.latest_image is None: missing.append('camera')
                if self.latest_imu is None: missing.append('imu')
                if self.latest_lidar is None: missing.append('lidar')
                if self.latest_odometry_info is None: missing.append('odometry')
                if self.world_car_pose is None: missing.append('world_pose')
                self.get_logger().info(f'대기 중 ({elapsed:.1f}초) - 미수신 센서: {", ".join(missing)}')
                last_log_time = self.get_clock().now()

            # 타임아웃 체크
            if elapsed > timeout:
                missing = []
                if self.latest_image is None: missing.append('camera')
                if self.latest_imu is None: missing.append('imu')
                if self.latest_lidar is None: missing.append('lidar')
                if self.latest_odometry_info is None: missing.append('odometry')
                if self.world_car_pose is None: missing.append('world_pose')
                self.get_logger().error(f'센서 데이터 대기 타임아웃 미수신: {", ".join(missing)}')
                break

        # 값들 초기화
        self.steps_done=0
        self.collision_flag=0.0
        self.collision_sensor=None
        self.is_arrived_at_goal=0.0
        self.prev_dist = None
        
        # Stuck 감지 변수 초기화 (중요)
        self.stuck_count = 0
        if hasattr(self, 'prev_lidar_raw'):
            self.prev_lidar_raw = None
            
        return self.get_observation()
    
    def publish_zero_action(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(stop_msg)
