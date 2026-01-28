MAP_WIDTH = 30.0
MAP_HEIGHT = 30.0
MAP_WALL_SIZE = {width:2,height:28}
RADIUS = 1.5

class respawn_pose_node:
    def __init__(self):

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

    def get_safe_pose(use_random=True):
        d_map_wall_size = MAP_WALL_SIZE.width / 2.0 # map 구성 시 바깥 가장자리 벽들의 중심이 월드맵의 선에 걸쳐있기 때문에 안쪽으로 절반만큼 겹침
        width_limit = MAP_WIDTH / 2.0 - d_map_wall_size - RADIUS # 추가로 벽 안쪽에 바로 중심 점이 소환될 시 바깥 벽이랑 겹치기 때문에 반지름만큼 더 뺴줌
        height_limit = MAP_HEIGHT / 2.0 - d_map_wall_size - RADIUS

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
                return (x,y,0.05) # 바퀴의 크기를 생각했을때 지면과 딱 붙여도 상관없을 것 같은데 혹시 몰라서 조금 띄움
