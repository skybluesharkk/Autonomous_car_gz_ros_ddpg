import rclpy
from rclpy.node import Node
from custom_interfaces.srv import Reset 
from geometry_msgs.msg import Pose
import sys
import subprocess
import os
from geometry_msgs.msg import Twist

class ServiceNode(Node):

    def __init__(self):
        super().__init__('ServiceNode')
        # 서비스 서버
        self.srv = self.create_service(Reset, 'reset', self.reset_callback)
        # 속도 초기화용 발행자
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.get_logger().info('서비스 노드 시작됨.')

    def reset_callback(self, request, response):
        self.get_logger().info(f'랜덤 이동할 위치: x={request.x}, y={request.y}, z={request.z}')
        
        # 속도 초기화
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = 0.0
        self.vel_pub.publish(move_msg)

        # Gazebo 서비스를 통해 차량 위치 설정 (ign service 명령어 사용)
        # IGN_PARTITION 환경변수 설정 필요
        env = os.environ.copy()
        env['IGN_PARTITION'] = 'david_sim'
        
        # set_pose 서비스 호출
        pose_cmd = f'''ign service -s /world/my_car_world/set_pose \
            --reqtype ignition.msgs.Pose \
            --reptype ignition.msgs.Boolean \
            --timeout 1000 \
            --req "name: 'car', position: {{x: {request.x}, y: {request.y}, z: {request.z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}"'''
        
        try:
            result = subprocess.run(pose_cmd, shell=True, env=env, capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                self.get_logger().info('Gazebo set_pose 서비스 호출 성공')
            else:
                self.get_logger().warn(f'Gazebo set_pose 실패: {result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Gazebo set_pose 타임아웃')
        except Exception as e:
            self.get_logger().error(f'Gazebo set_pose 오류: {e}')

        response.success = True
        self.get_logger().info('가제보에 reset명령 전달 완료함.')

        return response

def main(args=None):
    rclpy.init(args=args)
    service_node = ServiceNode()
    rclpy.spin(service_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()