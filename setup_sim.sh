#!/bin/bash
export IGN_PARTITION=david_sim
export IGN_IP=127.0.0.1

# 1. 브리지 실행 (CameraInfo 추가 및 시간 리매핑)
ros2 run ros_gz_bridge parameter_bridge \
  /world/my_car_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock \
  /model/car/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V \
  /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image[ignition.msgs.Image \
  /camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo \
  /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  /lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked \
  --ros-args -r /model/car/tf:=/tf \
  -r /world/my_car_world/clock:=/clock &

sleep 2
# 2. 정적 변환 발행 (카메라 및 라이다 연결)
ros2 run tf2_ros static_transform_publisher 1.05 0 0.5 0 0 0 chassis/chassis_link car/chassis/chassis_link/gpu_lidar &
ros2 run tf2_ros static_transform_publisher 1.1 0 0.3 -1.5708 0 -1.5708 chassis/chassis_link car/chassis/chassis_link/camera &