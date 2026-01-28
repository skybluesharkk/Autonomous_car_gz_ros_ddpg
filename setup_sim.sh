#!/bin/bash
export IGN_PARTITION=david_sim
export IGN_IP=127.0.0.1

# 1. 브리지 실행 (핵심: /model/car/tf를 표준 /tf로 연결)
ros2 run ros_gz_bridge parameter_bridge \
  /world/my_car_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock \
  /model/car/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V \
  /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /camera@sensor_msgs/msg/Image[ignition.msgs.Image \
  /camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo \
  /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  /imu@sensor_msgs/msg/Imu[ignition.msgs.IMU \
  /model/car/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
  /world/my_car_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V \
  /collision@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts \
  --ros-args \
  -r /model/car/tf:=/tf \
  -r /world/my_car_world/clock:=/clock

sleep 2

# 2. 정적 변환 발행 (부모 프레임을 SDF의 child_frame_id와 일치시킴)
# SDF에서 <child_frame_id>chassis::chassis_link</child_frame_id>로 설정했으므로 
# ROS 2에서는 car/chassis/chassis_link가 됩니다.
ros2 run tf2_ros static_transform_publisher 0 0 0.4 0 0 0 car/chassis/chassis_link lidar_frame &
ros2 run tf2_ros static_transform_publisher 1.0 0 0.3 -1.5708 0 -1.5708 car/chassis/chassis_link camera_link &