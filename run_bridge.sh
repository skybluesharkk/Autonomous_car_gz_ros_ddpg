#!/bin/bash
export IGN_PARTITION=david_sim
export IGN_IP=127.0.0.1

ros2 run ros_gz_bridge parameter_bridge \
    '/world/my_car_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
    '/model/car/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V' \
    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
    '/camera@sensor_msgs/msg/Image[ignition.msgs.Image' \
    '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo' \
    '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan' \
    '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU' \
    '/model/car/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
    '/world/my_car_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V' \
    '/model/car/pose@geometry_msgs/msg/Pose]ignition.msgs.Pose' \
    --ros-args \
    -r '/world/my_car_world/clock:=/clock'
