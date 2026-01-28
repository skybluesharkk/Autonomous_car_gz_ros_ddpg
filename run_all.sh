#!/bin/bash
# run_all.sh - 통합 실행 스크립트
# Gazebo + ROS Bridge + Service Node를 한 번에 실행합니다.

set -e

# 1. 기본 패키지 경로 설정
export PKG_PATH="/home/david/ros2_car_ws/src/gazebo_car_sim_package"
export IGN_PARTITION=david_sim
export IGN_IP=127.0.0.1

# 2. 모델 검색 경로
MODELS_DIR="$PKG_PATH/models"
OBSTACLES_DIR="$PKG_PATH/models/obstacles"
CAR_MODELS_DIR="$PKG_PATH/worlds/my_car_world"

export IGN_GAZEBO_RESOURCE_PATH="$MODELS_DIR:$OBSTACLES_DIR:$CAR_MODELS_DIR:$IGN_GAZEBO_RESOURCE_PATH"
export IGN_GAZEBO_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH:$CONDA_PREFIX/share/ignition/gazebo6"
export GZ_SIM_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH"

# 3. 플러그인 및 라이브러리 경로
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$CONDA_PREFIX/lib/ign-gazebo-6/plugins"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"

# 4. GPU 가속
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export MESA_GL_VERSION_OVERRIDE=4.5

# 5. 월드 파일
WORLD_FILE="$PKG_PATH/worlds/my_car_world/my_car_world.sdf"

echo "=========================================="
echo "통합 시뮬레이션 서버 시작"
echo "=========================================="

# ROS2 환경 소싱
source /home/david/ros2_car_ws/install/setup.bash

# 6. xvfb-run 내에서 모든 것 실행
# 백그라운드에서 Bridge, TF, ServiceNode를 먼저 띄우고, Gazebo를 포그라운드로 실행
xvfb-run --server-num=99 --server-args="-screen 0 1024x768x24 +extension GLX" \
bash -c "
    echo '[시작] ROS GZ Bridge...'
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
        --ros-args --remap /world/my_car_world/clock:=/clock &

    echo '[시작] TF Publishers...'
    ros2 run tf2_ros static_transform_publisher 1.05 0 0.5 0 0 0 chassis/chassis_link car/chassis/chassis_link/gpu_lidar &
    ros2 run tf2_ros static_transform_publisher 1.1 0 0.3 -1.5708 0 -1.5708 chassis/chassis_link car/chassis/chassis_link/camera &

    echo '[시작] Service Node...'
    python3 $PKG_PATH/ddpg_algorithm/service_node.py &

    # 잠시 대기 후 Gazebo 시작
    sleep 2
    echo '[시작] Ignition Gazebo...'
    ign gazebo -s -r $WORLD_FILE -v 4
"
