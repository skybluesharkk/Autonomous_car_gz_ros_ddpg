#!/bin/bash

# 1. 기본 패키지 경로 설정
export PKG_PATH="/home/david/ros2_car_ws/src/gazebo_car_sim_package"
echo "package 경로 설정 완료"
export IGN_PARTITION=david_sim
echo "partition name 설정 완료"
export IGN_IP=127.0.0.1
echo "gazebo ip 설정 완료"
#export ROS_DOMAIN_ID=36
echo "ROS_DOMAIN_ID 설정 완료"
# 2. 모델 검색 경로 최적화 (이미지 구조 반영)
# 각 모델 폴더의 '부모' 폴더들을 모두 등록해줘야 합니다.
MODELS_DIR="$PKG_PATH/models"
OBSTACLES_DIR="$PKG_PATH/models/obstacles"
# car, knuckle, tire가 들어있는 경로
CAR_MODELS_DIR="$PKG_PATH/worlds/my_car_world" 

# 3. 가제보 리소스 경로 통합
# 기존 경로($IGN_GAZEBO_RESOURCE_PATH)를 뒤에 붙여 시스템 기본 모델도 찾게 합니다.
export IGN_GAZEBO_RESOURCE_PATH="$MODELS_DIR:$OBSTACLES_DIR:$CAR_MODELS_DIR:$IGN_GAZEBO_RESOURCE_PATH"
# 가제보 기본 share 폴더 추가 (ground_plane 등 기본 모델용)
export IGN_GAZEBO_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH:$CONDA_PREFIX/share/ignition/gazebo6"
export GZ_SIM_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH"

# 4. 플러그인 및 라이브러리 경로
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$CONDA_PREFIX/lib/ign-gazebo-6/plugins"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"

# 5. NVIDIA GPU 가속 (RTX 4060)
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export MESA_GL_VERSION_OVERRIDE=4.5

# 6. 월드 파일 절대 경로
WORLD_FILE="$PKG_PATH/worlds/my_car_world/my_car_world.sdf"

echo "------------------------------------------"
echo "영찬님의 서버에서 모델 경로를 수정하여 가제보를 실행합니다."
echo "찾으려는 모델 위치: models/, obstacles/, my_car_world/"
echo "------------------------------------------"

# 7. xvfb-run 실행 (GUI 없이 서버 가동)
xvfb-run --server-num=99 --server-args="-screen 0 1024x768x24 +extension GLX" \
ign gazebo -s -r "$WORLD_FILE" -v 4