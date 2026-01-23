#!/bin/bash

# 1. PATH 및 기본 경로 설정
export PATH="$CONDA_PREFIX/bin:/usr/bin:/bin:/usr/sbin:/sbin"
# 패키지 절대 경로를 지정하여 어떤 위치에서 실행해도 문제가 없게 합니다.
PKG_PATH="/home/david/ros2_car_ws/src/gazebo_car_sim_package"

# 2. 리소스 경로 설정 (가장 중요)
# Error Code 13을 해결하기 위해 car와 obstacles 부모 폴더를 모두 등록합니다.
MODELS_PATH="$PKG_PATH/models"
OBSTACLES_PATH="$PKG_PATH/models/obstacles"
WORLDS_PATH="$PKG_PATH/worlds/my_car_world"

# Fortress(v6)와 Harmonic(v8) 모두 호환되도록 두 변수를 모두 설정합니다.
export GZ_SIM_RESOURCE_PATH="$MODELS_PATH:$OBSTACLES_PATH:$WORLDS_PATH:$GZ_SIM_RESOURCE_PATH"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"

# 3. 그래픽 엔진 안정화 설정 (Linux/MobaXterm 맞춤)
# 맥북용 DYLD_ 대신 리눅스용 LD_LIBRARY_PATH를 사용합니다.
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# 가제보 플러그인 경로 (현재 서버의 Fortress v6 기준)
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$CONDA_PREFIX/lib/ign-gazebo-6/plugins"
export IGN_RENDERING_PLUGIN_PATH="$CONDA_PREFIX/lib/ign-rendering-6/engine-plugins"

# 4. X11 포워딩 확인 (디버깅용)
if [ -z "$DISPLAY" ]; then
    echo "에러: DISPLAY 변수가 없습니다. MobaXterm 연결을 확인하세요."
    exit 1
fi

echo "------------------------------------------"
echo "영찬님의 가제보 GUI를 시작합니다 (Client Mode)."
echo "대상 월드: $WORLDS_PATH/my_car_world.sdf"
echo "------------------------------------------"

# 5. GUI 모드(-g)로 서버에 접속
# 서버가 이미 xvfb-run으로 실행 중이라면, 이 명령어가 화면만 띄워줍니다.
ign gazebo -g "$WORLDS_PATH/my_car_world.sdf"