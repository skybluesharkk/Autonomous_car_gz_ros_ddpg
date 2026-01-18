#!/bin/bash

# 필수 환경 변수 설정
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models:$(pwd)/models/my_car_world

# macOS/M2 렌더링 에러 해결 핵심 변수
export GZ_RENDERING_ENGINE_GUESS=ogre2
export OGRE_RTT_MODE=Copy  # 맥북 그래픽 충돌 방지 핵심 설정

echo "가제보 시뮬레이션(Ogre2 엔진) 실행..."
gz sim -s -v 4 models/my_car_world/my_car_world.sdf