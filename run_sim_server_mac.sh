#!/bin/zsh

# 1. 현재 작업 디렉토리 및 Conda 경로 자동 설정
export PKG_PATH=$(pwd)

# 2. 리소스 및 플러그인 경로 (Conda 내부 공유 자원 포함)
# 가제보와 렌더링 엔진이 기본 머티리얼(shadow_caster 등)을 찾을 수 있도록 share 경로를 추가합니다.
export IGN_GAZEBO_RESOURCE_PATH="$PKG_PATH:$PKG_PATH/models:$PKG_PATH/models/obstacles:$PKG_PATH/worlds:$PKG_PATH/worlds/my_car_world"
export IGN_GAZEBO_RESOURCE_PATH="$CONDA_PREFIX/share/ignition/gazebo6:$IGN_GAZEBO_RESOURCE_PATH"
export IGN_GAZEBO_RESOURCE_PATH="$CONDA_PREFIX/share/ignition/rendering6:$IGN_GAZEBO_RESOURCE_PATH"

# 렌더링 엔진 전용 리소스 경로 (PSSM/shadow_caster 에러 해결 핵심)
export IGN_RENDERING_RESOURCE_PATH="$CONDA_PREFIX/share/ignition/rendering6"

# 플러그인 경로 설정
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$CONDA_PREFIX/lib/ign-gazebo-6/plugins"

# 3. 그래픽 환경 설정 초기화 및 최적화
# 이전 시도에서 꼬였을 수 있는 변수들을 정리합니다.
unset OGRE_RESOURCE_PATH
unset __GLX_VENDOR_LIBRARY_NAME
unset MESA_GL_VERSION_OVERRIDE

# 맥 M2에서 렌더링 안정성을 위해 엔진을 명시합니다.
export GZ_RENDERING_DEFAULT_ENGINE="ogre"

# 4. 실행할 월드 파일
WORLD_FILE="$PKG_PATH/worlds/my_car_world/my_car_world.sdf"

echo "------------------------------------------"
echo "영찬님의 MacBook Air (M2)에서 서버를 재가동합니다."
echo "에러 수정: OGRE 기본 리소스 경로 및 머티리얼 로드 보완"
echo "------------------------------------------"

# 5. 서버 실행
ign gazebo -s -r "$WORLD_FILE" -v 4 --render-engine ogre