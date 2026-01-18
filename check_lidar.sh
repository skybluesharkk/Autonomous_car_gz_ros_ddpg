#!/bin/bash
# 1. 가제보 통신을 위한 IP 설정
export GZ_IP=127.0.0.1

# 2. Conda 경로 간섭 제거 (가장 중요)
# Homebrew의 gz-transport 라이브러리를 정상적으로 로드하기 위함입니다.
export PATH=$(echo $PATH | tr ":" "\n" | grep -v "miniconda" | grep -v "anaconda" | tr "\n" ":" | sed 's/:$//')

echo "LIDAR 센서 데이터를 실시간으로 출력합니다... (종료: Ctrl+C)"

# 3. 라이다 토픽 모니터링 실행
# chassis.sdf에 정의된 /lidar 토픽을 구독합니다.
gz topic -e -t /lidar