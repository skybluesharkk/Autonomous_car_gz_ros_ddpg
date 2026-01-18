#!/bin/bash
echo "가제보 완전 초기화를 시작합니다..."

# 1. 패턴 매칭을 이용한 강력한 프로세스 종료 (pkill 사용)
# gz-sim과 관련된 모든 프로세스를 강제로 찾아내서 종료합니다.
pkill -9 -f gz-sim 2>/dev/null
pkill -9 -f ruby 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null

# 2. 가제보 캐시 및 설정 파일 완전 삭제
# 단순 캐시뿐만 아니라 GUI의 레이아웃과 상태가 저장된 폴더를 통째로 날립니다.
rm -rf ~/.gz/sim/

# 3. 맥북 전용: 공유 메모리 및 통신 잔상 정리
# 맥 환경에서 가제보 통신이 꼬였을 때 발생하는 잔상을 비웁니다.
rm -rf /tmp/gz-*

echo "프로세스 및 전체 캐시 정리가 완료되었습니다."