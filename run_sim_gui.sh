#!/bin/bash
# 1. IP 설정
export GZ_IP=127.0.0.1

# 2. PATH에서 Conda 경로를 일시적으로 완전히 제거 (가장 중요)
# 현재 PATH에서 'miniconda'나 'anaconda'가 포함된 경로를 제외한 나머지만 다시 설정합니다.
export PATH=$(echo $PATH | tr ":" "\n" | grep -v "miniconda" | grep -v "anaconda" | tr "\n" ":" | sed 's/:$//')

# 3. Qt 관련 환경변수 초기화
unset QT_QPA_PLATFORM_PLUGIN_PATH
unset QT_PLUGIN_PATH
unset QML2_IMPORT_PATH
export DYLD_LIBRARY_PATH=""

echo "Conda 경로를 완전히 격리하고 가제보 GUI를 실행합니다..."

# 4. GUI 실행
gz sim -g