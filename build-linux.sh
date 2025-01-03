#!/bin/bash

set -e
echo "$0 $@"

while getopts ":t:a:" opt; do
  case $opt in
    t) TARGET_SOC=$OPTARG ;;
    a) TARGET_ARCH=$OPTARG ;;
    :) echo "Option -$OPTARG requires an argument." ; exit 1 ;;
    ?) echo "Invalid option: -$OPTARG index:$OPTIND" ;;
  esac
done

# 判断是否指定 -t 和 -a
if [ -z "${TARGET_SOC}" ] || [ -z "${TARGET_ARCH}" ]; then
  echo "$0 -t <target> -a <arch>"
  exit -1
fi

if [[ -z ${GCC_COMPILER} ]]; then
    # 根据 TARGET_SOC 自动推断或提示
    if [[ ${TARGET_SOC} = "rv1106"  || ${TARGET_SOC} = "rv1103" ]]; then
        echo "Please set GCC_COMPILER for $TARGET_SOC"
        exit
    elif [[ ${TARGET_SOC} = "rv1109" || ${TARGET_SOC} = "rv1126" ]]; then
        GCC_COMPILER=arm-linux-gnueabihf
    else
        GCC_COMPILER=aarch64-linux-gnu
    fi
fi
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++

if ! command -v "${CC}" >/dev/null 2>&1; then
    echo "${CC} is not available"
    exit
fi

# 默认构建类型: Release
BUILD_TYPE=Release
BUILD_DEMO_NAME="AI_Face_Tracker"

ROOT_PWD=$( cd "$( dirname "$0" )" && pwd )

# 关键：把原先的 cpp 改为 src/cpp
BUILD_DEMO_PATH="${ROOT_PWD}/src/cpp"

if [[ ! -d "${BUILD_DEMO_PATH}" ]]; then
    echo "Error: cpp directory not found in ${BUILD_DEMO_PATH}"
    exit
fi

# 简单地把 rv1103 => rv1106，rk3566/8/2 => rk356x 等映射
case ${TARGET_SOC} in
    rv1103)  TARGET_SOC="rv1106" ;;
    rk3566|rk3568|rk3562) TARGET_SOC="rk356x" ;;
    rv1126)  TARGET_SOC="rv1126" ;;
    # ...
    *)
        # 可选：只做简单判断
        ;;
esac

# 安装目录固定到脚本同级的 release 下
INSTALL_DIR="${ROOT_PWD}/release"
# build 目录同理
BUILD_DIR="${ROOT_PWD}/build/build_${BUILD_DEMO_NAME}_${TARGET_SOC}_${TARGET_ARCH}_${BUILD_TYPE}"

mkdir -p "${BUILD_DIR}"
rm -rf "${INSTALL_DIR}" 2>/dev/null
mkdir -p "${INSTALL_DIR}"

cd "${BUILD_DIR}"
cmake "${BUILD_DEMO_PATH}" \
    -DTARGET_SOC=${TARGET_SOC} \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=${TARGET_ARCH} \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"

make -j4
make install

echo "Install done! Check ${INSTALL_DIR}"

