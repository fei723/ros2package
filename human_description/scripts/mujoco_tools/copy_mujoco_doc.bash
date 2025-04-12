#!/usr/bin/env bash

# 获取脚本当前目录
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
ROBOT_ID=M92U0
HAND_ID=m92u_hand

# 显示帮助信息
show_help() {
    echo "Usage: $0 <robot_id> <hand_id> <target_folder>"
    echo "  robot_id: The ID of the robot. Supported types: [M92C, M92UW, M92U0]."
    echo "  hand_id: The ID of the hand. Supported types: [m92c_hand, m92u_hand]."
    echo "  target_folder: The directory where the MJCF and meshes will be saved."
    echo "  Example: $0 M92UW m92u_hand /path/to/target"
}

# 检查参数数量
if [ "$#" -lt 3 ]; then
    show_help
    exit 1
fi

# 获取输入参数
ROBOT_ID=$1
HAND_ID=$2
TARGET_FOLDER=$3

echo "SCRIPT_DIR is: $SCRIPT_DIR"
# 定义路径
MJCF_PATH="$(dirname "$(dirname "${SCRIPT_DIR}")")/mjcf/${ROBOT_ID}"
MESHES_PATH="$(dirname "$(dirname "${SCRIPT_DIR}")")/meshes/${ROBOT_ID}/visual"
HAND_MESHES_PATH="$(dirname "$(dirname "${SCRIPT_DIR}")")/meshes/robot_hands/${HAND_ID}/visual"

echo "正在删除 $MESHES_PATH 下的所有软链接..."
find "$MESHES_PATH" -type l -exec rm -f {} \;

echo "删除完成。"

# 创建目标文件夹
echo "Creating target folder: $TARGET_FOLDER"
mkdir -p "${TARGET_FOLDER}/meshes/${ROBOT_ID}"
mkdir -p "${TARGET_FOLDER}/mjcf"

# 复制 mesh 文件 和 MJCF 文件 到目标文件夹
echo "Copying mesh files..."
cp -r "${MESHES_PATH}" "${TARGET_FOLDER}/meshes/${ROBOT_ID}"
find "$HAND_MESHES_PATH" -type f -name "*.stl" -exec cp {} "${TARGET_FOLDER}/meshes/${ROBOT_ID}/visual" \;

echo "Copying mjcf files..."
cp -r "${MJCF_PATH}" "${TARGET_FOLDER}/mjcf/"


echo "MJCF and meshes are saved in: $TARGET_FOLDER"
