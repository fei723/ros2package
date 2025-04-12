#!/usr/bin/env bash
# This script converts xacro (URDF variant) into MJCF for `robot_description` package
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
ROBOT_ID=M92U0
HAND_ID=m92u_hand

show_help() {
    echo "Usage: $0 <robot_id> <hand_id>"
    echo "  robot_id: The ID of the robot. Supported types: [M92C, M92UW, M92U0]."
    echo "  hand_id: The ID of the hand. Supported types: [m92c_hand, m92u_hand]."
    echo "  Example: $0 M92UW m92u_hand"
}

# 检查参数数量
if [ "$#" -lt 2 ] || [ $# -eq 0 ] || [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    show_help
    exit 0
fi

ROBOT_ID=$1
HAND_ID=$2 # fourier_hand inspire_hand
XACRO_PATH="$(dirname "$(dirname "$SCRIPT_DIR")")/robots/"${ROBOT_ID}"/${ROBOT_ID}.urdf.xacro"

MJCF_PATH="$(dirname "$(dirname "$SCRIPT_DIR")")/mjcf/${ROBOT_ID}/${ROBOT_ID}.mujoco.original.xml"
TMP_URDF_PATH="$(dirname "$(dirname "$SCRIPT_DIR")")/urdf/${ROBOT_ID}/${ROBOT_ID}_tmp.urdf"

# Arguments for xacro
XACRO_ARGS=(
    transmission_type:=simple
    ros2_control:=true
    ros2_control_plugin:=mujoco
    base_parent_link:="base_footprint"
)
MESHES_PATH="$(dirname "$(dirname "${SCRIPT_DIR}")")/meshes/${ROBOT_ID}/visual"
HAND_MESHES_PATH="$(dirname "$(dirname "${SCRIPT_DIR}")")/meshes/robot_hands/${HAND_ID}/visual"

echo "正在删除 $TARGET_DIR 下的所有软链接..."
find "$MESHES_PATH" -type l -exec rm -f {} \;

echo "删除完成。"

for dir in "$HAND_MESHES_PATH"/*; do
    if [ -f "$dir" ]; then
        # 获取文件名
        filename=$(basename "$dir")
        # 创建软链接到目标目录
        ln -s "$dir" "$MESHES_PATH/$filename"
        echo "已创建软链接: $filename"
    fi
done
# Remove old MJCF file
rm "${MJCF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to MJCF
ros2 run xacro xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${TMP_URDF_PATH}" &&
    compile "${TMP_URDF_PATH}" "${MJCF_PATH}" &&
    echo "Created new ${MJCF_PATH}"

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null
