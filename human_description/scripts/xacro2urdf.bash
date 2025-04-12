#!/usr/bin/env bash
# This script converts xacro (URDF variant) into URDF for `human_description` package
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
ROBOT_ID=M92UW
LOAD_GROUPS="[left_leg, right_leg, waist, sensors, head, left_arm, right_arm, left_hand, right_hand]"
# 检查参数数量
if [ "$#" -lt 2 ]; then
    echo "用法: $0 ROBOT_ID"
    exit 1
fi

ROBOT_ID=$1
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/robots/"${ROBOT_ID}"/${ROBOT_ID}.urdf.xacro"
URDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/"${ROBOT_ID}"/${ROBOT_ID}.urdf"
# URDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/"${ROBOT_ID}"/${ROBOT_ID}_fixed_flinger.urdf"
HAND_ID=fourier_hand # fourier_hand inspire_hand

if [ "$2" == "all" ]; then
    LOAD_GROUPS="[left_leg, right_leg, waist, sensors, head, left_arm, right_arm, left_hand, right_hand]"
elif [ "$2" == "lower" ]; then
    LOAD_GROUPS="[left_leg, right_leg, sensors]"
    URDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/${ROBOT_ID}/${ROBOT_ID}_lower_limbs.urdf"
elif [ "$2" == "upper" ]; then
    LOAD_GROUPS="[waist, sensors, head, left_arm, right_arm, left_hand, right_hand]"
    URDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/${ROBOT_ID}/${ROBOT_ID}_upper_limbs.urdf"
else
    echo "参数不匹配"
fi

# Arguments for xacro
XACRO_ARGS=(
    transmission_type:=simple
    ros2_control:=false
    # hand_id:="${HAND_ID}"
    ros2_control_plugin:=isaac
    # base_parent_link:="base"
    # base_position:="0 0 0"
    # finger_joint_fixed:=true
    # head_joint_fixed:=true
    # command_interface:="[position]"
    # "['left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm', 'left_hand', 'right_hand', 'sensors']"
    # load_groups:="[left_leg, right_leg, sensors]"
    # load_groups:="[waist, sensors, head, left_arm, right_arm, left_hand, right_hand]"
    # load_groups:="${LOAD_GROUPS}"
)
MESHES_PATH="$(dirname "${SCRIPT_DIR}")/meshes/${ROBOT_ID}/visual"
HAND_MESHES_PATH="$(dirname "${SCRIPT_DIR}")/meshes/robot_hands/${HAND_ID}/visual"

# Remove old URDF file
rm "${URDF_PATH}" 2>/dev/null



# Process xacro into URDF
ros2 run xacro xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${URDF_PATH}" &&
    echo "Created new ${URDF_PATH}"

check_urdf ${URDF_PATH}
