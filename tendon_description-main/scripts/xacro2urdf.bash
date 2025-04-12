#!/usr/bin/env bash
# This script converts xacro (URDF variant) into URDF for `human_description` package
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
ROBOT_ID=robot_leg #finger

XACRO_PATH="$(dirname "${SCRIPT_DIR}")/xacro/"${ROBOT_ID}"/"${ROBOT_ID}".urdf.xacro"
URDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/"${ROBOT_ID}"/"${ROBOT_ID}".urdf"

# Arguments for xacro
XACRO_ARGS=(
    transmission_type:=simple
    ros2_control:=true
    ros2_control_plugin:=mujoco
)

# Remove old URDF file
rm "${URDF_PATH}" 2>/dev/null

# Process xacro into URDF
ros2 run xacro xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${URDF_PATH}" &&
    echo "Created new ${URDF_PATH}"

check_urdf ${URDF_PATH}
