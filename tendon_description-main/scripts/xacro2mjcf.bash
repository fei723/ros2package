#!/usr/bin/env bash
# This script converts xacro (URDF variant) into MJCF for `tendon_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"

ROBOT_ID=finger

XACRO_PATH="$(dirname "${SCRIPT_DIR}")/xacro/"${ROBOT_ID}"/"${ROBOT_ID}".urdf.xacro"
MJCF_PATH="$(dirname "${SCRIPT_DIR}")/mjcf/"${ROBOT_ID}"/"${ROBOT_ID}".mujoco.original.xml"
TMP_URDF_PATH="$(dirname "${SCRIPT_DIR}")/mjcf/tendon_tmp.urdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=tendon
    ros2_control:=true
    ros2_control_plugin:=mujoco
    ros2_control_command_interface:="[position, velocity, effort, stiffness, damping]"
)

# Remove old MJCF file
rm "${MJCF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to MJCF
ros2 run xacro xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${TMP_URDF_PATH}" &&
    compile "${TMP_URDF_PATH}" "${MJCF_PATH}" &&
    echo "Created new ${MJCF_PATH}"

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null
