import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    # 获取功能包路径
    nav2_pkg_dir = get_package_share_directory('hkc_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 配置文件路径
    nav2_params_path = os.path.join(nav2_pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 声明launch参数
    declare_map_path_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_pkg_dir, 'maps', 'zhanting.yaml'),
        # default_value=os.path.join(nav2_pkg_dir, 'maps', 'warehouse_navigation.yaml'),
        description='Full path to map file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 启动Navigation2核心节点
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_path
        }.items()
    )

    # 启动rviz2
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        declare_map_path_cmd,
        declare_use_sim_time_cmd,
        nav2_bringup_cmd,
        rviz_cmd
    ])