import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('k1_slam')
    workspace_dir = os.path.abspath(os.path.join(package_dir, '..', '..', '..', '..'))
    maps_dir = os.path.join(workspace_dir, 'src', 'k1_maps')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(maps_dir, 'map1', 'map1.yaml'),
        description='Full path to the YAML map file to load'
    )
    params_file = os.path.join(package_dir, 'config', 'k1_localization.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'nav2_default_view.rviz')

    # MAP SERVER
    map_server = Node(
        package='nav2_map_server',  
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # MAP SERVER UPDATE
    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[params_file]
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # Localization SLAM TOOLBOX
    start_localization_slam_toolbox_node = Node(
        parameters=[
          params_file,
          {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        remappings=[
        ('/map', '/map')
        ],
        output='screen')

    # LIFECYCLE MANAGER
    lifecycle_manager_loc = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_localization',
    output='screen',
    parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'map_server',
                'map_saver',
                # 'amcl'
                # 'localization_slam_toolbox_node'
            ]
        }]
    )

    # RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_arg,
        map_server,
        map_saver,
        # amcl,
        start_localization_slam_toolbox_node,
        lifecycle_manager_loc,
        # rviz_node
    ])
