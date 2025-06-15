from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the package
    pkg_share = FindPackageShare('ghostswarm_robots').find('ghostswarm_robots')
    
    # World file
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'adversarial_environment.world'])
    
    # Gazebo launch
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    # Spawn UAV
    spawn_uav = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'uav1',
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '1.0',
                  '-file', PathJoinSubstitution([pkg_share, 'models', 'uav', 'model.sdf']),
                  '-reference_frame', 'world',
                  '-robot_namespace', 'uav1'],
        output='screen'
    )
    
    # Spawn UGV
    spawn_ugv = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ugv1',
                  '-x', '2.0',
                  '-y', '0.0',
                  '-z', '0.1',
                  '-file', PathJoinSubstitution([pkg_share, 'models', 'ugv', 'model.sdf']),
                  '-reference_frame', 'world',
                  '-robot_namespace', 'ugv1'],
        output='screen'
    )
    
    # UAV Agent
    uav_agent = Node(
        package='ghostswarm_robots',
        executable='uav_agent.py',
        name='uav_agent',
        namespace='uav1',
        output='screen',
        parameters=[{'agent_id': 'uav1'}]
    )
    
    # UGV Agent
    ugv_agent = Node(
        package='ghostswarm_robots',
        executable='ugv_agent.py',
        name='ugv_agent',
        namespace='ugv1',
        output='screen',
        parameters=[{'agent_id': 'ugv1'}]
    )
    
    # Coordinator
    coordinator = Node(
        package='ghostswarm_robots',
        executable='coordinator.py',
        name='coordinator',
        output='screen'
    )
    
    # RViz
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'default.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_uav,
        spawn_ugv,
        uav_agent,
        ugv_agent,
        coordinator,
        rviz
    ])
