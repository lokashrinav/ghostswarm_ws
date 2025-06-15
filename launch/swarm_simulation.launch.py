from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')
    world = LaunchConfiguration('world', default='empty.world')
    
    # Gazebo server
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver',
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world],
        output='screen'
    )

    # Gazebo client
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(PythonExpression(['not ', headless]))
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[LaunchConfiguration('model')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode if true'),
            
        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='Gazebo world file'),
            
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='Path to robot URDF file'),
            
        DeclareLaunchArgument(
            'rvizconfig',
            default_value='',
            description='Path to RViz config file'),
            
        gzserver_cmd,
        gzclient_cmd,
        rviz_node,
        robot_state_publisher,
    ])
