import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package Directories
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_robot_description = FindPackageShare(package='robot_description').find('robot_description')
    pkg_robot_launch = FindPackageShare(package='robot_launch').find('robot_launch')
    
    # Launch configuration variables specific to simulation
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to "false" to run headless.')
        
    declare_headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='false',
        description='Whether to run gazebo headless')
        
    declare_paused_cmd = DeclareLaunchArgument(
        name='paused',
        default_value='false',
        description='Start Gazebo paused')
        
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(pkg_robot_launch, 'worlds', 'robomaster_rale.world'),
        description='Full path to the world model file to load')

    # Robot description
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_robot_description, 'robot', 'cart_pole.urdf.xacro')
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Start Gazebo server and client
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world, ' -v 4']
        }.items()
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawn robot using ros_gz_sim
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                  '-name', 'cart_pole',
                  '-x', '0', '-y', '0', '-z', '1.225'],
        output='screen'
    )

    # Include robot control launch
    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_control'), '/launch/', 'robot_control.launch.py'
        ])
    )

    # Include commander launch
    commander_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('commander'), '/launch/', 'commander.launch.py'
        ])
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_paused_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(bridge)
    ld.add_action(spawn_entity_node)
    ld.add_action(robot_control_launch)
    ld.add_action(commander_launch)

    return ld