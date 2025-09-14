from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Load controller configuration
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("robot_control"), "config", "controller.yaml"]
    )

    ros2_control = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "robot", "cart_pole.ros2_control.xacro"]
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Spawn cart_controller
    cart_controller_spawner = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=["cart_controller"],
        output="screen",
    )

    #ros2 control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_control, robot_controllers],
        output="screen",
    )


    return LaunchDescription([
        joint_state_broadcaster_spawner, 
        cart_controller_spawner,
        ros2_control_node,
    ])