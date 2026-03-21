from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='robot.urdf.xacro',
        )
    )

    description_file = LaunchConfiguration('description_file')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("control_one_motor_pkg_ex"),
                    "description",
                    "urdf",
                    description_file,
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("control_one_motor_pkg_ex"),
            "config",
            "hexapod_controller.yaml",
        ]
    )

    print(f"Loading controllers from: {robot_controllers}")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        arguments=['--ros-args', '--log-level', 'info'],
        
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager"],
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trajectory_controller", "-c", "/controller_manager"],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("control_one_motor_pkg"), "rviz", "rviz.rviz"]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=20.0,
                    actions=[velocity_controller_spawner,
                             trajectory_controller_spawner],
                )]
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # velocity_controller_spawner,
        # trajectory_controller_spawner,
        delay_controller,
        # rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

# ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
# ros2 topic pub --once /trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{points: [{positions: [10000, 5000], time_from_start: {sec: 2, nanosec: 0}}], joint_names: ["joint_1","joint_2"]}'
# ros2 topic pub --once /trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{points: [{positions: [10000], time_from_start: {sec: 2, nanosec: 0}}], joint_names: ["joint_1"]}'


# ros2 service call /ethercat_manager/get_sdo ethercat_msgs/srv/GetSdo "{master_id: 0, slave_position: 0, sdo_index: 0x6041, sdo_subindex: 0, sdo_data_type: 'uint16'}"
# ros2 service call /ethercat_manager/set_sdo ethercat_msgs/srv/SetSdo "{master_id: 0, slave_position: 0, sdo_index: 0x6040, sdo_subindex: 0, sdo_data_type: 'uint16', sdo_value: 15}"
# watch -n 0.1 "ethercat upload -p 0 -t uint32 0x60FD 0"
# ros2 topic echo /trajectory_controller/state
# ros2 control set_controller_state trajectory_controller active
