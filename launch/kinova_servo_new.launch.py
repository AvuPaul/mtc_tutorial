# Copyright (c) 2023 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# launch commmand:
# ros2 launch mtc_tutorial kinova_servo.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true

import os
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")

    launch_arguments = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "7",
        "gripper_max_velocity": gripper_max_velocity,
        "gripper_max_force": gripper_max_force,
        "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
    }

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings=launch_arguments)
        # .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("mtc_tutorial", "config/kinova_simulated_config.yaml")
    servo_params = {"mtc_tutorial": servo_yaml}

    # print("servo_params:", servo_params)

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context) == "true"})

    # move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #         moveit_config.to_dict(),
    #     ],
    # )

    # RViz
    rviz_config_file = (
        get_package_share_directory("mtc_tutorial") + "/rviz/gen3_servo.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # # Static TF
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    # )

    # # Publish TF
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[
    #         moveit_config.robot_description,
    #     ],
    # )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # parameters=[moveit_config.to_dict(), ros2_controllers_path],
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        # arguments=[
        #     "joint_state_broadcaster",
        #     "--controller-manager-timeout",
        #     "300",
        #     "--controller-manager",
        #     "/controller_manager",
        # ],
    )

    # robot_pos_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),
            ComposableNode(
                package="moveit_servo_kinova",
                plugin="moveit_servo_kinova::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo_kinova",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    nodes_to_start = [
        ros2_control_node,
        # robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        # robot_pos_controller_spawner,
        # robot_hand_controller_spawner,
        # fault_controller_spawner,
        # move_group_node,
        # static_tf,
        servo_node,
        container,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Max velocity for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use arm's internal gripper connection",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_external_cable",
            default_value="false",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
