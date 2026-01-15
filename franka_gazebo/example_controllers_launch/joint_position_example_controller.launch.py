# Copyright (c) 2025
# MIT License

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    srdf_file = LaunchConfiguration("srdf_file")

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "hand:=true ",
            "sim_ignition:=true ",
            "simulation_controllers:=",
            controllers_yaml,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }
    robot_description_semantic_content = Command(
        [
            "cat ",
            PathJoinSubstitution(
                [
                    "/home/ohheemin/gazebo_ws/src/edu-franka_simulation/panda_config",
                    srdf_file,
                ]
            ),
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content,
            value_type=str,
        )
    }
    kinematics_yaml = PathJoinSubstitution(
        [
            "/home/ohheemin/gazebo_ws/src/edu-franka_simulation/panda_config",
            "kinematics.yaml",
        ]
    )
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                              "default_planner_request_adapters/ResolveConstraintFrames "
                              "default_planner_request_adapters/FixWorkspaceBounds "
                              "default_planner_request_adapters/FixStartStateBounds "
                              "default_planner_request_adapters/FixStartStateCollision "
                              "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_config = {
        "panda_arm": {
            "planner_configs": ["RRTConnect", "RRT", "RRTstar", "TRRT", "PRM"],
            "projection_evaluator": "joints(panda_joint1,panda_joint2)",
            "longest_valid_segment_fraction": 0.005,
        }
    }
    moveit_controllers = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["joint_position_example_controller"],
            "joint_position_example_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [
                    "panda_joint1",
                    "panda_joint2",
                    "panda_joint3",
                    "panda_joint4",
                    "panda_joint5",
                    "panda_joint6",
                    "panda_joint7",
                    "panda_finger_joint1",
                    "panda_finger_joint2"
                ],
            },
        },
    }

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=PathJoinSubstitution(
            [FindPackageShare("franka_description")]
        ),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            robot_description,
            robot_description_semantic,
        ],
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": "-r -v 1 empty.sdf"
        }.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "panda_arm",
            "-allow_renaming",
            "true",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_position_example_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_position_example_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    move_group_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    ompl_planning_pipeline_config, 
                    ompl_planning_config,            
                    moveit_controllers,
                    {
                        "publish_robot_description": True,
                        "publish_robot_description_semantic": True,
                        "publish_planning_scene": True,
                        "publish_geometry_updates": True,
                        "publish_state_updates": True,
                        "publish_transforms_updates": True,
                        "monitor_dynamics": False,
                    },
                ],
            )
        ],
    )
    moveit_pick_place_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="arm_controllers",
                executable="moveit_pick_place_node",
                name="moveit_pick_place",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                ],
            )
        ],
    )

    return [
        set_gazebo_model_path,
        gz_launch,
        robot_state_publisher_node,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        joint_position_example_controller_spawner,
        move_group_node,
        moveit_pick_place_node,
    ]


def generate_launch_description():

    declared_arguments = [

        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="franka_gazebo",
            description="Package with ros2_control config",
        ),

        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="ros2_control controller yaml",
        ),

        DeclareLaunchArgument(
            "description_package",
            default_value="franka_description",
            description="URDF/XACRO package",
        ),

        DeclareLaunchArgument(
            "description_file",
            default_value="panda_arm.urdf.xacro",
            description="URDF XACRO file",
        ),

        DeclareLaunchArgument(
            "srdf_file",
            default_value="panda.srdf",
            description="SRDF file (from panda_config)",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )