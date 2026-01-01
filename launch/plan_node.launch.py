from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config =  MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config").to_dict()


    # MTC Demo node
    pick_place_demo = Node(
        # package="mtc_tutorial",
        # executable="mtc_tutorial",
        package="custom_plan",
        executable="plan_node",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])