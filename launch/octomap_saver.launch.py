from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    save_directory_arg = DeclareLaunchArgument(
        'save_directory',
        default_value='/home/rafilappy/ws_moveit_forked/src/custom_plan/octomaps',
        description='Directory to save/load octomaps'
    )
    
    default_filename_arg = DeclareLaunchArgument(
        'default_filename',
        default_value='octomap',
        description='Base filename for saved octomaps'
    )
    
    # Octomap saver node
    octomap_saver_node = Node(
        package='custom_plan',
        executable='octomap_saver',
        name='octomap_saver',
        output='screen',
        parameters=[{
            'save_directory': LaunchConfiguration('save_directory'),
            'default_filename': LaunchConfiguration('default_filename'),
        }],
        remappings=[
            # Remap if your octomap topic is different
            # ('/planning_scene_world/octomap', '/your_octomap_topic'),
        ]
    )
    
    return LaunchDescription([
        save_directory_arg,
        default_filename_arg,
        octomap_saver_node,
    ])
