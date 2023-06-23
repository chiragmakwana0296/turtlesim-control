import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pid_pose_controller_node = Node(
        package='turtle_control',
        executable='pid_pose_controller',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("turtle_control"), "config", 'params.yaml')],
        # prefix=['xterm -e gdb -ex run --args'],
    )

    rotate_turtle_node = Node(
        name="rotate_turtle_goal_three",
        package='turtle_control',
        executable='rotate_circle',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("turtle_control"), "config", 'params.yaml')],
        # prefix=['xterm -e gdb -ex run --args'],
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    

    return launch.LaunchDescription([
        pid_pose_controller_node,
        turtlesim_node,
        rotate_turtle_node
    ])

if __name__ == '__main__':
    launch_description = generate_launch_description()
    launch.launch(launch_description)