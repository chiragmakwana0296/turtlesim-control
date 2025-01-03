import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pid_pose_controller_node = Node(
        package='turtle_control',
        executable='pid_pose_controller',
        namespace='turtle1',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("turtle_control"), "config", 'params_acceleration.yaml')],
        # prefix=['xterm -e gdb -ex run --args'],
    )
    
    follow_grid_node = Node(
        package='turtle_control',
        executable='follow_grid',
        namespace='turtle1',
        # prefix=['xterm -e gdb -ex run --args'],

    )
    
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )

    return launch.LaunchDescription([
        pid_pose_controller_node,
        follow_grid_node,
        turtlesim_node
        
    ])

if __name__ == '__main__':
    launch_description = generate_launch_description()
    launch.launch(launch_description)