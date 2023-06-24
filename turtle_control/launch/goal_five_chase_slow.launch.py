import launch
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

PARAM_FILE_NAME = "params_chaser_slow.yaml"

def generate_launch_description():
    ####################################################################
    # TURTLE 1 Robber
    ####################################################################
    pid_pose_controller_node = Node(
        package='turtle_control',
        executable='pid_pose_controller',
        output='screen',
        namespace="turtle1",
        parameters=[os.path.join(get_package_share_directory("turtle_control"), "config", PARAM_FILE_NAME)],
        # prefix=['xterm -e gdb -ex run --args'],
    )

    rotate_turtle_node = Node(
        name="rotate_turtle_goal_three",
        package='turtle_control',
        executable='rotate_circle',
        namespace="turtle1",
        output='screen',
        parameters=[os.path.join(get_package_share_directory("turtle_control"), "config", PARAM_FILE_NAME)],
        # prefix=['xterm -e gdb -ex run --args'],
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    
    ####################################################################
    # TURTLE 2 Police
    ####################################################################
    pid_pose_controller_two_node = Node(
        package='turtle_control',
        executable='pid_pose_controller',
        output='screen',
        namespace="turtle2",
        parameters=[os.path.join(get_package_share_directory("turtle_control"), "config", PARAM_FILE_NAME)],
        # prefix=['xterm -e gdb -ex run --args'],
    )
    
    chaser_node = Node(
        name="turtle_chaser_slow_node",
        package='turtle_control',
        executable='chase_turtle',
        namespace="turtle2",
        output='screen',
        parameters=[os.path.join(get_package_share_directory("turtle_control"), "config", PARAM_FILE_NAME)],
        remappings=[
            ('/turtle2/rt_real_pose', '/turtle1/rt_real_pose'),
        ]
        # prefix=['xterm -e gdb -ex run --args'],
    )

    delay_timer = TimerAction(
        period=10.0,  # Delay of 5 seconds
        actions=[ExecuteProcess(cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "{ x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"]),
                pid_pose_controller_two_node,
                chaser_node],
    )


    return launch.LaunchDescription([
        pid_pose_controller_node,
        turtlesim_node,
        rotate_turtle_node,
        delay_timer,
        
    ])

if __name__ == '__main__':
    launch_description = generate_launch_description()
    launch.launch(launch_description)