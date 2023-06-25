# flytbase-turtlesim

## Setup

- Install ROS2 humble or galactic

## Package setup 
- create ROS 2 workspace
    ```bash
    mkdir -p ~/flytbase_ws/src && cd ~/flytbase_ws/src
    ```
- clone repo 
    ```bash
    git clone git@github.com:chiragmakwana0296/flytbase-turtlesim.git
    ```

- build workspace

    ```bash
    cd ~/flytbase_ws && colcon build
    ```
- source workspace (every time new terminal instance is launched or add to ~/.bashrc)

    ```bash
    source install/setup.bash
    ```

## Run nodes
- goal 1 
    ```bash
    ros2 launch turtle_control pid_pose_controller.launch.py
    ```

- goal 2
    ```bash
    ros2 launch turtle_control goal_two_make_grid.launch.py
    ```

- goal 3 
    ```bash
    ros2 launch turtle_control goal_three_circle.launch.py
    ```

- goal 4 
    ```bash
    ros2 launch turtle_control goal_four_chase_fast.launch.py
    ```

- goal 5 
    ```bash
    ros2 launch turtle_control goal_five_chase_slow.launch.py
    ```

- goal 6 
    ```bash
    ros2 launch turtle_control goal_six_chase_noisy.launch.py
    ```
