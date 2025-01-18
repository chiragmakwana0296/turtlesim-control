# flytbase-turtlesim

## Setup

- Install ROS2 humble or galactic

## Package setup 
- create ROS 2 workspace
    ```bash
    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    ```
- clone repo 
    ```bash
    git clone git@github.com:chiragmakwana0296/turtlesim-control.git
    ```

- build workspace

    ```bash
    cd ~/ros2_ws && colcon build
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
    In goal 1 PID controller is implemented to make turtlebot reach to a given pose by controlling linear & angular velocity in minimum time preventing the overshoot from goal.
  Algorithm:
- Retrieve the PID gain values through ros2 parameters in runtime. 
- Update the integral of the error by adding the current error value to error_integral_.
- Calculate the derivative of the error by subtracting the previous error (last_error_) from the current error.
- Update the previous error value (last_error_) with the current error value.
- Retrieve the target position (target_x and target_y) and current position (current_x and current_y) from the appropriate objects.
- Calculate the angle to the goal using the atan2 function.
- If the add_acce_deccl_limits_ flag is true, perform the following steps:
- Calculate the desired linear velocity using the PID controller equation.
- Retrieve the current linear velocity from the appropriate object.
- Calculate the linear acceleration based on the difference between the desired and current linear velocities.
- Limit the linear acceleration to the maximum acceleration and deceleration values.
- Update the linear velocity by adding the product of the linear acceleration and time difference (dt) to the current linear velocity.
- If the add_acce_deccl_limits_ flag is false, calculate the linear velocity using the PID controller equation without considering acceleration and deceleration limits.
- Limit the linear velocity to the maximum linear velocity limit.
- Calculate the error in the heading angle (error_theta) by subtracting the current pose's theta from the angle to the goal.
- Normalize the error in the heading angle to the range of [-3.14, 3.14] using while loops.
- Calculate the derivative of the heading angle error by subtracting the previous heading error (last_error_theta_) from the current heading error.
- Update the previous heading error value (last_error_theta_) with the current heading error value.
- Update the integral of the heading error by adding the current heading error to error_integral_theta_.
- Calculate the angular velocity using the PID controller equation for the heading angle.
- Limit the angular velocity to the maximum angular velocity limit.
- Create and return a std::pair object containing the calculated linear velocity and angular velocity.


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
