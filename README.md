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
- goal 1 [video](https://drive.google.com/file/d/1V2AOpMV--nYkzrAXx7_JL740wGM34idO/view?usp=sharing)
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


- goal 2 [video](https://drive.google.com/file/d/1qGjE4D6dcIihuezuRkR_bug0DG96_m-M/view?usp=sharing)
    ```bash
    ros2 launch turtle_control goal_two_make_grid.launch.py
    ```
    1.Add acceleration profile:
    
    - In goal 1 turtlebot usually overshoots at high cmd_vel value and stops all of sudden as soon as zero cmd_vel is given, as it lacks the acceleration and deceleration profile. In goal 2 calculated velocity based on error is adjusted using the acceleration and deceleration limits. 
    - First desired velocity is calculated using the PID control equation by combining the proportional, Integral and derivative components by multiplying the error. linear acceleration is calculated by subtracting the current linear velocity from the desired linear velocity and dividing it by the time difference dt_. Which gives the change in velocity per unit time. Then the linear_velocity is updated by adding the product of linear acceleration and time difference dt_ to current linear velocity, it gives change in velocity over time period. 
        
    - After introducing the acceleration and deceleration profile system became over-damped, so required to retune the PID gains along with the acceleration and deceleration limits to minimize the time to reach goal and overshoot.
     
    - The acceleration limit can be enabled and disabled by setting the ros2 parameter “add_acce_deccl_limits” to true or false respectively.
    
    2.Grid pattern: 
    - Grid pattern coordinate generator function is implemented with grid size parameters setting to generate the list of grid coordinates, Then the ros2 action client is used to follow the list of waypoints received from generator function. 

- goal 3 [video](https://drive.google.com/file/d/1xrsIxseCZdNJLUkmZ-WhbiH-7mn4OdQb/view?usp=sharing)
    ```bash
    ros2 launch turtle_control goal_three_circle.launch.py
    ```
    In goal 3 to make turtlebot to rotate at given velocity and radius, “V=R*omega” equitation is used to determine the angular velocity. Ros node is implemented with the cmd_vel topic interface to publish the angular and linear velocity to the PID controller to make turtle1 move in a circular path. 

- goal 4 [video](https://drive.google.com/file/d/1DPNvyrDZKPXK0_zonfj_V2MQgUHE0qi1/view?usp=sharing)
    ```bash
    ros2 launch turtle_control goal_four_chase_fast.launch.py
    ```
    The chasing behavior is based on the received feedback with the current x,y position of RT from the action server’s feedBack callback. Turtle2 (PT) uses another instance of PID controller node to follow the rt_real_pose using the action client as soon as RT publishes the pose. 

- goal 5 [video](https://drive.google.com/file/d/1KpEdM3FA76LLlXoVWpyxnkSCgmz7Sa94/view?usp=sharing)
    ```bash
    ros2 launch turtle_control goal_five_chase_slow.launch.py
    ```
    In this goal the velocity of PT is restricted by setting the max_linear and angular velocity parameters to half of the RT. so it makes difficult and sometimes impossible to chase down the RT. So in order to fix this issue a chasing strategy is used to predict the RT path by using previously received rt_real_pose. To predict the RT movement path upcoming 3 poses are stored as evidance_pose_ vector. These 3 positions are used to calculate the radius and center coordinate.

  Algorithm to find radius and center coordinate is:
    - Calculate the midpoint of line joining p1-p2 and p2-p3 
    - Calculate the equation and slope of perpendicular bisector through the midpoints
       Find the intersection of these two bisectors. Which is center coordinates
    - Calculating the radius by calculating the distance between center and p1/p2/p3 coordinates
      Once the center and radius is calculated, the all circumference coordinates can be estimated at regular intervals. And these coordinate list used to make PR chase in opposite direction of RT.

- goal 6
    ```bash
    ros2 launch turtle_control goal_six_chase_noisy.launch.py
    ```
