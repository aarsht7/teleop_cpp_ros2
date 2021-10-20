# teleop_cpp_ros2
C++ Implementation of the Generic Keyboard Teleop for ROS2: https://github.com/ros-teleop/teleop_twist_keyboard

## Features

This particular implementation does away with keeping the history of previous speed settings, and heavily cuts down on the amount of printing that is done to the terminal via the use of carriage returns (\r).

Furthermore, the last command that was sent is reflected, and invalid commands are identified as such.



## Installing the Package

As per standard ROS practice, make a workspace, go to the workspace's src directory, and clone this repository, then run catkin_make in the root of the workspace, and source the resulting setup.bash!

```bash
git clone https://github.com/1at7/teleop_cpp_ros2.git
cd ..
colcon build --packages-select teleop_cpp_ros2

source install/setup.bash
```



## Running the Node

```bash
source the ros2 worksapce and

# In terminal, run
ros2 run teleop_cpp_ros2 teleop

# If you want to see the outputs, check the /cmd_vel topic
ros2 topic echo /cmd_vel
```



## Usage

Same as the original + some addons

```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
---------------------------
Simple Teleoperation with arrow keys
          ⇧
        ⇦   ⇨
          ⇩

          A
        D   C
          B
This increases/decreases speed linearly.
---------------------------
t : up (+z)
b : down (-z)
s/S : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
NOTE: Increasing or Decreasing will take affect live on the moving robot.
      Consider Stopping the robot before changing it.
      
CTRL-C to quit
```



------
