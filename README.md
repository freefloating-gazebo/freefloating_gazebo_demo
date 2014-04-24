freefloating_gazebo_demo
========================

This package contains an example of underwater robot using the freefloating_gazebo plugin.

The example can be run in three steps:

- Synchronize UWsim and Gazebo files: roslaunch freefloating_gazebo_demo g500arm5e.launch parse:=true

Will scan the launchfile to create urdf from xacro files to be used in UWsim

- UWsim: roslaunch freefloating_gazebo_demo g500arm5e.launch 

Launches only UWsim with body and joint position topics

- Gazebo without gui: roslaunch freefloating_gazebo_demo g500arm5e_gazebo.launch uwsim:=true

Launches Gazebo to allow dynamic simulation. Gazebo is launched with the freefloating_gazebo_fluid and freefloating_gazebo_control plugins.
A pid_control node is also launched and allows position and velocity control of the robot body and joints.
