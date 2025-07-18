# Universal Robots Manipulator

## System
* Ubuntu 22.04
* ROS Humble
* Gazebo Ignition


## Robotic Arm

Robot Specs: UR3e robot. The URDF was built from scratch using the official mesh and collision models, with modifications to include a vertical prismatic (slider) joint.

![link_rviz](media/photos/link_rviz.png)

![link_gazebo](media/photos/link_gazebo.png)


## Gazebo Setup, Simulation, and, Trajectory planning using Moveit

The modified URDF was properly configured and successfully integrated with both Gazebo and MoveIt.
Full Video Link: [setup_link](https://drive.google.com/file/d/1mLRTUpEMEnchoLMJXQKBJcATHelnPKRM/view?usp=sharing)

![moveit_planning](media/gifs/robot_setup_and_planning_moveit_gui.gif)


To run the gazebo simulation, launch the controllers, and the movegroup interface
```bash
# source your workspace
ros2 launch manipulator_bringup simulation.launch.py use_sim_time:=true
```

To plan and execute trajectories using Moveit's rviz GUI
```bash
# source your workspace
ros2 launch manipulator_moveit moveit_rviz.launch.py
```


## Custom Goal, trajectory planning, and velocities profiles

In this C++ node, two target goal poses were defined, and a trajectory was generated from the home position to Pose_1, and then to Pose_2. Time parameterization was applied between trajectory points to achieve trapezoidal velocity profiles.

Full Video Link:[velocity_profile_1](https://drive.google.com/file/d/1yXJh0pUoaGdy2gCkndLf5UreJuGtQ1Jj/view?usp=sharing)


![planning_and_execution](media/gifs/trapezoidal_velocities_goal_planning_execution.gif)

![velocity_profile_1](media/photos/trapezoidal_profile_1.png)


To run this test example
```bash
# source your workspace and launch the gazebo simulation
ros2 launch manipulator_bringup simulation.launch.py use_sim_time:=true
`
# Launch the Plan and Execute Node
ros2 run manipulator_controller plan_and_execute
``` 


## Waypoint definition and Cartesian Motion

In this C++ node, a set of 8 predefined points was used to simulate small movements along the X-Y plane. The manipulator executed these movements while adjusting its vertical slider as needed (end effector only moving along the X-Y axis). Time-parameterized trajectories were used to generate trapezoidal velocity profiles; however, the results were less precise and smooth for shorter trajectories.

Full Video Link:[velocity_profile_2](https://drive.google.com/file/d/11uBrrSfHUE7bH3j_yI8hV7jpzi8ncU9v/view?usp=sharing)

![waypointfollowing](media/gifs/velocity_profiles_cartesian_points.gif)

To run this test example
```bash
# source your workspace and launch the gazebo simulation
ros2 launch manipulator_bringup simulation.launch.py use_sim_time:=true
`
# Launch the Plan and Execute Node
ros2 run manipulator_controller waypoint_following
``` 


**Possible solutions:**
I am planning to implement a custom velocity and acceleration profiles that follow a trapezoidal structure.
Example: Say I have 4 points - a, b, c, and d
* Acceleration phase (a → b): Gradually increasing velocity to reach a specified maximum.
* Constant velocity phase (b → c): Maintaining steady speed across the main portion of the trajectory.
* Deceleration phase (c → d): Smoothly reducing velocity to a stop.


I have also provided the Google Drive link to all the videos for this assignment. I will keep on adding more videos and update the repository if I am able to workout better solutions as well.
Drive Link: [drive_link](https://drive.google.com/drive/folders/1h_jmQnbTA07PygvYllR8yE4lVmgx2R0T?usp=sharing)





 
