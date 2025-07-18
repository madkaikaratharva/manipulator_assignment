# Universal Robots Manipulator

Robot Specs: UR3e Robot. The URDF was modified an a vertical slider joint was added.

![link_rviz](media/photos/link_rviz.png)


![link_gazebo](media/photos/link_gazebo.png)


## Gazebo Setup, Simulation, and Trajectory planning using Moveit

Modified URDF configured properly, interfacing with gazebo and moveit.
Full video link: 

![moveit_planning](media/gifs/robot_setup_and_planning_moveit_gui.gif)


To run the gazebo simulation, launch the controllers, and the movegroup interface
```bash
# In your workspace
ros2 launch manipulator_bringup simulation.launch.py use_sim_time:=true
```

To plan and execute trajectories using Moveit's rviz GUI
```bash
# In your workspace
ros2 launch manipulator_moveit moveit_rviz.launch.py
```


 
