# Framework for Trackless Robocoasters
This repo maintains the entire catkin workspace for the project. 
NOTE: ROS Melodic, Python 2.7

Motion Planning : RBE550 at Worcester Polytechnic Institute
- Victoria Heffern, vjheffern@wpi.edu

## Repository Guide
- src
  - kuka_kr560_desc
    - meshes : contains collision (stl files) and visual (dae files)
    - urdf : contains all of the xacro files for kuka arm
  - robocoaster_control: contains planners, plotting
    - src
      - arm_rrt_planner : RRT for kuka arm
      - robocoaster_telemetry : manages the plotting of g-forces and accel on EE
      - RRT_planner : RRT for chassis
    - urdf : main overview xacro
    - config: yaml files
    - launch: launch files (gazebo.launch is the main one)
  - robocoaster_gazebo: contrains gazebo files
    - worlds : contains the large floor/building plan
  
