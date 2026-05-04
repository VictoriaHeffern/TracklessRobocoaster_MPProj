# Framework for Trackless Robocoasters
This repo maintains the entire catkin workspace for the project. 

Motion Planning : RBE550 at Worcester Polytechnic Institute
- Victoria Heffern, vjheffern@wpi.edu

NOTE: ROS Melodic, Python 2.7

## Repository Guide
- src
  - kuka_kr560_desc
    - meshes : contains collision (stl files) and visual (dae files)
    - urdf : contains all of the xacro files for kuka arm
  - robocoaster_control: contains planners, plotting
  - robocoaster_gazebo: contrains gazebo files and main launch
