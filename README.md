# SimpleUnityROSSimulation
Creating a simple Unity project to interface with ROS (and possibly ROS2). This is currently a work in progress as I figure out what I want to do. More details will be added later on.

## Development Environment
* OS: Ubuntu 20.04
* Unity version: Unity 2020.3.10f1
* ROS version: ROS Noetic

### ROS Setup
1. [Create](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) a ROS workspace.
2. Add the [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) package to the src folder in your workspace.
3. For now, the [universal_robot](https://github.com/ros-industrial/universal_robot) repository is also included in this workspace.
4. Run `catkin_make` in the root folder of your workspace.

## Instructions for use
### ROS
1. In your terminal ensure ROS is sourced properly:
```
source /opt/ros/<distro>/setup.bash
```
2. Your ROS workspace with the ros_tcp_endpoint package should also be sourced:
```
source ~/path/to/ros_ws/devel/setup.sh
```
3. Navigate to the ROS folder in this repo and run the launch file. This folder is not a package (yet), so the exact path will need to be specified. 
```
roslaunch test.launch
```
This launches the endpoint and [robot_state_publisher](http://wiki.ros.org/robot_state_publisher) nodes.

4. Launch rviz using the following command: `rosrun rviz rviz`. When RViz opens, open the ROS/ur5e_visualization.rivz file from this repo (File > Open config). 


### Unity
1. Open the SimpleROSArm project in Unity.
2. Open the Assets/Scenes/SampleScene scene. This scene has a robot that is able to publish its joint state.
3. In the Editor, select the ur5e prefab. In the `RobotController` component, you can set the robot's joint goals. The joint state should be updated in RViz. 

## External Resources
* Unity [URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer) - used to import the robots into Unity

* [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) repository - all URDFs and associated meshes were obtained from here