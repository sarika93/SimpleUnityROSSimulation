# SimpleUnityROSSimulation
Creating a simple Unity project to interface with ROS1 (and possibly ROS2). This is currently a work in progress as I figure out what I want to do. More details will be added later on.

## Development Environment
* OS: Ubuntu 20.04
* Unity version: Unity 2020.3.10f1
* ROS version: ROS Noetic

## Cloning the repo
This repository has sub-modules so you will need to run: `git clone --recurse-submodules -j8 [INSERT LINK]`

### ROS Setup
1. Navigate to the ROS1 folder in this repository, which is a ROS workspace: `cd ROS1/`.
4. Run `catkin_make` here.

## Instructions for use
### ROS
1. In your terminal ensure ROS is sourced properly:
```
source /opt/ros/<distro>/setup.bash
```
2. Your ROS workspace with the ros_tcp_endpoint package should also be sourced:
```
source ~/path/to/ROS1/devel/setup.sh
```
3. From any folder run the launch file: 
```
roslaunch unity_ros_demo test.launch
```
This launches the endpoint and [robot_state_publisher](http://wiki.ros.org/robot_state_publisher) nodes, as well as rviz. 


### Unity
1. Open the SimpleROSArm project in Unity.
2. Open the Assets/Scenes/SampleScene scene. This scene has a robot that is able to publish its joint state.
3. In the Editor, select the ur5e prefab. In the `RobotController` component, you can set the robot's joint goals. The joint state should be updated in RViz. 

## External Resources
* Unity [URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer) - used to import the robots into Unity

* [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) repository - all URDFs and associated meshes were obtained from here