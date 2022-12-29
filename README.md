# RoboticArm


To replicate our work you need to follow the next sections. To use it directly go INSERT LINK

## Technical requirements
- Ubuntu 20.04
- ROS Noetic

## 1. Robot description
### 1.1 Create the ROS package for the robot description
```bash
cd catkin_ws/src
```

```bash
catkin_create_pkg project_robot_description roscpp tfgeometry_msgs urdf rviz xacro
```

```bash
sudo apt-get install ros-noetic-urdf
```

```bash
sudo apt-get install ros-noetic-xacro
```


Next, download all available folders in the repository. We will use the robot model.

```bash
git clone https://github.com/PacktPublishing/Mastering-ROS-for-Robotics-Programming-Third-edition.git
```

From the folder ``Chapter3/mastering_ros_robot_description_pkg`` copy all files to ``catkin_ws/src/project_robot_description``.


In this project we will use a seven-DOF robotic arm, showed in the next picture.

<p align="center"><img src="./readme_images/robot.png"/></p>


