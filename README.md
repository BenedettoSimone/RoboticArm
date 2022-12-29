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

### 1.2 Attach an RGB camera
To do that you need to modify the file ``project_robot_description/urdf/seven_dof_arm.xacro`` adding the following code. The first elements of this block are an extra link and joint added to the URDF file that represents the camera.
```XML
<link name="camera_link">
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="${cameraSize} ${cameraSize} ${cameraSize}"/>
        </geometry>
      </collision>

      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="${cameraSize} ${cameraSize} ${cameraSize}"/>
        </geometry>
        <material name="green"/>
      </visual>

      <inertial>
        <mass value="${cameraMass}" />
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <box_inertia m="${cameraMass}" x="${cameraSize}" y="${cameraSize}" z="${cameraSize}" />
        <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
      </inertial>
    </link>

    <joint name="camera_joint" type="fixed">
      <axis xyz="0 1 0" />
      <origin xyz=".2 0 0" rpy="0 0 0"/>
      <parent link="wrist_pitch_link"/>
      <child link="camera_link"/>
    </joint>

```
You need also to define these XACRO properties.
```XML
<!--Camera-->
<xacro:property name="cameraSize" value="0.05"/>
<xacro:property name="cameraMass" value="0.1"/>
```

After, create the file ``seven_dof_arm.gazebo`` in the folder ``project_robot_description/urdf`` inserting the following code. With this code we can use the Gazebo plugin that gives us the camera functionality and publishes the image to a ROS message.

```XML
<?xml version="1.0"?>
<robot>
 <gazebo reference="camera_link">
    <material>Gazebo/Green</material>
    <sensor type="camera" name="camera">
      <update_rate>30.0</update_rate>
      <visualize></visualize>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>seven_dof_arm/camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```
And finally you need to include the file in the URDF model.
```XML
<xacro:include filename="$(find project_robot_description)/urdf/seven_dof_arm.gazebo"/>
```
```
roslaunch project_gazebo seven_dof_arm_world.launch
rostopic list
rosrun image_view image_view image:=/seven_dof_arm/camera/image_raw

```



