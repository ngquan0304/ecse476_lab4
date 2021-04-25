
---
# Mobile Robotics - Lab4

## Objectives
- To create a transform that relates the camera frame to the robot's torso frame
- To calibrate and validate coordinates of blokcs with respect to the torso frame, as seen by camera

## Command sets:

1. **To play jsp files with Baxter in Gazebo**

[Learning ROS ws] `roslaunch baxter_gazeo baxter_world.launch`

[Learning ROS ws] `roslaunch baxter_launch_files baxter_playfile_nodes.launch`

- Suppose we use `arm1.jsp`, Redirect to the directory contain the file.

[This Repo ws] `rosrun baxter_playfile_nodes baxter_playback arm1.jsp`

2. **To show pcd files and demo block recognition in RViZ**

[Learning ROS ws] `roslaunch baxter_gazebo baxter_world.launch`

[Learning ROS ws] `roslaunch baxter_launch_files baxter_playfile_nodes.launch`

[Don't use this] `rosrun table_transform find_table_frame`

[This Repo ws] `roslaunch table_transform table_frame_wrt_cam.launch`

[This Repo ws] `roslaunch table_transform camera_frame_wrt_head.launch`

[This Repo ws] `rosrun table_transform find_block`

---
# Result
## Part 1:
- Calibrated values for head/kinect_link: in `camera_frame_wrt_head.launch`
  
> \<node pkg="tf" type="static_transform_publisher" name="kinect_calib" args="0.166 0.03 0.123 0.102 1.425 0.0  head kinect_link 50"/>

## Part 2:
### **block5.pcd:**
    - Translation: [0.638, 0.012, -0.186]
    - Rotation: in Quaternion [-0.007, -0.002, 0.050, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.100]
            in RPY (degree) [-0.844, -0.197, 5.715]
### **block8.pcd:**
    - Translation: [0.806, -0.401, -0.179]
    - Rotation: in Quaternion [-0.007, -0.002, 0.050, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.100]
            in RPY (degree) [-0.844, -0.197, 5.715]
### **block9.pcd:**
    - Translation: [0.411, -0.410, -0.181]
    - Rotation: in Quaternion [-0.007, -0.002, 0.050, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.100]
            in RPY (degree) [-0.844, -0.197, 5.714]
### **block10.pcd:**
    - Translation: [0.459, 0.090, -0.188]
    - Rotation: in Quaternion [-0.007, -0.002, 0.050, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.100]
            in RPY (degree) [-0.844, -0.197, 5.714]

#### Remarks:
- The coordinates are within ± 6 mm compared to provided data.