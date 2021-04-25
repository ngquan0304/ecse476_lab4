
---
# Mobile Robotics - Lab4

## Objectives
- To create a transform that relates the camera frame to the robot's torso frame
- To calibrate and validate coordinates of blokcs with respect to the torso frame, as seen by camera

## Command sets:

1. **To play jsp files with Baxter in Gazebo**

- E.g.: to show `arm1.pcd` and play `arm1.jsp` 

[Learning ROS ws] `roslaunch baxter_gazeo baxter_world.launch`

[Learning ROS ws] `roslaunch baxter_launch_files baxter_playfile_nodes.launch`

[This Repo ws] `rosrun pcd_utils display_pcd_files` then `arm1.pcd`

[This Repo ws] `rosrun baxter_playfile_nodes baxter_playback arm1.jsp`

1. **To show pcd files and demo block recognition in RViZ**

[Learning ROS ws] `roslaunch baxter_gazebo baxter_world.launch`

[Learning ROS ws] `roslaunch baxter_launch_files baxter_playfile_nodes.launch`

[This Repo ws] `roslaunch table_transform table_frame_wrt_cam_block.launch`

[This Repo ws] `roslaunch table_transform camera_frame_wrt_head_block.launch`

[This Repo ws] `rosrun table_transform find_block`

---
# Lab Result
## Part 1:
- Calibrated values for head/kinect_link using PCDs of arm: in `camera_frame_wrt_head.launch`
- Please check the calibrated images [HERE](https://drive.google.com/drive/folders/1OCkdHrOwKWOUz8bLaBRzT_fLfCAEopgL?usp=sharing).

## Part 2:
- For this part, we use a box filter to obtain only the pointcloud of the upper surface of the block to calculate the x,y coordinate of its centroid, with z = 0 w.r.t `table_frame`.
- Observed that eventhough the calibration from the first part can be perceived to be matched, the output errors are large.
- Decided to find coordinates twice:
  - Using the launch file made in first part: `camera_frame_wrt_head.launch`
    > \<node pkg="tf" type="static_transform_publisher" name="kinect_calib" args="0.166 0.005 0.123 0.007 1.425 0.0  head kinect_link 50"/>

  - Using a recalibrated launch file for this part: `camera_frame_wrt_head_block.launch`
    > \<node pkg="tf" type="static_transform_publisher" name="kinect_calib" args="0.171 0.023 0.123 0.015 1.425 0.0  head kinect_link 50"/>
---
### With `camera_frame_wrt_head.launch`
1. **block1.pcd**
```
- Translation: [0.498, -0.083, -0.185]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.353]
```
2. **block2.pcd**
```
- Translation: [0.639, -0.088, -0.185]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.350]
```
3. **block3.pcd**
```
- Translation: [0.631, -0.380, -0.180]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.350]
```
4. **block4.pcd**
```
- Translation: [0.727, -0.189, -0.183]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.349]
```
5. **block5.pcd**
```
- Translation: [0.637, -0.011, -0.186]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.346]
```
6. **block6.pcd**
```
- Translation: [0.458, -0.144, -0.185]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.346]
```
7. **block7.pcd**
```
- Translation: [0.778, -0.116, -0.184]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.344]
```
8. **block8.pcd**
```
- Translation: [0.795, -0.428, -0.179]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.344]
```
9.  **block9.pcd**
```
- Translation: [0.400, -0.427, -0.181]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.343]
```
10. **block10.pcd**
```
- Translation: [0.460, 0.071, -0.188]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.343]
```
11. **block11.pcd**
```
- Translation: [0.587, -0.403, -0.180]
- Rotation: in Quaternion [-0.007, -0.002, 0.038, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.076]
            in RPY (degree) [-0.844, -0.197, 4.340]
```
---
### With `camera_frame_wrt_head_block.lauch`
1. **block1.pcd**
```
- Translation: [0.503, -0.059, -0.185]
- Rotation: in Quaternion [-0.007, -0.002, 0.042, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.084]
            in RPY (degree) [-0.844, -0.197, 4.792]
```
2. **block2.pcd**
```
- Translation: [0.644, -0.062, -0.185]
- Rotation: in Quaternion [-0.007, -0.002, 0.042, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.084]
            in RPY (degree) [-0.844, -0.197, 4.802]
```
3. **block3.pcd**
```
- Translation: [0.641, -0.354, -0.180]
- Rotation: in Quaternion [-0.007, -0.002, 0.042, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.084]
            in RPY (degree) [-0.844, -0.197, 4.823]
```
4. **block4.pcd**
```
- Translation: [0.733, -0.161, -0.183]
- Rotation: in Quaternion [-0.007, -0.002, 0.042, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.084]
            in RPY (degree) [-0.844, -0.197, 4.835]
```
5. **block5.pcd:**
```
At time 12457.544
- Translation: [0.641, 0.015, -0.186]
- Rotation: in Quaternion [-0.007, -0.002, 0.042, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.085]
            in RPY (degree) [-0.844, -0.197, 4.864]
```
6. **block6.pcd:**
```
- Translation: [0.464, -0.121, -0.185]
- Rotation: in Quaternion [-0.007, -0.002, 0.043, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.085]
            in RPY (degree) [-0.844, -0.197, 4.891]
```
7. **block7.pcd:**
```
- Translation: [0.784, -0.088, -0.184]
- Rotation: in Quaternion [-0.007, -0.002, 0.043, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.086]
            in RPY (degree) [-0.844, -0.197, 4.907]
```
8. **block8.pcd:**
```
- Translation: [0.805, -0.400, -0.179]
- Rotation: in Quaternion [-0.007, -0.002, 0.043, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.086]
            in RPY (degree) [-0.844, -0.197, 4.920]

```
9. **block9.pcd:**
```
At time 12629.750
- Translation: [0.410, -0.405, -0.181]
- Rotation: in Quaternion [-0.007, -0.002, 0.043, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.087]
            in RPY (degree) [-0.844, -0.197, 4.959]
```
10. **block10.pcd:**
```
- Translation: [0.462, 0.094, -0.188]
- Rotation: in Quaternion [-0.007, -0.002, 0.043, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.087]
            in RPY (degree) [-0.844, -0.197, 4.988]
```
11. **block10.pcd:**
```
- Translation: [0.596, -0.379, -0.180]
- Rotation: in Quaternion [-0.007, -0.002, 0.045, 0.999]
            in RPY (radian) [-0.015, -0.003, 0.090]
            in RPY (degree) [-0.844, -0.197, 5.176]
```
---
### Remarks:
- The coordinates are within ± 14.0 mm for x-axis, ± 27mm for y-axis, and ± 5 mm for x-axis when using the arm pcd files.
- The coordinates are within ± 5 mm for the error on all axis when recalibrated using the given coordinates of block 5, 8, 9, and 10.
