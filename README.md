# Mobile Robotics - Lab4

## Objective
- To create a transform that relates the camera frame to the robot's torso frame
- to calibrate and validate coordinates of blokcs with respect to the torso frame, as seen by camera

## Part 1: Depth camera extrinsic calibration
At workspace of learning_ros package:

- Run gazebo simulation of baxter file.

`roslaunch baxter_gazebo baxter_world.launch`

- Run the playfiles of baxter to relocate .

`roslaunch baxter_launch_files baxter_playfiles_nodes.launch`

From here on, suppose we are at the directory contain pcd files and want to use `arm1.pcd` along with `arm1.jsp`. Both are currently in `src/ecse476_lab4/table_transform/arm_images`

- Display pcd file.

`rosrun pcl_utils display_pcd_file` then `arm1.pcd`

'src/ecse476_lab4/table_transform/arm_images/arm2.pcd'

- In RViz, add pointcloud2 that subsribes to `/pcd`, and change the global options' frame to `camera_depth_optical_frame` to check if the pointcloud can be seen.

- Find and create a launch file to publish the transformation of the table w.r.t. to the camera

`rosrun table_transform find_table_frame` (can stop the node after done)

- Publish the transformation above

`roslaunch table_transform table_frame_wrt_cam.launch`

- Publish the transformation from the camera to the head of the robot

`roslaunch table_transform camera_frame_wrt_head.launch`

(make sure to keep the global setting at `torso` to observe the robot normally)

- Move the robot right arm to the position in `arm1.pcd` (currentl directory must be at arm_images. Can access the folder by `roscd table_transform`, then `cd arm_images`)

`rosrun baxter_playfile_nodes baxter_playback arm1.jsp`