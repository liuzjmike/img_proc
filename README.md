# Image Processor

## Run the program
rosrun img_proc img_proc /sim/rrbot/camera1/image_raw /sim/rrbot/joint_states  
rosrun img_proc record_controller /sim/rrbot/ joint1 joint2  
rosrun actionlib axclient.py /record  
rosrun rviz rviz  
(The above commands are included in calibrate.launch)
rosservice call /compute_cost /camera1/rviz_camera_pub/image_raw /broken/rrbot/joint_states