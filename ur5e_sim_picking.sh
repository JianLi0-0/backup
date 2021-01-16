ps aux | grep ros |  awk '{print $2}' | xargs kill -9;ps aux | grep rviz |  awk '{print $2}' | xargs kill -9&
sleep 1;
roslaunch ur_e_gazebo ur5e.launch transmission_hw_interface:=hardware_interface/PositionJointInterface&
sleep 2;
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=true&
sleep 1;
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true&
roslaunch realsense2_camera rs_rgbd.launch&
sleep 1;
roslaunch industry_library_robot camera_calibration_pub.launch;
