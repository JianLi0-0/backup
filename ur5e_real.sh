ps aux | grep ros |  awk '{print $2}' | xargs kill -9;ps aux | grep rviz |  awk '{print $2}' | xargs kill -9&
roslaunch industry_library_robot camera_calibration_pub.launch&
sleep 1;roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.3&
# sleep 4;roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch&
# sleep 3;roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true;
