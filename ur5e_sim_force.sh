ps aux | grep ros |  awk '{print $2}' | xargs kill -9;ps aux | grep rviz |  awk '{print $2}' | xargs kill -9&
roslaunch industry_library_robot camera_calibration_pub.launch&
sleep 1;roslaunch ur_e_gazebo ur5e.launch transmission_hw_interface:=hardware_interface/VelocityJointInterface&# transmission_hw_interfac:=hardware_interface/VelocityJointInterface& PositionJointInterface
sleep 3;roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=true&
sleep 2;roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true&
sleep 2;roslaunch gravity_compensation gravity_compensation_sim.launch&
