
# 代码使用说明

### 启动gazebo力控仿真
./ur5e_sim_force.sh
### force torque sensor calibraiton
roslaunch force_torque_sensor_calib example_ft_calib.launch
### gravity compensation for force torque sensor
roslaunch gravity_compensation gravity_compensation_sim.launch


### 摄像头标定
```
roslaunch industry_library_robot ur5e_calibration.launch
roslaunch industry_library_robot ur5e_calibration2.launch
```


### picking:
```
cd /home/ur5e/lee_ws/src/fmauch_universal_robot/ur_e_description/urdf
gedit ur5e.urdf.xacro // uncommon <origin xyz="0.0 0.0 0.12" rpy="${-pi} 0.0 ${-pi/2.0}" />
roslaunch industry_library_robot ur5e_real_picking.launch

cd /home/ur5e/lee_ws/src 
./run_yolo_and_gui.sh

rosrun industry_library_robot picking
```



### polishing:
#### 使用ur5e的时候，在示教板上手动连接，归零力传感器
```
cd /home/ur5e/lee_ws/src/fmauch_universal_robot/ur_e_description/urdf
gedit ur5e.urdf.xacro # uncommon <origin xyz="0.0 0.0 0.12" rpy="${-pi} 0.0 ${-pi/2.0}" />
roslaunch industry_library_robot ur5e_real_force_ctl.launch

cd /home/ur5e/lee_ws/src 
./run_yolo_and_gui.sh

rosrun ultrasound_robot wipe_blackboard # 使用ur5e的时候，在示教板上手动连接，归零力传感器

rosrun rqt_plot rqt_plot # monitor force sensor /wrench/
```
