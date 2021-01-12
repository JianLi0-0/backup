
# 代码使用说明
自写代码：industry_library_robot, ohters
其余均为开源代码
### 启动gazebo力控仿真
./ur5e_sim_force.sh
### force torque sensor calibraiton
roslaunch force_torque_sensor_calib example_ft_calib.launch
### gravity compensation for force torque sensor
roslaunch gravity_compensation gravity_compensation_sim.launch