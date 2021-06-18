#include "my_robot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "picking");
    ros::NodeHandle nh;

    Robot robot(nh);
    // ros::Rate loop_rate(1000);
    ros::Duration(0.3).sleep();
    // robot.joint_position_control(robot.home_angles.data, 3.0);

    ros::spin();
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     robot.broadcast_tf();
    //     loop_rate.sleep();
    // }
    return 0;
}
