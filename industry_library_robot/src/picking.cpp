#include "my_robot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "picking");
    ros::NodeHandle nh;

    Robot robot(nh);
    // ros::Rate loop_rate(1000);

    ros::spin();
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     robot.broadcast_tf();
    //     loop_rate.sleep();
    // }
    return 0;
}
