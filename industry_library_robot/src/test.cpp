#include "my_robot.h"
// #include <boost/property_tree/json_parser.hpp>
// #include <boost/property_tree/ptree.hpp>
#include "utils/utils.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace pt = boost::property_tree;

// template<typename T>
// std::vector<T> as_vector(boost::property_tree::ptree const & ptIn, boost::property_tree::ptree::key_type const & key)
// {
//   std::vector<T> r;
//   for(auto & item : ptIn.get_child(key)) r.push_back(item.second.get_value<T>());
//   return r;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    Robot robot(nh);
    ros::Rate loop_rate(1000);

	pt::ptree root;
	pt::read_json("/home/qwe/lee_ws/src/industry_library_robot/json_config/robot_config.json", root);

    // ros::spin();
    while (ros::ok())
    {
        // ros::spinOnce();
        // robot.broadcast_tf();
        // loop_rate.sleep();

        geometry_msgs::Pose cartesian_position;
        std::vector<double> position(as_vector<double>(root, "force_ctl.pos"));
        cartesian_position.position.x = position[0];
        cartesian_position.position.y = position[1];
        cartesian_position.position.z = position[2];

        std::vector<double> orient(as_vector<double>(root, "force_ctl.orient"));
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(orient[0], orient[1], orient[2]);
 
        geometry_msgs::Quaternion grasping_orientation;
        grasping_orientation.x = myQuaternion.x();
        grasping_orientation.y = myQuaternion.y();
        grasping_orientation.z = myQuaternion.z();
        grasping_orientation.w = myQuaternion.w();
        cartesian_position.orientation = grasping_orientation;
        try
        {
            robot.cartesian_position_control(cartesian_position, 2.0, 0.0);
        }
        catch(char const* e)
        {
            std::cerr << e << '\n';
        }
        break;
    }
    return 0;
}
