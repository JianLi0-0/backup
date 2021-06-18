#ifndef MY_ROBOT_H
#define MY_ROBOT_H

#include <cmath>
#include<iostream>
#include<fstream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "industry_library_robot/pick_and_place.h"
#include <trac_ik/trac_ik.hpp>
#include <jsoncpp/json/json.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandActionGoal.h>

using namespace std;

class Robot
{
    public:
        Robot(ros::NodeHandle &nh);
        virtual ~Robot();

        ros::NodeHandle nh_;
        ros::Publisher ur_pub;
        ros::Publisher gazebo_position_controller;
        ros::Publisher pos_tra_controller;
        ros::Publisher gripper_controller;

        // ros::Subscriber desired_pose_sub;
        ros::ServiceServer pick_and_place;
        ros::ServiceServer move_home_pose;
        ros::Timer tf_broadcast_publish_timer;

        geometry_msgs::Pose home_pose;
        tf::StampedTransform base_to_ee_link;
        tf::StampedTransform ee_link_to_camera_link;
        tf::StampedTransform camera_link_to_color_optical_frame;
        std_msgs::Float64MultiArray home_angles;
        Json::Value json_params;
        Json::Value picking_params;
        Json::Reader reader;
        std::ifstream config_file;
        std::vector<double> cartesian_box_position;
        geometry_msgs::Quaternion grasping_orientation;
        tf::TransformBroadcaster tf_broadcast;
        geometry_msgs::Pose raw_object_position;
        // double cartesian_box_position[3][3];

        TRAC_IK::TRAC_IK ik_solver;
        KDL::JntArray joint_seed;
        KDL::Frame desired_eef_pose;
        std_msgs::Float64MultiArray last_computed_angle;
        double ik_angle_change_threshold = 0.14;

        // void haha(const std_msgs::Float32ConstPtr& msg);
        bool pick_and_place_callback(industry_library_robot::pick_and_place::Request  &req, industry_library_robot::pick_and_place::Response &res);
        bool move_home_pose_callback(industry_library_robot::pick_and_place::Request  &req, industry_library_robot::pick_and_place::Response &res);
        bool cartesian_position_control(geometry_msgs::Pose target_position, double duration, double delay);
        void joint_position_control(std::vector<double> target_joint_position, double duration);
        std_msgs::Float64MultiArray manipulator_ik(geometry_msgs::Pose target_pose);
        geometry_msgs::Pose listen_to_transform(string base_frame, string target_frame, ros::Time time_stamped);
        tf::StampedTransform listen_to_transform_tf(string base_frame, string target_frame, ros::Time time_stamped);
        void tf_broadcast_callback(const ros::TimerEvent& e);
        ros::Time broadcast_tf(geometry_msgs::Pose p);
    private:
};

#endif
