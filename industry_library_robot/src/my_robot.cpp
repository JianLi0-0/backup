#include "my_robot.h"

Robot::Robot(ros::NodeHandle &nh):nh_(nh), joint_seed(6),
    config_file("/home/ur5e/lee_ws/src/industry_library_robot/json_config/robot_config.json"),
    ik_solver("base_link", "ee_link", "/robot_description", 0.008, 1e-5, TRAC_IK::Distance)
{
    // load config file
    if(!reader.parse(config_file, json_params)){
        cout<<"Failed to load the config file."<<endl;
        exit(1);
	}
	else
	{
        cout<<"Config file loaded."<<endl;
		picking_params = json_params["picking"];
	}
    // set up publisher and subscriber
    // ur_pub = nh_.advertise<std_msgs::String>("/ur_hardware_interface/script_command", 1);
    // gazebo_position_controller = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);
    pos_tra_controller = nh_.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 1);
    gripper_controller = nh_.advertise<control_msgs::GripperCommandActionGoal>("/ezgripper/main/goal", 1);
    // desired_pose_sub = nh_.subscribe("/touch/stylus_pose" , 5, &Robot::haha, this);
    move_home_pose = nh_.advertiseService("move_home_pose_service", &Robot::move_home_pose_callback, this);
    pick_and_place = nh_.advertiseService("pick_and_place_service", &Robot::pick_and_place_callback, this);
    // variables initialization
    for(auto point:picking_params["place_position"]["1"])
        cartesian_box_position.push_back(point.asDouble());

    Json::Value json_joint_angle = picking_params["home_angles"];// read from json config file
    for(int i=0;i<6;i++){
        home_angles.data.push_back(json_joint_angle[i].asDouble());
        last_computed_angle.data.push_back(json_joint_angle[i].asDouble());    // home postion
    }
    
    ros::Duration(0.2).sleep();
    control_msgs::GripperCommandActionGoal gripper_command_msg;
    gripper_command_msg.goal.command.position = 60;
    gripper_command_msg.goal.command.max_effort = 50;
    gripper_controller.publish(gripper_command_msg);

}

Robot::~Robot() {}

bool Robot::move_home_pose_callback(industry_library_robot::pick_and_place::Request  &req, industry_library_robot::pick_and_place::Response &res)
{
    // gazebo_position_controller.publish(home_angles);
    // ros::Duration(3.0).sleep();
    joint_position_control(home_angles.data, 3.0);
    res.is_successful = true;
}

bool Robot::pick_and_place_callback(industry_library_robot::pick_and_place::Request  &req, industry_library_robot::pick_and_place::Response &res)
{
    geometry_msgs::Pose cartesian_position;
    static tf::Transform camera_to_object;
    static tf::Transform base_to_object;
    control_msgs::GripperCommandActionGoal gripper_command_msg;

    base_to_ee_link = listen_to_transform_tf("base_link", "ee_link", ros::Time(0));
    auto ee_link_to_color_optical_frame = listen_to_transform_tf("ee_link", "camera_color_frame", ros::Time(0));

    camera_to_object.setIdentity();
    camera_to_object.setOrigin(tf::Vector3(req.object_position.position.x, req.object_position.position.y, req.object_position.position.z));
    base_to_object = base_to_ee_link * ee_link_to_color_optical_frame * camera_to_object;

    grasping_orientation.x = picking_params["grasp_orientation"][req.object_name][0].asDouble();
    grasping_orientation.y = picking_params["grasp_orientation"][req.object_name][1].asDouble();
    grasping_orientation.z = picking_params["grasp_orientation"][req.object_name][2].asDouble();
    grasping_orientation.w = picking_params["grasp_orientation"][req.object_name][3].asDouble();
    cout << req.object_name << grasping_orientation << endl;

    cartesian_position.position.x = base_to_object.getOrigin().getX() + picking_params["compensation"][req.object_name][0].asDouble();
    cartesian_position.position.y = base_to_object.getOrigin().getY() + picking_params["compensation"][req.object_name][1].asDouble();
    cartesian_position.position.z = base_to_object.getOrigin().getZ() + picking_params["compensation"][req.object_name][2].asDouble();
    cartesian_position.orientation = grasping_orientation;
    
    // cout << "base_to_ee_link: " << base_to_ee_link.getOrigin().getX() << ", " << base_to_ee_link.getOrigin().getY() << ", "  << base_to_ee_link.getOrigin().getZ() << endl;
    // cout << "ee_link_to_color_optical_frame: " << ee_link_to_color_optical_frame.getOrigin().getX() << ", " << ee_link_to_color_optical_frame.getOrigin().getY() << ", "  << ee_link_to_color_optical_frame.getOrigin().getZ() << endl;
    cout << "camera_to_object: " << camera_to_object.getOrigin().getX() << ", " << camera_to_object.getOrigin().getY() << ", "  << camera_to_object.getOrigin().getZ() << endl;
    cout << "base_to_object: " << base_to_object.getOrigin().getX() << ", " << base_to_object.getOrigin().getY() << ", "  << base_to_object.getOrigin().getZ() << endl;

    try{

        cout << "Press e to proceed." << endl;
        char proceed;
        cin >> proceed;
        if(proceed!='e') throw("Interrupt ");

        // first approach
        cartesian_position.position.z += picking_params["approach_gap"].asDouble();
        // cout << "cartesian_position" << cartesian_position << endl;
        cartesian_position_control(cartesian_position, 3.0, 0.5);
        

        // approach the object
        cartesian_position.position.z += -picking_params["approach_gap"].asDouble();
        cartesian_position_control(cartesian_position, 1.0, 0.0);
        
        // grasp 
        gripper_command_msg.goal.command.position =  picking_params["grasp_angle"][req.object_name].asDouble();
        gripper_command_msg.goal.command.max_effort = picking_params["grasp_force"][req.object_name].asDouble();
        gripper_controller.publish(gripper_command_msg);
        ros::Duration(2.0).sleep();

        // lift
        cartesian_position.position.z += picking_params["lift_gap"].asDouble();
        cartesian_position_control(cartesian_position, 3.0, 0.0);
        
        // move to palce position
        geometry_msgs::Pose temp_box_position; // get temp_box_position frome req.place_point
        temp_box_position.position.x = picking_params["place_position"][req.place_point][0].asDouble();
        temp_box_position.position.y = picking_params["place_position"][req.place_point][1].asDouble();
        temp_box_position.position.z = picking_params["place_position"][req.place_point][2].asDouble();
        temp_box_position.orientation = grasping_orientation;

        temp_box_position.position.z += picking_params["approach_gap"].asDouble();
        cartesian_position_control(temp_box_position, 3.0, 0.5);
        
        
        // approach the box
        temp_box_position.position.z += -picking_params["approach_gap"].asDouble();
        cartesian_position_control(temp_box_position, 1.0, 0.0);
        
        
        // release
        gripper_command_msg.goal.command.position = 40;
        gripper_command_msg.goal.command.max_effort = 100;
        gripper_controller.publish(gripper_command_msg);
        ros::Duration(2.0).sleep();

        // lift
        temp_box_position.position.z += picking_params["lift_gap"].asDouble();
        cartesian_position_control(temp_box_position, 3.0, 0.0);
        
        // back to stand by position
        joint_position_control(home_angles.data, 3.0);
        // cartesian_position_control(home_pose, 3.0, 0.0);
        
        
    }
    catch(const char* msg)
    {
        cerr << msg << endl;
        cout << "Terminate the service" << endl;
        res.is_successful = false;
        return false;
    }
    cout << "place_point: " << req.place_point << endl;
    res.is_successful = true;
    return true;
}

bool Robot::cartesian_position_control(geometry_msgs::Pose target_position, double duration, double delay)
{
    static trajectory_msgs::JointTrajectory joint_tra;
    static std::vector<string> joint_names = {"shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    std_msgs::Float64MultiArray target_joint_position;

    joint_tra.joint_names.assign(joint_names.begin(), joint_names.end());
    joint_tra.points.resize(1);
    // inverse kinematics
    target_joint_position = manipulator_ik(target_position);

    joint_tra.points[0].positions.assign(target_joint_position.data.begin(), target_joint_position.data.end());
    joint_tra.points[0].time_from_start = ros::Duration(duration);
    pos_tra_controller.publish(joint_tra);
    ros::Duration(duration + delay).sleep();
    return true;
}

void Robot::joint_position_control(std::vector<double> target_joint_position, double duration)
{
    static trajectory_msgs::JointTrajectory joint_tra;
    static std::vector<string> joint_names = {"shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    joint_tra.joint_names.assign(joint_names.begin(), joint_names.end());
    joint_tra.points.resize(1);

    joint_tra.points[0].positions.assign(target_joint_position.begin(), target_joint_position.end());
    joint_tra.points[0].time_from_start = ros::Duration(duration);
    pos_tra_controller.publish(joint_tra);
    ros::Duration(duration).sleep();
}

std_msgs::Float64MultiArray Robot::manipulator_ik(geometry_msgs::Pose target_pose)
{
    KDL::JntArray return_joints;
    double it[] = {target_pose.position.x, target_pose.position.y, target_pose.position.z};
    memcpy(desired_eef_pose.p.data, it, sizeof(it));
    memcpy(desired_eef_pose.M.data, desired_eef_pose.M.Quaternion(target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w).data,sizeof(desired_eef_pose.M.data));
    joint_seed.data << last_computed_angle.data[0],last_computed_angle.data[1],last_computed_angle.data[2],last_computed_angle.data[3],last_computed_angle.data[4],last_computed_angle.data[5];
    int rc = ik_solver.CartToJnt(joint_seed, desired_eef_pose, return_joints);
    
    if(rc<0){
        ROS_ERROR("Did not find IK solution");
        throw "Did not find IK solution";
    }

    for(int i=0;i<6;i++) 
        last_computed_angle.data[i] = return_joints.data(i);
        
    return last_computed_angle;
}

geometry_msgs::Pose Robot::listen_to_transform(string base_frame, string target_frame, ros::Time time_stamped)
{
    tf::StampedTransform transform;
    geometry_msgs::Pose target_pose;

    static tf::TransformListener listener(ros::Duration(10));
    listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(base_frame, target_frame, ros::Time(0), transform);

    target_pose.position.x = transform.getOrigin().x();
    target_pose.position.y = transform.getOrigin().y();
    target_pose.position.z = transform.getOrigin().z();
    target_pose.orientation.x = transform.getRotation().x();
    target_pose.orientation.y = transform.getRotation().y();
    target_pose.orientation.z = transform.getRotation().z();
    target_pose.orientation.w = transform.getRotation().w();

    return target_pose;
}

tf::StampedTransform Robot::listen_to_transform_tf(string base_frame, string target_frame, ros::Time time_stamped)
{
    tf::StampedTransform transform;
    static tf::TransformListener listener(ros::Duration(5));
    listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(base_frame, target_frame, ros::Time(0), transform);
    return transform;
}
