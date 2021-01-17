#include "StatesHub.h"
#include "ForceTorqueController.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/PoseArray.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <ultrasound_robot/wipe_bb.h>

void start_states_hub_thread(std::shared_ptr<SharedVariable> ptr)
{
	std::cout << "states_hub_thread" << std::endl;
    StatesHub states_hub(ptr);
    ros::waitForShutdown();
};

class WipeBlackboard
{
    private:
        pt::ptree config_tree_;
        std::shared_ptr<SharedVariable> shared_variable_ptr_;
        ros::NodeHandle nh_;
        ros::Subscriber pointclound_sub_;
        ros::Publisher joint_velocity_command_pub_;
        ros::Publisher pointcloud_pub_;
        ros::Publisher pointcloud_normals_pub_;
        ros::Publisher pose_array_pub;
        ros::ServiceServer wipe_blackboard_service;

        

    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

        WipeBlackboard(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree);
        ~WipeBlackboard();
        void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
        void estimatePointcloudNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const pcl::PointCloud<pcl::Normal>::Ptr normals);
        void publishPoseArray(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
        bool wipe_blackboard_service_callback(ultrasound_robot::wipe_bb::Request  &req, ultrasound_robot::wipe_bb::Response &res);
        geometry_msgs::PoseArray fromNormalsToPoseArray(const geometry_msgs::PoseArray pixel_points, 
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const pcl::PointCloud<pcl::Normal>::Ptr normals);
};

WipeBlackboard::WipeBlackboard(std::shared_ptr<SharedVariable> ptr, const pt::ptree config_tree):
    cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>)
{
    shared_variable_ptr_ = ptr;
    config_tree_ = config_tree;
    pointclound_sub_ = nh_.subscribe< pcl::PointCloud<pcl::PointXYZ> >("/camera/depth_registered/points", 1, &WipeBlackboard::pointcloud_callback, this);
    pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/pointcloud", 1);
    pointcloud_normals_pub_ = nh_.advertise<pcl::PointCloud<pcl::Normal> > ("/pointcloud_normals", 1);
    pose_array_pub = nh_.advertise<geometry_msgs::PoseArray>("/normal_vectors", 1);
    wipe_blackboard_service = nh_.advertiseService("wipe_blackboard_service", &WipeBlackboard::wipe_blackboard_service_callback, this);
}

WipeBlackboard::~WipeBlackboard()
{
    std::cout << "~WipeBlackboard()" << std::endl;
}

void WipeBlackboard::pointcloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    // std::cout << "pointclound_callback()" << std::endl;
    cloud = msg->makeShared();
    // //直通滤波器
    // pcl::PassThrough<pcl::PointXYZ>pass;
    // pass.setInputCloud(cloud);    //输入点云
    // pass.setFilterFieldName("z");    //沿z轴过滤
    // pass.setFilterLimits(-1.0,1.0);    //选取0-1之间
    // pass.setKeepOrganized(true);
    // //    pass.setFilterLimitsNegative(true);    //可选择0-1之间数据保留还是舍弃
    // pass.filter(*cloud_filtered);    //过滤
}

void WipeBlackboard::estimatePointcloudNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
     // camera_color_optical_frame
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setNormalEstimationMethod (normal_estimation.COVARIANCE_MATRIX);
    normal_estimation.setMaxDepthChangeFactor(0.02f);
    normal_estimation.setNormalSmoothingSize(10.0f);
    normal_estimation.setInputCloud(pointcloud);

    // auto past = ros::Time::now();
    normal_estimation.compute(*normals);
    // std::cout << "ros::Time::now()-past: " << ros::Time::now()-past << std::endl;

    // // visualize normals
    // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    // viewer.setBackgroundColor (0.0, 0.0, 0.5);
    // viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(pointcloud, normals);
    // while (!viewer.wasStopped ())
    // {
    //   viewer.spinOnce ();
    // }

}

void WipeBlackboard::publishPoseArray(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
    pointcloud_pub_.publish(*pointcloud);
}

bool WipeBlackboard::wipe_blackboard_service_callback(ultrasound_robot::wipe_bb::Request  &req, ultrasound_robot::wipe_bb::Response &res)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    auto pointcloud = cloud;
    estimatePointcloudNormals(pointcloud, normals);
    auto pose_array = fromNormalsToPoseArray(req.waypoints, pointcloud, normals);
    pose_array_pub.publish(pose_array);
    return true;
}

geometry_msgs::PoseArray WipeBlackboard::fromNormalsToPoseArray(const geometry_msgs::PoseArray pixel_points, 
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = pointcloud->header.frame_id;
    // std::cout << pose_array.header.frame_id << std::endl;
    // estimatePointcloudNormals(pointcloud, normals);
    for(int i=0;i<pixel_points.poses.size();i++)
    {
        auto pixel = pixel_points.poses[i];
        geometry_msgs::Pose temp_pose;
        auto width = pointcloud->width;
        auto height = pointcloud->height;
        // pixel.position.x : -> (right)  pixel.position.y: \|/ (down)
        auto index = (int)pixel.position.x + (int)pixel.position.y*width;
        
        auto position = pointcloud->points[index].data;
        temp_pose.position.x = position[0];
        temp_pose.position.y = position[1];
        temp_pose.position.z = position[2];

        Eigen::Vector3d z_axis(normals->points[index].normal_x, 
                                normals->points[index].normal_y, 
                                normals->points[index].normal_z);
        // std::cout << "z_axis " << z_axis << std::endl;
        static Eigen::Vector3d x_axis;
        if(i != pixel_points.poses.size()-1)
        {
            auto pixel = pixel_points.poses[i+1];
            auto next_index = (int)pixel.position.x + (int)pixel.position.y*width;
            // number of points must be larger than 1
            auto temp_x_axis = (pointcloud->points[next_index].getVector3fMap() - pointcloud->points[index].getVector3fMap());
            x_axis << temp_x_axis.matrix()(0,0), temp_x_axis.matrix()(1,0), temp_x_axis.matrix()(2,0);
            x_axis.normalize();
        }
        Eigen::Matrix3d R;
        auto y_axis = z_axis.cross(x_axis);
        y_axis.normalize();
        R << x_axis, y_axis, z_axis;
        Eigen::Quaterniond q(R);
        temp_pose.orientation.x = q.x();
        temp_pose.orientation.y = q.y();
        temp_pose.orientation.z = q.z();
        temp_pose.orientation.w = q.w();
        pose_array.poses.push_back(temp_pose);
    }

    return pose_array;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wipe_blackboard");
    ros::NodeHandle nh;
    std::shared_ptr<SharedVariable> shared_variable_ptr = std::make_shared<SharedVariable>();
    pt::ptree root;
	pt::read_json("/home/qwe/lee_ws/src/ultrasound_robot/config/force_controller.json", root);
    ForceTorqueController ft_controller(shared_variable_ptr, root);
    WipeBlackboard wipe_blackboard(shared_variable_ptr, root);
    shared_variable_ptr->config_tree = root;
    ros::Rate loop_rate( int(1.0/root.get<double>("delta_t", 0.005)) );
    // ros::Publisher vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();
    sleep(1);
    std::thread states_hub_thread(start_states_hub_thread, shared_variable_ptr);
    sleep(1);

    while(ros::ok())
    {
        wipe_blackboard.publishPoseArray(wipe_blackboard.cloud_filtered);
        loop_rate.sleep();
        // std::cout << "jonit_velocity: " << jonit_velocity << std::endl;
        // std::cout << "vel_pub: " << vel << std::endl;
        // std::cout << "shared_variable_ptr.use_count()" <<  shared_variable_ptr.use_count() << std::endl;
    }

    ros::waitForShutdown();
    states_hub_thread.join();

    return 0;
}