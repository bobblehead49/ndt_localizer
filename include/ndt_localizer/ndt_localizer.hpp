#ifndef NDT_LOCALIZER_HPP_
#define NDT_LOCALIZER_HPP_

#include <ndt_localizer/ndt_tools.hpp>

#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>


class NDTLocalizer
{
public:
    NDTLocalizer();
    ~NDTLocalizer();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber points_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber init_pose_sub_;
    ros::Publisher points_pub_;
    ros::Publisher full_map_pub_;
    ros::Publisher target_map_pub_;
    ros::Publisher convergence_pub_;
    ros::Publisher fitness_score_pub_;
    ros::Publisher num_iteration_pub_;
    ros::Publisher trans_prob_pub_;    

    ros::Time last_scan_stamp_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    geometry_msgs::Twist last_base_twist_;

    nav_msgs::Odometry last_odom_;

    std::string map_frame_;
    std::string base_frame_;
    std::string lidar_frame_;
    std::string points_topic_;
    std::string map_topic_;
    std::string odom_topic_;

    bool points_initialized_;
    bool odom_initialized_;
    bool pose_initialized_;
    bool use_submaps_;

    float min_scan_range_;
    float max_scan_range_;
    float voxel_leaf_size_;
    float translation_error_tolerance_;
    float rotation_error_tolerance_;
    float submap_include_distance_;
    float submap_update_shift_;

    Eigen::Matrix4f base2lidar_matrix_;
    Eigen::Matrix4f lidar2base_matrix_;

    pcl::PointCloud<pcl::PointXYZI> target_map_;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

    std::map<int, SubmapWithInfo> submap_map_;

    Pose last_base_pose_;
    Pose last_predicted_base_pose_;
    Pose last_map_updated_base_pose_;

    PredictionMethod prediction_method_;

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
};

#endif // NDT_LOCALIZER_HPP_
