#ifndef NDT_LOCALIZER_TOOLS_HPP_
#define NDT_LOCALIZER_TOOLS_HPP_

#include <string>
#include <map>
#include <unordered_set>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_listener.h>


struct Pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

enum PredictionMethod
{
    ZERO,
    LINEAR,
    ODOM
};
const std::vector<std::string> prediction_method_candidates = {"zero", "linear", "odom"};

struct SubmapWithInfo
{
    bool in_group;
    int group_id;
    double distance;
    Eigen::Matrix4f position_matrix;
    pcl::PointCloud<pcl::PointXYZI> points;
};

double diff_radian(const double, const double);
bool get_tf(const tf2_ros::Buffer&, const std::string&, const std::string&, geometry_msgs::TransformStamped&);
PredictionMethod convert_prediction_method(const std::string&);
Eigen::Matrix4f convert_tf2matrix(const geometry_msgs::TransformStamped&);
geometry_msgs::TransformStamped convert_matrix2tf(const Eigen::Matrix4f&);
Pose convert_matrix2pose(const Eigen::Matrix4f&);
Eigen::Matrix4f convert_pose2matrix(const Pose&);
Pose get_linear_prediction_pose(const Pose&, const geometry_msgs::Twist&, const double);
std::unordered_set<int> get_neighbor_ids(const std::map<int, SubmapWithInfo>&, const float);
void apply_range_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZI>::Ptr&, const float, const float);
void apply_voxel_grid_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZI>::Ptr&, const float);
void load_full_map(const std::string&, pcl::PointCloud<pcl::PointXYZI>&);
void load_submaps(const std::string&, std::map<int, SubmapWithInfo>&);
void update_submap_distances(const Pose&, std::map<int, SubmapWithInfo>&);

#endif  // NDT_LOCALIZER_TOOLS_HPP_
