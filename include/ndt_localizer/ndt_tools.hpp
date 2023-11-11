// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

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
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>


/**
 * \brief Pose structure.
 */
struct Pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

/**
 * \brief Prediction method enumeration.
 */
enum PredictionMethod
{
    ZERO,
    LINEAR,
    ODOM
};

/**
 * \brief Prediction method candidates.
 */
const std::vector<std::string> prediction_method_candidates = {"zero", "linear", "odom"};

/**
 * \brief Submap structure.
 */
struct SubmapWithInfo
{
    bool in_group;
    int group_id;
    double distance;
    Eigen::Matrix4f position_matrix;
    pcl::PointCloud<pcl::PointXYZI> points;
};

/**
 * \brief Gets the difference between two angles in radian from -pi to pi.
 * \param rad_1 The first angle.
 * \param rad_2 The second angle.
 * \return The difference between two angles in radian from -pi to pi.
 */
double diff_radian(const double rad_1, const double rad_2);

/**
 * \brief Gets the transform from source_frame to target_frame.
 * \param source_frame The source frame.
 * \param target_frame The target frame.
 * \param transform The transform from source_frame to target_frame.
 * \return True if the transform is found.
 */
bool get_tf(const tf2_ros::Buffer& tf_buffer, const std::string& target_frame, const std::string& source_frame, geometry_msgs::TransformStamped& transform);

/**
 * \brief Converts prediction method string to enum. Terminates the program if an invalid string is given.
 * \param prediction_method_str The prediction method string.
 * \return The prediction method enum.
 */
PredictionMethod convert_prediction_method(const std::string& prediction_method_str);

/**
 * \brief Converts eigen transform matrix to tf.
 * \param matrix The eigen transform matrix.
 * \return The tf.
 */
geometry_msgs::TransformStamped convert_matrix2tf(const Eigen::Matrix4f& matrix);

/**
 * \brief Converts tf to eigen transform matrix.
 * \param transform The tf.
 * \return The eigen transform matrix.
 */
Eigen::Matrix4f convert_tf2matrix(const geometry_msgs::TransformStamped& transform);

/**
 * \brief Converts eigen transform matrix to pose.
 * \param matrix The eigen transform matrix.
 * \return The transform pose.
 */
Pose convert_matrix2pose(const Eigen::Matrix4f&);

/**
 * \brief Converts transform pose to eigen transform matrix.
 * \param pose The transform pose.
 * \return The eigen transform matrix.
 */
Eigen::Matrix4f convert_pose2matrix(const Pose&);

/**
 * \brief Converts odometry to eigen transform matrix.
 * \param odom The odometry.
 * \return The eigen transform matrix.
 */
Eigen::Matrix4f convert_odom2matrix(const nav_msgs::Odometry& odom);

/**
 * \brief Gets the linearly predicted pose.
 * \param current_pose The current pose.
 * \param twist The twist.
 * \param dt The time diffference multiplied to the twist.
 * \return The linearly predicted pose.
 */
Pose get_linear_prediction_pose(const Pose& current_pose, const geometry_msgs::Twist& twist, const double dt);

/**
 * \brief Gets nearby submap ids.
 * \param submap_map The map of submaps with info.
 * \param include_distance The distance to include.
 * \return The nearby submap ids.
 */
std::unordered_set<int> get_neighbor_ids(const std::map<int, SubmapWithInfo>& submap_map, const float include_distance);

/**
 * \brief Applies range filter to the point cloud.
 * \param scan The input point cloud.
 * \param scan_filtered The output point cloud.
 * \param min_range The minimum range to include.
 * \param max_range The maximum range to include.
 */
void apply_range_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan, pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_filtered, const float min_scan_range, const float max_scan_range);

/**
 * \brief Applies voxel grid filter to the point cloud.
 * \param scan The input point cloud.
 * \param scan_filtered The output point cloud.
 * \param voxel_leaf_size The voxel leaf size.
 */
void apply_voxel_grid_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan, pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_filtered, const float voxel_leaf_size);

/**
 * \brief Loads the full map.
 * \param filepath The file path to the map.
 * \param map The output point cloud.
 */
void load_full_map(const std::string& filepath, pcl::PointCloud<pcl::PointXYZI>& map);

/**
 * \brief Loads the submaps.
 * \param submap_dir The directory path to the submaps.
 * \param submap_map The output map of submaps with info.
 */
void load_submaps(const std::string& submap_dir, std::map<int, SubmapWithInfo>& submap_map);

/**
 * \brief Updates the submap distances.
 * \param current_pose The current pose.
 * \param submap_map The map of submaps with info.
 */
void update_submap_distances(const Pose& current_pose, std::map<int, SubmapWithInfo>& submap_map);

#endif  // NDT_LOCALIZER_TOOLS_HPP_
