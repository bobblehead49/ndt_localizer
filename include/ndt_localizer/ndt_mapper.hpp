// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#ifndef NDT_MAPPER_HPP_
#define NDT_MAPPER_HPP_

#include <ndt_localizer/ndt_tools.hpp>
#include <pose_graph/pose_graph.hpp>

#include <string>
#include <set>
#include <map>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#if USE_OPENMP_PCL==1
#include <pclomp/ndt_omp.h>
#endif


class NDTMapper
{
public:
    /**
     * \brief Constructor
    */
    NDTMapper();

    /**
     * \brief Destructor
    */
    ~NDTMapper();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber points_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber map_save_request_sub_;
    ros::Publisher map_pub_;
    ros::Publisher target_map_pub_;
    ros::Publisher points_pub_;
    ros::Publisher convergence_pub_;
    ros::Publisher fitness_score_pub_;
    ros::Publisher num_iteration_pub_;
    ros::Publisher trans_prob_pub_;
    ros::Publisher deque_pub_;
    ros::Publisher marker_pub_;

    ros::Time current_scan_stamp_;
    ros::Time last_scan_stamp_;
    ros::Time last_map_publish_stamp_;

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

    bool odom_initialized_;
    bool map_initialized_;
    bool pcd_dense_option_;
    bool use_loop_closure_;
    bool adjusted_loop_with_last_scan_;
    bool attempting_loop_closure_;
    bool save_uncompressed_map_;
    bool save_submaps_;

    int submap_scan_count_;
    int submap_scan_size_;
    int current_submap_id_;
    int current_group_id_;

    float min_scan_range_;
    float max_scan_range_;
    float voxel_leaf_size_;
    float translation_error_tolerance_;
    float rotation_error_tolerance_;
    float map_add_shift_;
    float submap_include_distance_;
    float map_publish_interval_;
    float submap_connect_distance_;
    float initial_guess_resolution_;
    float initial_guess_count_;
    float loop_score_limit_;
    float loop_confirmation_translation_tolerance_;
    float loop_confirmation_rotation_tolerance_;

    Eigen::Matrix4f base2lidar_matrix_;
    Eigen::Matrix4f lidar2base_matrix_;

    pcl::PointCloud<pcl::PointXYZI> submap_;
    pcl::PointCloud<pcl::PointXYZI> target_map_;

#if USE_OPENMP_PCL == 1
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
#elif USE_OPENMP_PCL == 0
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
#endif

    Pose last_base_pose_;
    Pose last_added_base_pose_;
    Pose last_predicted_base_pose_;

    std::map<int, SubmapWithInfo> submap_map_;

    PoseGraph global_pose_graph_;

    SubmapWithInfo init_submap_with_info_;

    PredictionMethod prediction_method_;

    /** 
     * \brief Gets connected submap ids in pose graph within the specified distance.
     * \param target_id The target submap id.
     * \param include_distance The distance to include submaps.
     * \return The set of connected submap ids.
     */
    std::unordered_set<int> get_locally_connected_ids(const int target_id, const float include_distance);

    /** 
     * \brief Groups the given submap ids.
     * \param connected_id_set The set of submap ids to group.
     */
    void group_submaps(const std::unordered_set<int>& connected_id_set);

    /** 
     * \brief Publishes markers of loop closure.
     * \param target_matrix The target submap position for loop closure.
     * \param guess_matrix The initial guess matrix for localization.
     * \param destination_matrix_map The submap positions of the entire loop.
     */
    void publish_loop_markers(const Eigen::Matrix4f& target_matrix, const Eigen::Matrix4f& guess_matrix, const std::map<int, Eigen::Matrix4f>& destination_matrix_map);

    /** 
     * \brief Gets the target submap ids for loop closure.
     * \param connected_id_set The set of connected submap ids to exclude.
     * \return The set of target submap ids.
     */
    std::unordered_set<int> get_loop_target_ids(const std::unordered_set<int>& connected_id_set);

    /** 
     * \brief Gets the initial guess matrix for loop closure.
     * \param source_matrix The source matrix.
     * \param target_matrix The target matrix.
     * \param initial_guess_matrix The initial guess matrix for localization.
     */
    void get_initial_guess(const Eigen::Matrix4f& source_matrix, const Eigen::Matrix4f& target_matrix, Eigen::Matrix4f& initial_guess_matrix);

    /** 
     * \brief Gets the loop closure correction matrix.
     * \param source_matrix The source matrix.
     * \param target_id_set The set of target submap ids to use for loop closure localization.
     * \param initial_guess_matrix The initial guess matrix for localization.
     * \param correction_matrix The loop closure correction matrix.
     * \return True if an acceptable correction is found.
     */
    bool get_loop_correction(const Eigen::Matrix4f& source_matrix, const std::unordered_set<int>& target_id_set, const Eigen::Matrix4f& initial_guess_matrix, Eigen::Matrix4f& correction_matrix);

    /** 
     * \brief Gets the submap id path from start to goal.
     * \param start_id_candidates The candidate set of submap ids of start.
     * \param goal_id The goal submap id.
     * \return The submap id path from start to goal.
     */
    std::vector<int> get_loop_id_path(const std::unordered_set<int>& start_id_candidates, const int goal_id);

    /** 
     * \brief Removes submap ids from same groups.
     * \param id_path The submap id path from start to goal.
     * \return The grouped submap id path from start to goal.
     */
    std::vector<int> get_grouped_loop_id_path(const std::vector<int>&);

    /** 
     * \brief Adjusts the submap angles of the given submap id path.
     * \param loop_correction_matrix The loop closure correction matrix.
     * \param destination_matrix_map The destinations for each submap of the entire loop.
     */
    void adjust_angles(const Eigen::Matrix4f& loop_correction_matrix, std::map<int, Eigen::Matrix4f>& destination_matrix_map);

    /**
     * \brief Adjusts the submap positions of the given submap id path.
     * \param loop_correction_matrix The loop closure correction matrix.
     * \param destination_matrix_map The destinations for each submap of the entire loop.
    */
    void adjust_positions(const Eigen::Matrix4f& loop_correction_matrix, std::map<int, Eigen::Matrix4f>& destination_matrix_map);

    /** 
     * \brief Shifts the submap point clouds of the given submap id path.
     * \param destination_matrix_map The destinations for each submap of the entire loop.
     */
    void shift_submaps(const std::map<int, Eigen::Matrix4f>& destination_matrix_map);

    /** 
     * \brief Attempts to close the loop with the given submap id set.
     * \param map2base_matrix The map to base transform matrix.
     * \param loop_target_id_set The set of target submap ids to use for loop closure localization.
     * \return True if loop closure was attempted.
     */
    bool close_loop(const std::unordered_set<int>& loop_target_id_set, Eigen::Matrix4f& map2base_matrix);

    /** 
     * \brief Updates submap and target map with the given scan.
     * \param scan_for_mapping The scan to update the maps.
     * \param map2base_matrix The map to base transform matrix.
     * \param base_pose The map to base transform pose.
     * \return True if loop closure was attempted.
     */
    void update_maps(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_for_mapping, Eigen::Matrix4f& map2base_matrix, Pose& base_pose);

    /** 
     * \brief Executed when a odom topic is received.
     * \param msg The received message.
     */
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);

    /** 
     * \brief Executed when a point cloud topic is received.
     * \param msg The received message.
     */
    void points_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

    /** 
     * \brief Executed when a map save request topic is received.
     * \param msg The received message.
     */
    void map_save_request_callback(const std_msgs::StringConstPtr& msg);
};

#endif // NDT_MAPPER_HPP_
