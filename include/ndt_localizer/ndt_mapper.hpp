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


class NDTMapper
{
public:
    NDTMapper();
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
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

    Pose last_base_pose_;
    Pose last_added_base_pose_;
    Pose last_predicted_base_pose_;

    std::map<int, SubmapWithInfo> submap_map_;

    PoseGraph global_pose_graph_;

    SubmapWithInfo init_submap_with_info_;

    PredictionMethod prediction_method_;

    std::unordered_set<int> get_locally_connected_ids(const int, const float);
    void group_submaps(const std::unordered_set<int>&);
    void publish_loop_markers(const Eigen::Matrix4f& target_matrix, const Eigen::Matrix4f& guess_matrix, const std::map<int, Eigen::Matrix4f>& destination_matrix_map);
    std::unordered_set<int> get_loop_target_ids(const std::unordered_set<int>&);
    void get_initial_guess(const Eigen::Matrix4f& source, const Eigen::Matrix4f& target, Eigen::Matrix4f& initial_guess);
    bool get_loop_correction(const Eigen::Matrix4f& map2base_matrix, const std::unordered_set<int>& target_id_set, const Eigen::Matrix4f& initial_guess, Eigen::Matrix4f& correction);
    std::vector<int> get_loop_id_path(const std::unordered_set<int>&, const int);
    std::vector<int> get_grouped_loop_id_path(const std::vector<int>&);
    void adjust_angles(const Eigen::Matrix4f&, std::map<int, Eigen::Matrix4f>&);
    void adjust_positions(const Eigen::Matrix4f&, std::map<int, Eigen::Matrix4f>&);
    void shift_submaps(const std::map<int, Eigen::Matrix4f>&);
    bool close_loop(const Eigen::Matrix4f& map2base_matrix, const std::unordered_set<int>& loop_target_id_set, Pose& base_pose);
    void update_maps(const pcl::PointCloud<pcl::PointXYZI>::Ptr&, const Eigen::Matrix4f&, Pose&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void points_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void map_save_request_callback(const std_msgs::StringConstPtr&);
};

#endif // NDT_MAPPER_HPP_
