#include <ndt_localizer/ndt_localizer.hpp>
#include <ndt_localizer/ndt_tools.hpp>

#include <string>
#include <unordered_set>
#include <filesystem>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#if USE_OPENMP_PCL==1
#include <pclomp/ndt_omp.h>
#endif


NDTLocalizer::NDTLocalizer() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
    // Get parameters
    std::string maps_dir, map_name, prediction_method_str;
    int ndt_max_iterations, openmp_thread_num, openmp_neighbor_search_method;
    float ndt_resolution, ndt_step_size, ndt_transformation_epsilon;
    float rotation_error_tolerance_deg;
    pnh_.param<std::string>("maps_directory", maps_dir, "./");
    pnh_.param<std::string>("map_name", map_name, "map");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
    pnh_.param<std::string>("points_topic", points_topic_, "/points");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    pnh_.param<float>("min_scan_range", min_scan_range_, 0.3);
    pnh_.param<float>("max_scan_range", max_scan_range_, 200.0);
    pnh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.5);
    pnh_.param<int>("ndt_max_iterations", ndt_max_iterations, 30);
    pnh_.param<float>("ndt_resolution", ndt_resolution, 2.0);
    pnh_.param<float>("ndt_step_size", ndt_step_size, 0.1);
    pnh_.param<float>("ndt_transformation_epsilon", ndt_transformation_epsilon, 0.001);
    pnh_.param<std::string>("prediction_method", prediction_method_str, "linear");
    pnh_.param<float>("translation_error_tolerance", translation_error_tolerance_, 1.5);
    pnh_.param<float>("rotation_error_tolerance", rotation_error_tolerance_deg, 15.0);
    pnh_.param<bool>("use_submaps", use_submaps_, true);
    pnh_.param<float>("submap_include_distance", submap_include_distance_, 30.0);
    pnh_.param<float>("submap_update_shift", submap_update_shift_, 5.0);
    pnh_.param<int>("openmp_thread_num", openmp_thread_num, 16);
    pnh_.param<int>("openmp_neighbor_search_method", openmp_neighbor_search_method, 2);

    // Convert parameters
    prediction_method_ = convert_prediction_method(prediction_method_str);
    rotation_error_tolerance_ = rotation_error_tolerance_deg * M_PI / 180.0;

    // Initialize variables
    points_initialized_ = false;
    odom_initialized_ = false;
    pose_initialized_ = false;
    last_base_pose_ = {0, 0, 0, 0, 0, 0};
    last_predicted_base_pose_ = {0, 0, 0, 0, 0, 0};
    last_map_updated_base_pose_ = {0, 0, 0, 0, 0, 0};
    last_base_twist_.linear.x = last_base_twist_.linear.y = last_base_twist_.linear.z = 0;
    last_base_twist_.angular.x = last_base_twist_.angular.y = last_base_twist_.angular.z = 0;

    // Setup ndt
#if USE_OPENMP_PCL == 1
    ndt_.setNumThreads(openmp_thread_num);
    ndt_.setNeighborhoodSearchMethod(static_cast<pclomp::NeighborSearchMethod>(openmp_neighbor_search_method));
    ROS_INFO("Using OpenMP with %d threads.", openmp_thread_num);
#elif USE_OPENMP_PCL == 0
    ROS_INFO("Not using OpenMP.");
#endif
    ndt_.setResolution(ndt_resolution);
    ndt_.setStepSize(ndt_step_size);
    ndt_.setTransformationEpsilon(ndt_transformation_epsilon);
    ndt_.setMaximumIterations(ndt_max_iterations);

    // Load map
    std::filesystem::path maps_dir_fs(maps_dir);
    maps_dir_fs = maps_dir_fs / map_name;
    pcl::PointCloud<pcl::PointXYZI> full_map;
    if (use_submaps_)
    {
        load_submaps(maps_dir_fs / "submaps", submap_map_);
        for (auto submap_with_info : submap_map_)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_points_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            apply_voxel_grid_filter(submap_with_info.second.points.makeShared(), filtered_points_ptr, ndt_resolution);
            submap_map_[submap_with_info.first].points = *filtered_points_ptr;
            full_map.operator+=(submap_with_info.second.points);
        }
    }
    else
    {
        load_full_map(maps_dir_fs / "map.pcd", full_map);
        target_map_ = full_map;
    }

    // Setup publishers and subscribers
    points_sub_ = nh_.subscribe(points_topic_, 1, &NDTLocalizer::points_callback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &NDTLocalizer::odom_callback, this);
    init_pose_sub_ = nh_.subscribe("/initialpose", 1, &NDTLocalizer::init_pose_callback, this);
    points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_localizer/points", 1);
    full_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_localizer/map", 1, true);
    target_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_localizer/target_map", 1);
    convergence_pub_ = nh_.advertise<std_msgs::Bool>("/ndt_localizer/convergence", 1);
    fitness_score_pub_ = nh_.advertise<std_msgs::Float32>("/ndt_localizer/fitness_score", 1);
    num_iteration_pub_ = nh_.advertise<std_msgs::Int32>("/ndt_localizer/iteration_num", 1);
    trans_prob_pub_ = nh_.advertise<std_msgs::Float32>("/ndt_localizer/transformation_probability", 1);

    // Wait for subscribers
    ros::Duration(0.5).sleep();

    // Publish full map
    pcl::PointCloud<pcl::PointXYZI>::Ptr full_map_ptr(new pcl::PointCloud<pcl::PointXYZI>(full_map));
    apply_voxel_grid_filter(full_map_ptr, full_map_ptr, ndt_resolution);
    sensor_msgs::PointCloud2 map_msg;
    full_map_ptr->header.frame_id = map_frame_;
    pcl::toROSMsg(*full_map_ptr, map_msg);
    full_map_pub_.publish(map_msg);

    // Get static transforms
    geometry_msgs::TransformStamped base2lidar_tf;
    while (ros::ok() && !get_tf(tf_buffer_, base_frame_, lidar_frame_, base2lidar_tf))
        ros::Duration(0.5).sleep();
    base2lidar_matrix_ = convert_tf2matrix(base2lidar_tf);
    lidar2base_matrix_ = base2lidar_matrix_.inverse();
}

NDTLocalizer::~NDTLocalizer(){}

void NDTLocalizer::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    if (!odom_initialized_)
    {
        last_odom_ = *msg;
        odom_initialized_ = true;
        return;
    }

    // Get odom matrices
    Eigen::Matrix4f current_odom_matrix, last_odom_matrix;
    current_odom_matrix = convert_odom2matrix(*msg);
    last_odom_matrix = convert_odom2matrix(last_odom_);

    // Get map to base matrices
    Eigen::Matrix4f map2base_matrix, last_map2base_matrix;
    last_map2base_matrix = convert_pose2matrix(last_predicted_base_pose_);
    map2base_matrix = last_map2base_matrix * last_odom_matrix.inverse() * current_odom_matrix;

    // Update prediction
    last_predicted_base_pose_ = convert_matrix2pose(map2base_matrix);

    // Set values for next callback
    last_odom_ = *msg;
}

void NDTLocalizer::init_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    // Convert initial pose
    Pose init_pose;
    init_pose.x = msg->pose.pose.position.x;
    init_pose.y = msg->pose.pose.position.y;
    init_pose.z = msg->pose.pose.position.z;
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(init_pose.roll, init_pose.pitch, init_pose.yaw);

    // Apply initial pose
    last_base_twist_.linear.x = 0.0;
    last_base_twist_.linear.y = 0.0;
    last_base_twist_.linear.z = 0.0;
    last_base_twist_.angular.x = 0.0;
    last_base_twist_.angular.y = 0.0;
    last_base_twist_.angular.z = 0.0;
    last_base_pose_ = init_pose;
    last_predicted_base_pose_ = init_pose;

    if (use_submaps_)
    {
        // Update submap distances
        update_submap_distances(init_pose, submap_map_);

        // Update target map
        target_map_.clear();
        std::unordered_set<int> target_id_set;
        target_id_set = get_neighbor_ids(submap_map_, submap_include_distance_);
        for (auto id : target_id_set)
            target_map_.operator+=(submap_map_[id].points);
        target_map_.header.frame_id = map_frame_;
        ndt_.setInputTarget(target_map_.makeShared());
    }
    else
        ndt_.setInputTarget(target_map_.makeShared());

    pose_initialized_ = true;
    ROS_INFO("Initial pose received.");
}

void NDTLocalizer::points_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!ros::ok() || !pose_initialized_)
        return;

    double dt;
    if (!points_initialized_)
    {        
        dt = 1.0;
        points_initialized_ = true;
    }
    else
        dt = (msg->header.stamp - last_scan_stamp_).toSec();

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    // Convert scan data
    pcl::fromROSMsg(*msg, *scan);

    // Apply range filter
    apply_range_filter(scan, scan_filtered, min_scan_range_, max_scan_range_);

    // Apply voxel grid filter
    apply_voxel_grid_filter(scan_filtered, scan_filtered, voxel_leaf_size_);

    // Predict current base pose with predictive model
    Pose predicted_base_pose;
    switch (prediction_method_)
    {
        case ZERO:
            predicted_base_pose = last_base_pose_;
            break;
        case LINEAR:
            predicted_base_pose = get_linear_prediction_pose(last_base_pose_, last_base_twist_, dt);
            break;
        case ODOM:
            predicted_base_pose = last_predicted_base_pose_;
            break;
    }

    // Get current predicted map to lidar transform
    Eigen::Matrix4f predicted_map2base_matrix, predicted_map2lidar_matrix;
    predicted_map2base_matrix = convert_pose2matrix(predicted_base_pose);
    predicted_map2lidar_matrix = base2lidar_matrix_ * predicted_map2base_matrix;

    // Align scan to map
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_aligned(new pcl::PointCloud<pcl::PointXYZI>);
    ndt_.setInputSource(scan_filtered);
    ndt_.align(*scan_aligned, predicted_map2lidar_matrix);

    // Get ndt results
    bool has_convergered;
    int final_num_iteration;
    double fitness_score, transformation_probability;
    Eigen::Matrix4f ndt_map2lidar_matrix;
    has_convergered = ndt_.hasConverged();
    fitness_score = ndt_.getFitnessScore();
    final_num_iteration = ndt_.getFinalNumIteration();
    transformation_probability = ndt_.getTransformationProbability();
    ndt_map2lidar_matrix = ndt_.getFinalTransformation();

    // Convert to base pose
    Eigen::Matrix4f ndt_map2base_matrix;
    Pose ndt_base_pose;
    ndt_map2base_matrix = lidar2base_matrix_ * ndt_map2lidar_matrix;
    ndt_base_pose = convert_matrix2pose(ndt_map2base_matrix);

    // Calculate difference between ndt base pose and predicted base pose
    double diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw, translation_error, rotation_error;
    diff_x = ndt_base_pose.x - predicted_base_pose.x;
    diff_y = ndt_base_pose.y - predicted_base_pose.y;
    diff_z = ndt_base_pose.z - predicted_base_pose.z;
    diff_roll = diff_radian(ndt_base_pose.roll, predicted_base_pose.roll);
    diff_pitch = diff_radian(ndt_base_pose.pitch, predicted_base_pose.pitch);
    diff_yaw = diff_radian(ndt_base_pose.yaw, predicted_base_pose.yaw);
    translation_error = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    rotation_error = sqrt(diff_roll * diff_roll + diff_pitch * diff_pitch + diff_yaw * diff_yaw);

    // Get current map to lidar transform
    Pose base_pose;
    Eigen::Matrix4f map2base_matrix, map2lidar_matrix;
    if ((translation_error > translation_error_tolerance_ || rotation_error > rotation_error_tolerance_)
        && prediction_method_ == ODOM)
    {
        // Limit pose difference
        ROS_WARN_THROTTLE(1, "NDT pose is %.3fm and %.1fdeg apart from prediction. Limiting pose difference.", translation_error, rotation_error * 180 / M_PI);
        if (translation_error > translation_error_tolerance_)
        {
            double translation_ratio = translation_error_tolerance_ / translation_error;
            predicted_base_pose.x += diff_x * translation_ratio;
            predicted_base_pose.y += diff_y * translation_ratio;
            predicted_base_pose.z += diff_z * translation_ratio;
        }
        if (rotation_error > rotation_error_tolerance_)
        {
            double rotation_ratio = rotation_error_tolerance_ / rotation_error;
            predicted_base_pose.roll += diff_roll * rotation_ratio;
            predicted_base_pose.pitch += diff_pitch * rotation_ratio;
            predicted_base_pose.yaw += diff_yaw * rotation_ratio;
        }

        base_pose = predicted_base_pose;
        map2base_matrix = convert_pose2matrix(base_pose);
        map2lidar_matrix = base2lidar_matrix_ * map2base_matrix;
    }
    else
    {
        // Use ndt pose
        base_pose = ndt_base_pose;
        map2base_matrix = ndt_map2base_matrix;
        map2lidar_matrix = ndt_map2lidar_matrix;
    }

    // Update target map if shift is large enough
    double shift_squared;
    diff_x = base_pose.x - last_map_updated_base_pose_.x;
    diff_y = base_pose.y - last_map_updated_base_pose_.y;
    diff_z = base_pose.z - last_map_updated_base_pose_.z;
    shift_squared = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
    if (shift_squared >= submap_update_shift_ * submap_update_shift_
        && use_submaps_)
    {
        // Update last map updated base pose
        last_map_updated_base_pose_ = base_pose;

        // Update submap distances
        update_submap_distances(base_pose, submap_map_);

        // Update target map
        target_map_.clear();
        std::unordered_set<int> target_id_set;
        target_id_set = get_neighbor_ids(submap_map_, submap_include_distance_);
        for (auto id : target_id_set)
            target_map_.operator+=(submap_map_[id].points);
        ndt_.setInputTarget(target_map_.makeShared());

        // Publish target map
        sensor_msgs::PointCloud2 target_map_msg;
        target_map_.header.frame_id = map_frame_;
        pcl::toROSMsg(target_map_, target_map_msg);
        target_map_pub_.publish(target_map_msg);
    }

    // Publish tf
    geometry_msgs::TransformStamped map2base_tf;
    map2base_tf = convert_matrix2tf(map2base_matrix);
    map2base_tf.header.stamp = msg->header.stamp;
    map2base_tf.header.frame_id = map_frame_;
    map2base_tf.child_frame_id = base_frame_;
    tf_broadcaster_.sendTransform(map2base_tf);

    // Publish matching points
    if (points_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 matching_points_msg;
        scan_aligned->header.frame_id = map_frame_;
        pcl::toROSMsg(*scan_aligned, matching_points_msg);
        points_pub_.publish(matching_points_msg);
    }

    // Publish convergence status
    std_msgs::Bool convergence_msg;
    convergence_msg.data = has_convergered;
    convergence_pub_.publish(convergence_msg);

    // Publish fitness score
    std_msgs::Float32 fitness_score_msg;
    fitness_score_msg.data = fitness_score;
    fitness_score_pub_.publish(fitness_score_msg);

    // Publish number of iterations
    std_msgs::Int32 num_iteration_msg;
    num_iteration_msg.data = final_num_iteration;
    num_iteration_pub_.publish(num_iteration_msg);

    // Publish transformation probability
    std_msgs::Float32 trans_prob_msg;
    trans_prob_msg.data = transformation_probability;
    trans_prob_pub_.publish(trans_prob_msg);

    // Set values for next callback
    last_base_twist_.linear.x = (base_pose.x - last_base_pose_.x) / dt;
    last_base_twist_.linear.y = (base_pose.y - last_base_pose_.y) / dt;
    last_base_twist_.linear.z = (base_pose.z - last_base_pose_.z) / dt;
    last_base_twist_.angular.x = diff_radian(base_pose.roll, last_base_pose_.roll) / dt;
    last_base_twist_.angular.y = diff_radian(base_pose.pitch, last_base_pose_.pitch) / dt;
    last_base_twist_.angular.z = diff_radian(base_pose.yaw, last_base_pose_.yaw) / dt;
    last_base_pose_ = base_pose;
    last_scan_stamp_ = msg->header.stamp;
    last_predicted_base_pose_ = base_pose;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_localizer");
    NDTLocalizer ndt_localizer;
    ros::spin();

    return 0;
}
