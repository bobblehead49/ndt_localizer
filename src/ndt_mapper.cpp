#include <ndt_localizer/ndt_mapper.hpp>
#include <ndt_localizer/ndt_tools.hpp>

#include <string>
#include <sstream>
#include <queue>
#include <set>
#include <unordered_set>
#include <map>
#include <vector>
#include <limits>
#include <filesystem>
#include <fstream>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// Constructor
NDTMapper::NDTMapper() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
    // Get parameters
    std::string prediction_method_str;
    int ndt_max_iterations;
    float ndt_resolution, ndt_step_size, ndt_transformation_epsilon;
    float deg_error_tolerance;
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
    pnh_.param<std::string>("points_topic", points_topic_, "/points");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    pnh_.param<std::string>("map_topic", map_topic_, "/map");
    pnh_.param<float>("map_publish_interval", map_publish_interval_, 15.0);
    pnh_.param<float>("min_scan_range", min_scan_range_, 0.3);
    pnh_.param<float>("max_scan_range", max_scan_range_, 100.0);
    pnh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.5);
    pnh_.param<int>("ndt_max_iterations", ndt_max_iterations, 30);
    pnh_.param<float>("ndt_resolution", ndt_resolution, 2.0);
    pnh_.param<float>("ndt_step_size", ndt_step_size, 0.1);
    pnh_.param<float>("ndt_transformation_epsilon", ndt_transformation_epsilon, 0.001);
    pnh_.param<std::string>("prediction_method", prediction_method_str, "odom");
    pnh_.param<float>("translation_error_tolerance", translation_error_tolerance_, 0.8);
    pnh_.param<float>("rotation_error_tolerance", deg_error_tolerance, 8.0);
    pnh_.param<float>("map_add_shift", map_add_shift_, 1.0);
    pnh_.param<int>("submap_scan_size", submap_scan_size_, 1);
    pnh_.param<float>("submap_include_distance", submap_include_distance_, 30.0);
    pnh_.param<float>("submap_connect_distance", submap_connect_distance_, 5.0);
    pnh_.param<bool>("use_loop_closure", use_loop_closure_, true);
    pnh_.param<float>("loop_connect_distance", loop_connect_distance_, 15.0);
    pnh_.param<float>("loop_closure_confirmation_error", loop_closure_confirmation_error_, 0.05);
    pnh_.param<bool>("save_uncompressed_map", save_uncompressed_map_, true);
    pnh_.param<bool>("save_submaps", save_submaps_, true);

    // Convert parameters
    prediction_method_ = convert_prediction_method(prediction_method_str);
    rotation_error_tolerance_ = deg_error_tolerance * M_PI / 180.0;

    // Initialize variables
    map_initialized_ = false;
    odom_initialized_ = false;
    adjusted_loop_with_last_scan_ = false;
    submap_scan_count_ = 0;
    current_submap_id_ = 0;
    current_group_id_ = 0;
    init_submap_with_info_.in_group = false;
    init_submap_with_info_.group_id = -1;
    init_submap_with_info_.distance = 0;
    init_submap_with_info_.position_matrix = Eigen::Matrix4f::Identity();
    last_base_pose_ = {0, 0, 0, 0, 0, 0};
    last_added_base_pose_ = {0, 0, 0, 0, 0, 0};
    last_predicted_base_pose_ = {0, 0, 0, 0, 0, 0};
    last_base_twist_.linear.x = last_base_twist_.linear.y = last_base_twist_.linear.z = 0;
    last_base_twist_.angular.x = last_base_twist_.angular.y = last_base_twist_.angular.z = 0;

    // Get static transforms
    ros::Rate wait_rate(2);
    geometry_msgs::TransformStamped base2lidar_tf;
    while (ros::ok() && !get_tf(tf_buffer_, base_frame_, lidar_frame_, base2lidar_tf))
        wait_rate.sleep();
    base2lidar_matrix_ = convert_tf2matrix(base2lidar_tf);
    lidar2base_matrix_ = base2lidar_matrix_.inverse();

    // Setup ndt
    ndt_.setResolution(ndt_resolution);
    ndt_.setStepSize(ndt_step_size);
    ndt_.setTransformationEpsilon(ndt_transformation_epsilon);
    ndt_.setMaximumIterations(ndt_max_iterations);

    // Setup subscribers and publishers
    points_sub_ = nh_.subscribe(points_topic_, 1000000, &NDTMapper::points_callback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 1000000, &NDTMapper::odom_callback, this);
    map_save_request_sub_ = nh_.subscribe("/ndt_mapper/map_save_request", 1, &NDTMapper::map_save_request_callback, this);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 1, true);
    target_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_mapper/target_map", 1, true);
    points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_mapper/points", 1, true);
    convergence_pub_ = nh_.advertise<std_msgs::Bool>("/ndt_mapper/convergence", 1);
    fitness_score_pub_ = nh_.advertise<std_msgs::Float32>("/ndt_mapper/fitness_score", 1);
    num_iteration_pub_ = nh_.advertise<std_msgs::Int32>("/ndt_mapper/iteration_num", 1);
    trans_prob_pub_ = nh_.advertise<std_msgs::Float32>("/ndt_mapper/transformation_probability", 1);
    deque_pub_ = nh_.advertise<std_msgs::Empty>("/ndt_mapper/deque", 1000000);
}

// Destructor
NDTMapper::~NDTMapper(){}

// Gets the submap ids to include in the target map.
std::unordered_set<int> NDTMapper::get_locally_connected_ids(const int target_id, const float include_distance)
{
    std::unordered_set<int> connected_id_set;
    std::queue<int> search_id_queue;
    search_id_queue.push(target_id);
    while (!search_id_queue.empty())
    {
        int search_id = search_id_queue.front();
        search_id_queue.pop();
        if (submap_map_[search_id].distance <= include_distance)
        {
            connected_id_set.insert(search_id);
            std::unordered_set<int> adjacent_nodes;
            adjacent_nodes = global_pose_graph_.get_adjacent_nodes(search_id);
            for (auto adjacent_node : adjacent_nodes)
                if (connected_id_set.find(adjacent_node) == connected_id_set.end())
                    search_id_queue.push(adjacent_node);
        }
    }
    return connected_id_set;
}

// Classifies submaps to a group.
void NDTMapper::group_submaps(const std::unordered_set<int>& connected_id_set)
{
    // Get search area for nearby groups.
    std::unordered_set<int> search_id_set;
    search_id_set = connected_id_set;
    for (auto id : connected_id_set)
    {
        std::unordered_set<int> adjacent_nodes;
        adjacent_nodes = global_pose_graph_.get_adjacent_nodes(id);
        search_id_set.merge(adjacent_nodes);
    }

    // Get nearby group ids.
    std::set<int> group_id_set;
    for (auto id : search_id_set)
    {
        if (submap_map_[id].in_group)
            group_id_set.insert(submap_map_[id].group_id);
    }

    // Get group id for the target submaps.
    int group_id;
    if (group_id_set.empty())
        group_id = current_group_id_++;
    else
    {
        group_id = *group_id_set.begin();
        group_id_set.erase(group_id_set.begin());
    }

    // Set group id for the target submaps.
    for (auto id : connected_id_set)
    {
        submap_map_[id].in_group = true;
        submap_map_[id].group_id = group_id;
    }

    // Merge nearby groups.
    for (auto id : group_id_set)
    {
        for (auto submap : submap_map_)
        {
            if (submap.second.in_group
                && submap.second.group_id == id)
                submap.second.group_id = group_id;
        }
    }
}

// Gets the target submap ids for loop closure.
std::unordered_set<int> NDTMapper::get_loop_target_ids(const std::unordered_set<int>& connected_id_set)
{
    std::unordered_set<int> loop_target_id_set;
    loop_target_id_set = get_neighbor_ids(submap_map_, loop_connect_distance_);

    // Remove connected ids
    for (auto id : connected_id_set)
        loop_target_id_set.erase(id);

    // Add locally connected ids
    std::unordered_set<int> temp_id_set, target_map_id_set;
    for (auto id : loop_target_id_set)
    {
        temp_id_set = get_locally_connected_ids(id, submap_include_distance_);
        target_map_id_set.merge(temp_id_set);
    }

    return target_map_id_set;
}

// Apply loop closure to the given submaps.
void NDTMapper::get_loop_correction(const std::unordered_set<int>& loop_target_id_set, Eigen::Matrix4f& correction_matrix, float& fitness_score, bool& convergence, Pose& initial_guess_correction)
{
    // Set target map
    pcl::PointCloud<pcl::PointXYZI> target_cloud;
    for (auto id : loop_target_id_set)
        target_cloud.operator+=(submap_map_[id].points);
    ndt_.setInputTarget(target_cloud.makeShared());

    // Set source map
    pcl::PointCloud<pcl::PointXYZI> source_cloud;
    source_cloud = submap_map_[current_submap_id_].points;
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    apply_voxel_grid_filter(source_cloud.makeShared(), source_voxel_filtered, voxel_leaf_size_);
    ndt_.setInputSource(source_voxel_filtered);

    // Get closest loop target id
    float min_distance;
    int closest_loop_target_id;
    min_distance = std::numeric_limits<float>::max();
    for (auto id : loop_target_id_set)
    {
        if (submap_map_[id].distance < min_distance)
        {
            min_distance = submap_map_[id].distance;
            closest_loop_target_id = id;
        }
    }

    // Prepare relative matrix
    Eigen::Matrix4f source_translation_matrix, source_orientation_matrix, target_translation_matrix, target_orientation_matrix, relative_matrix;
    source_translation_matrix = source_orientation_matrix = \
        submap_map_[current_submap_id_].position_matrix * base2lidar_matrix_;
    source_translation_matrix.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    source_orientation_matrix.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();
    target_translation_matrix = target_orientation_matrix = \
        submap_map_[closest_loop_target_id].position_matrix * base2lidar_matrix_;
    target_translation_matrix.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    target_orientation_matrix.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();
    relative_matrix = source_translation_matrix.inverse() * target_translation_matrix * source_orientation_matrix.inverse() * target_orientation_matrix;

    // Get relative pose
    Pose relative_pose = convert_matrix2pose(relative_matrix);    

    // Prepare orientation matrix
    Eigen::Vector4f target_z_axis = \
        target_orientation_matrix * Eigen::Vector4f::UnitZ();
    Eigen::Matrix3f rotation_matrix_3x3 = \
        Eigen::AngleAxisf(-relative_pose.yaw, target_z_axis.head<3>()).toRotationMatrix();
    Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
    rotation_matrix.block<3, 3>(0, 0) = rotation_matrix_3x3;

    // Prepare main initial guess
    Eigen::Matrix4f main_initial_guess = \
        source_orientation_matrix.inverse() * rotation_matrix * target_orientation_matrix;
    main_initial_guess(2, 3) = relative_matrix(2, 3);

    // Publish tf
    Eigen::Matrix4f map2main_initial_guess_matrix;
    map2main_initial_guess_matrix = source_translation_matrix * rotation_matrix * target_orientation_matrix;
    map2main_initial_guess_matrix(2, 3) += relative_matrix(2, 3);
    geometry_msgs::TransformStamped map2main_initial_guess_tf;
    map2main_initial_guess_tf = convert_matrix2tf(map2main_initial_guess_matrix);
    map2main_initial_guess_tf.header.stamp = ros::Time::now();
    map2main_initial_guess_tf.header.frame_id = map_frame_;
    map2main_initial_guess_tf.child_frame_id = "main_initial_guess";
    tf_broadcaster_.sendTransform(map2main_initial_guess_tf);

    geometry_msgs::TransformStamped target_tf;
    target_tf = convert_matrix2tf(target_translation_matrix * target_orientation_matrix);
    target_tf.header.stamp = ros::Time::now();
    target_tf.header.frame_id = map_frame_;
    target_tf.child_frame_id = "target";
    tf_broadcaster_.sendTransform(target_tf);

    // Print pose
    Pose source_pose, target_pose, initial_guess_pose;
    source_pose = convert_matrix2pose(source_translation_matrix * source_orientation_matrix);
    target_pose = convert_matrix2pose(target_translation_matrix * target_orientation_matrix);
    initial_guess_pose = convert_matrix2pose(main_initial_guess);
    ROS_INFO("source_pose:\n (x, y, z, roll, pitch, yaw): (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)", source_pose.x, source_pose.y, source_pose.z, source_pose.roll*180.0/M_PI, source_pose.pitch*180.0/M_PI, source_pose.yaw*180.0/M_PI);
    ROS_INFO("target_pose:\n (x, y, z, roll, pitch, yaw): (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)", target_pose.x, target_pose.y, target_pose.z, target_pose.roll*180.0/M_PI, target_pose.pitch*180.0/M_PI, target_pose.yaw*180.0/M_PI);
    ROS_INFO("initial_guess_pose:\n (x, y, z, roll, pitch, yaw): (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)", initial_guess_pose.x, initial_guess_pose.y, initial_guess_pose.z, initial_guess_pose.roll*180.0/M_PI, initial_guess_pose.pitch*180.0/M_PI, initial_guess_pose.yaw*180.0/M_PI);

    std::vector<Eigen::Matrix4f> correction_matrix_vector;
    std::vector<float> fitness_score_vector;
    std::vector<bool> convergence_vector;
    Eigen::Matrix4f best_fit_correction_matrix = Eigen::Matrix4f::Identity();
    float best_fit_score = std::numeric_limits<float>::max();
    bool best_fit_convergence = false;
    for (int i = 0; i < 1; i++)
    {
        ROS_INFO("Loop closure trial: %d", i);

        // Prepare initial guess vector
        std::vector<Eigen::Matrix4f> initial_guess_vector;
        for (int x = -i ; x <= i ; x++)
        {
            for (int y=-i ; y<=i ; y++)
            {
                if (x == i || y == i || x == -i || y == -i)
                {
                    Eigen::Matrix4f initial_guess = main_initial_guess;
                    initial_guess(0, 3) += 1.0 * x;
                    initial_guess(1, 3) += 1.0 * y;
                    initial_guess_vector.push_back(initial_guess);
                }
            }
        }

        // Align
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_aligned(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto initial_guess : initial_guess_vector)
        {
            if (!ros::ok()) break;

            Eigen::Matrix4f map2initial_guess_matrix, initial_guess_translation_matrix, initial_guess_orientation_matrix;
            initial_guess_translation_matrix = initial_guess_orientation_matrix = initial_guess;
            initial_guess_translation_matrix.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
            initial_guess_orientation_matrix.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();
            map2initial_guess_matrix = source_translation_matrix * initial_guess_translation_matrix * source_orientation_matrix * initial_guess_orientation_matrix;
            geometry_msgs::TransformStamped map2initial_guess_tf;
            map2initial_guess_tf = convert_matrix2tf(map2initial_guess_matrix);
            map2initial_guess_tf.header.stamp = ros::Time::now();
            map2initial_guess_tf.header.frame_id = map_frame_;
            map2initial_guess_tf.child_frame_id = "initial_guess";
            tf_broadcaster_.sendTransform(map2initial_guess_tf);

            ndt_.align(*scan_aligned, initial_guess);
            correction_matrix_vector.push_back(ndt_.getFinalTransformation());
            fitness_score_vector.push_back(ndt_.getFitnessScore());
            convergence_vector.push_back(ndt_.hasConverged());
            std::cout << "." << std::flush;
        }
        std::cout << std::endl;
        if (!ros::ok()) break;

        // Get stats
        float best_fit_idx = \
            std::distance(fitness_score_vector.begin(), std::min_element(fitness_score_vector.begin(), fitness_score_vector.end()));
        best_fit_correction_matrix = correction_matrix_vector[best_fit_idx];
        best_fit_score = fitness_score_vector[best_fit_idx];
        best_fit_convergence = convergence_vector[best_fit_idx];

        ROS_INFO("Best fitness score: %.3f", best_fit_score);
        ROS_INFO_STREAM("Best fit correction matrix:\n" << best_fit_correction_matrix);
        ROS_INFO("Best fit convergence: %s", best_fit_convergence ? "true" : "false");

        if (best_fit_score < 0.4 || best_fit_score > 1000.0) break;
    }

    // Get best fit stats
    correction_matrix = best_fit_correction_matrix;
    fitness_score = best_fit_score;
    convergence = best_fit_convergence;

    // Get error from initial guess
    initial_guess_correction = convert_matrix2pose(correction_matrix * main_initial_guess.inverse());
}

// Get node id path from start to goal.
std::vector<int> NDTMapper::get_loop_id_path(const std::unordered_set<int>& start_id_candidates, const int goal_id)
{
    int length;
    length = std::numeric_limits<int>::max();
    std::vector<int> loop_id_path;
    for (auto id : start_id_candidates)
    {
        std::vector<int> temp_id_path;
        temp_id_path = global_pose_graph_.get_shortest_path(id, current_submap_id_);
        if ((int)temp_id_path.size() < length)
        {
            length = temp_id_path.size();
            loop_id_path = temp_id_path;
        }
    }
    return loop_id_path;
}

// Get node id path from start to goal with ids in same groups removed.
std::vector<int> NDTMapper::get_grouped_loop_id_path(const std::vector<int>& loop_id_path)
{
    std::vector<int> grouped_loop_id_path;
    std::unordered_set<int> checked_groups;
    for (auto id : loop_id_path)
    {
        if (submap_map_[id].in_group)
        {
            // Skip if group is already checked
            if (checked_groups.find(submap_map_[id].group_id) != checked_groups.end())
                continue;
            else
                checked_groups.insert(submap_map_[id].group_id);
        }
        grouped_loop_id_path.push_back(id);
    }
    return grouped_loop_id_path;
}

void NDTMapper::adjust_angles(const Pose& loop_correction_pose, std::map<int, Eigen::Matrix4f>& destination_matrix_map)
{
    // Get rotation matrix
    Pose rotation_pose;
    Eigen::Matrix4f rotation_matrix;
    rotation_pose.x = rotation_pose.y = rotation_pose.z = 0.0;
    rotation_pose.roll = loop_correction_pose.roll / (destination_matrix_map.size()-1);
    rotation_pose.pitch = loop_correction_pose.pitch / (destination_matrix_map.size()-1);
    rotation_pose.yaw = loop_correction_pose.yaw / (destination_matrix_map.size()-1);
    rotation_matrix = convert_pose2matrix(rotation_pose);

    // Rotate all destination matrices
    for (auto it = destination_matrix_map.begin(); it != destination_matrix_map.end(); it++)
    {
        // Get center of rotate matrix
        Eigen::Matrix4f center_matrix = it->second;
        center_matrix.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

        // Get transform matrix
        Eigen::Matrix4f transform_matrix = \
            center_matrix * rotation_matrix * center_matrix.inverse();

        // Rotate all destination matrices after the current matrix
        for (auto it2 = std::next(it); it2 != destination_matrix_map.end(); it2++)
            it2->second = transform_matrix * it2->second;
    }
}

void NDTMapper::adjust_positions(const Pose& loop_correction_pose, std::map<int, Eigen::Matrix4f>& destination_matrix_map)
{
    // Get translation matrix
    Eigen::Matrix4f translation_matrix;
    translation_matrix = Eigen::Matrix4f::Identity();
    translation_matrix(0, 3) = loop_correction_pose.x / (destination_matrix_map.size()-1);
    translation_matrix(1, 3) = loop_correction_pose.y / (destination_matrix_map.size()-1);
    translation_matrix(2, 3) = loop_correction_pose.z / (destination_matrix_map.size()-1);

    // Translate all destination matrices
    for (auto it = destination_matrix_map.begin(); it != destination_matrix_map.end(); it++)
    {
        // Translate all destination matrices after the current matrix
        for (auto it2 = std::next(it); it2 != destination_matrix_map.end(); it2++)
            it2->second = translation_matrix * it2->second;
    }
}

void NDTMapper::shift_submaps(const std::map<int, Eigen::Matrix4f>& destination_matrix_map)
{
    // Shift submaps
    for (auto it = destination_matrix_map.begin(); it != destination_matrix_map.end(); it++)
    {
        int submap_id;
        Eigen::Matrix4f destination_matrix, transform_matrix;
        submap_id = it->first;
        destination_matrix = it->second;
        transform_matrix = destination_matrix * submap_map_[submap_id].position_matrix.inverse();

        // Shift submaps if not in a group
        if (!submap_map_[submap_id].in_group)
        {
            pcl::transformPointCloud(submap_map_[submap_id].points, submap_map_[submap_id].points, transform_matrix);
            submap_map_[submap_id].position_matrix = destination_matrix;
            continue;
        }

        // Shift submaps in same group
        for (auto it2 = submap_map_.begin(); it2 != submap_map_.end(); it2++)
        {
            if (it2->second.in_group
                && it2->second.group_id == submap_map_[submap_id].group_id)
            {
                pcl::transformPointCloud(it2->second.points, it2->second.points, transform_matrix);
                it2->second.position_matrix = transform_matrix * it2->second.position_matrix;
            }
        }
    }
}

// Close loop.
bool NDTMapper::close_loop(const std::unordered_set<int>& loop_target_id_set, Pose& base_pose)
{
    // Get loop correction
    Eigen::Matrix4f loop_correction_matrix;
    Pose loop_correction_pose, initial_guess_correction;
    float loop_fitness_score;
    bool loop_convergence;
    float initial_guess_correction_distance;

    get_loop_correction(loop_target_id_set, loop_correction_matrix, loop_fitness_score, loop_convergence, initial_guess_correction);

    loop_correction_pose = convert_matrix2pose(loop_correction_matrix);
    initial_guess_correction_distance = sqrt(initial_guess_correction.x * initial_guess_correction.x + initial_guess_correction.y * initial_guess_correction.y);
    ROS_INFO("Loop correction:\n (x, y, z, roll, pitch, yaw): (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)", loop_correction_pose.x, loop_correction_pose.y, loop_correction_pose.z, loop_correction_pose.roll*180.0/M_PI, loop_correction_pose.pitch*180.0/M_PI, loop_correction_pose.yaw*180.0/M_PI);
    ROS_INFO("Error from initial guess:\n (x, y, z, roll, pitch, yaw): (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)", initial_guess_correction.x, initial_guess_correction.y, initial_guess_correction.z, initial_guess_correction.roll*180.0/M_PI, initial_guess_correction.pitch*180.0/M_PI, initial_guess_correction.yaw*180.0/M_PI);

    if (!loop_convergence || loop_fitness_score > 1000.0 || initial_guess_correction_distance > 10.0)
    {
        ROS_WARN("Loop closure aborted.\n Fitness score: %.3f, Initial guess correction distance: %.3f", loop_fitness_score, initial_guess_correction_distance);
        return false;
    }
    if (abs(initial_guess_correction.z) > 1.0)
        loop_correction_pose.z -= initial_guess_correction.z;

    // Get shortest path to root submap
    std::vector<int> loop_id_path;
    loop_id_path = get_loop_id_path(loop_target_id_set, current_submap_id_);

    // Prepare grouped loop id path
    std::vector<int> grouped_loop_id_path;
    grouped_loop_id_path = get_grouped_loop_id_path(loop_id_path);

    // Prepare destination matrix map
    std::map<int, Eigen::Matrix4f> destination_matrix_map;
    for (auto id : grouped_loop_id_path)
        destination_matrix_map.insert(std::make_pair(id, submap_map_[id].position_matrix));
    adjust_angles(loop_correction_pose, destination_matrix_map);
    adjust_positions(loop_correction_pose, destination_matrix_map);

    // Close loop
    shift_submaps(destination_matrix_map);

    // Update base pose
    Eigen::Matrix4f shifted_map2base_matrix;
    Pose shifted_map2base_pose;
    shifted_map2base_matrix = std::prev(destination_matrix_map.end())->second;
    shifted_map2base_pose = convert_matrix2pose(shifted_map2base_matrix);
    base_pose = last_base_pose_ = shifted_map2base_pose;

    // Update global pose graph if loop closure is confirmed
    ROS_INFO("Fitness score: %.3f", loop_fitness_score);
    if (initial_guess_correction_distance < loop_closure_confirmation_error_ && loop_fitness_score < 0.1)
    {
        for (auto id : loop_target_id_set)
            global_pose_graph_.add_edge(current_submap_id_, id);
        std::unordered_set<int> connected_id_set;
        connected_id_set = loop_target_id_set;
        connected_id_set.insert(current_submap_id_);
        for (auto id : loop_id_path)
            connected_id_set.insert(id);
        group_submaps(connected_id_set);
        ROS_INFO("Loop closed. Final correction distance: %.3fm", initial_guess_correction_distance);
    }

    return true;
}

// Update maps.
void NDTMapper::update_maps(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_for_mapping, const Eigen::Matrix4f& map2base_matrix, Pose& base_pose)
{
    // Add scan to maps
    submap_.operator+=(*scan_for_mapping);
    target_map_.operator+=(*scan_for_mapping);

    // Update submap scan count
    submap_scan_count_++;

    // Update last added base pose
    last_added_base_pose_ = base_pose;

    if (submap_scan_count_ >= submap_scan_size_)
    {
        // Update submap distances
        update_submap_distances(base_pose, submap_map_);

        // Add submap to map history
        SubmapWithInfo submap_with_info;
        submap_with_info = init_submap_with_info_;
        submap_with_info.points = submap_;
        submap_with_info.position_matrix = map2base_matrix;
        submap_map_.insert(std::make_pair(current_submap_id_, submap_with_info));
        global_pose_graph_.add_edge(current_submap_id_-1, current_submap_id_);

        // Get submap ids to add to target map
        std::unordered_set<int> target_id_set;
        if (use_loop_closure_)
        {
            target_id_set = get_locally_connected_ids(current_submap_id_, submap_include_distance_);

            // Check for loops
            std::unordered_set<int> loop_target_id_set;
            loop_target_id_set = get_loop_target_ids(target_id_set);

            // Enter loop closing process if loop is detected
            if (loop_target_id_set.size() >= 5)
                adjusted_loop_with_last_scan_ = close_loop(loop_target_id_set, base_pose);
        }
        else
            target_id_set = get_neighbor_ids(submap_map_, submap_include_distance_);

        // Update target map
        target_map_.clear();
        for (auto id : target_id_set)
        {
            target_map_.operator+=(submap_map_[id].points);

            // Update global pose graph and group submaps
            if (submap_map_[id].distance <= submap_connect_distance_
                && current_submap_id_ - id > (int)(submap_connect_distance_ / (map_add_shift_ * submap_scan_size_)) + 1
                && !adjusted_loop_with_last_scan_)
            {
                global_pose_graph_.add_edge(current_submap_id_, id);
                std::unordered_set<int> connected_id_set({id, current_submap_id_});
                group_submaps(connected_id_set);
            }
        }

        // Reset submap
        submap_scan_count_ = 0;
        submap_.clear();
        current_submap_id_++;
    }

    // Update ndt target
    ndt_.setInputTarget(target_map_.makeShared());

    // Publish map
    if ((ros::Time::now() - last_map_publish_stamp_).toSec() > map_publish_interval_
        && map_pub_.getNumSubscribers() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI> map;
        for (auto submap_with_info : submap_map_)
            map.operator+=(submap_with_info.second.points);
        map.header.frame_id = map_frame_;
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(map, map_msg);
        map_pub_.publish(map_msg);
        last_map_publish_stamp_ = ros::Time::now();
    }

    // Publish target map
    if (target_map_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 target_map_msg;
        target_map_.header.frame_id = map_frame_;
        pcl::toROSMsg(target_map_, target_map_msg);
        target_map_pub_.publish(target_map_msg);
    }
}

// Callback for odometry messages
void NDTMapper::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    if (!odom_initialized_)
    {
        last_odom_ = *msg;
        odom_initialized_ = true;
        return;
    }

    // Get current and last odom matrices
    Eigen::Matrix4f current_odom_matrix, last_odom_matrix, last_odom_translation_matrix, last_odom_orientation_matrix, last_map2base_matrix;
    current_odom_matrix = convert_odom2matrix(*msg);
    last_odom_matrix = convert_odom2matrix(last_odom_);
    last_odom_translation_matrix = last_odom_matrix;
    last_odom_translation_matrix.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    last_odom_orientation_matrix = last_odom_matrix;
    last_odom_orientation_matrix.block<3, 1>(0, 3) = Eigen::Vector3f(0.0, 0.0, 0.0);
    last_map2base_matrix = convert_pose2matrix(last_base_pose_);

    // Get map to base matrix
    Eigen::Matrix4f transform_matrix, map2base_matrix;
    transform_matrix = last_odom_translation_matrix.inverse() * current_odom_matrix;
    transform_matrix = last_odom_orientation_matrix.inverse() * transform_matrix;
    Pose transform_pose;
    transform_pose = convert_matrix2pose(transform_matrix);
    float ratio = 10.0;
    transform_pose.x *= ratio;
    transform_pose.y *= ratio;
    transform_pose.z *= ratio;
    transform_pose.roll *= ratio;
    transform_pose.pitch *= ratio;
    transform_pose.yaw *= ratio;
    transform_matrix = convert_pose2matrix(transform_pose);
    map2base_matrix = last_map2base_matrix * transform_matrix;

    // Update prediction
    last_predicted_base_pose_ = convert_matrix2pose(map2base_matrix);

    // Set values for next callback
    last_odom_ = *msg;
}

// Callback for point cloud messages
void NDTMapper::points_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!ros::ok())
        return;

    // Publish deque message
    std_msgs::Empty empty_msg;
    deque_pub_.publish(empty_msg);

    double dt = (msg->header.stamp - last_scan_stamp_).toSec();
    if (dt < 0.0)
        return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_for_mapping(new pcl::PointCloud<pcl::PointXYZI>);

    // Convert scan data
    pcl::fromROSMsg(*msg, *scan);

    // Apply range filter
    apply_range_filter(scan, scan_filtered, min_scan_range_, max_scan_range_);

    // Apply voxel grid filter
    apply_voxel_grid_filter(scan_filtered, scan_filtered, voxel_leaf_size_);

    // Initialize maps
    if (!map_initialized_)
    {
        // Transform scan to mapping point
        pcl::transformPointCloud(*scan_filtered, *scan_for_mapping, base2lidar_matrix_);

        // Initialize submap
        submap_ = *scan_for_mapping;
        SubmapWithInfo submap_with_info;
        submap_with_info = init_submap_with_info_;
        submap_with_info.points = submap_;
        submap_map_.insert(std::make_pair(current_submap_id_++, submap_with_info));
        submap_.clear();

        // Initialize target map
        target_map_ = *scan_for_mapping;
        ndt_.setInputTarget(target_map_.makeShared());

        // Set values for next callback
        last_scan_stamp_ = msg->header.stamp;
        last_map_publish_stamp_ = ros::Time::now();
        map_initialized_ = true;
        return;
    }

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
    predicted_map2lidar_matrix = predicted_map2base_matrix * base2lidar_matrix_;

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
    ndt_map2base_matrix = ndt_map2lidar_matrix * lidar2base_matrix_;
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

    // Get matrix for mapping
    Pose base_pose;
    Eigen::Matrix4f map2base_matrix, map2lidar_matrix;
    if ((translation_error > translation_error_tolerance_ || rotation_error > rotation_error_tolerance_)
        && prediction_method_ == ODOM
        && !adjusted_loop_with_last_scan_)
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
        map2lidar_matrix = map2base_matrix * base2lidar_matrix_;
    }
    else
    {
        // Use ndt pose
        base_pose = ndt_base_pose;
        map2base_matrix = ndt_map2base_matrix;
        map2lidar_matrix = ndt_map2lidar_matrix;
    }

    // Transform scan to mapping point
    pcl::transformPointCloud(*scan_filtered, *scan_for_mapping, map2lidar_matrix);

    // Update maps if shift is large enough
    double shift_squared;
    diff_x = base_pose.x - last_added_base_pose_.x;
    diff_y = base_pose.y - last_added_base_pose_.y;
    diff_z = base_pose.z - last_added_base_pose_.z;
    shift_squared = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
    if (shift_squared >= map_add_shift_ * map_add_shift_)
        update_maps(scan_for_mapping, map2base_matrix, base_pose);

    // Publish tf
    geometry_msgs::TransformStamped map2base_tf;
    map2base_tf = convert_matrix2tf(map2base_matrix);
    map2base_tf.header.stamp = msg->header.stamp;
    map2base_tf.header.frame_id = map_frame_;
    map2base_tf.child_frame_id = base_frame_;
    tf_broadcaster_.sendTransform(map2base_tf);

    // Publish mapping points
    if (points_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 mapping_points_msg;
        scan_for_mapping->header.frame_id = map_frame_;
        pcl::toROSMsg(*scan_for_mapping, mapping_points_msg);
        points_pub_.publish(mapping_points_msg);
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

// Callback for map save request message.
void NDTMapper::map_save_request_callback(const std_msgs::StringConstPtr& msg)
{
    if (!ros::ok())
        return;

    // Get entire map
    pcl::PointCloud<pcl::PointXYZI> map;
    for (auto submap_with_info : submap_map_)
        map.operator+=(submap_with_info.second.points);
    map.header.frame_id = map_frame_;

    // Save full map
    std::filesystem::path filepath, out_dir, out_file;
    out_dir = msg->data;
    filepath = out_dir / "map.pcd";
    ROS_INFO("Saving full map to %s", filepath.c_str());
    pcl::io::savePCDFileBinaryCompressed(filepath, map);
    ROS_INFO("Saved full map to %s", filepath.c_str());

    if (!ros::ok())
        return;

    if (save_uncompressed_map_)
    {
        // Save uncompressed full map
        filepath = out_dir / "uncompressed_map.pcd";
        ROS_INFO("Saving uncompressed full map to %s", filepath.c_str());
        pcl::io::savePCDFileASCII(filepath, map);
        ROS_INFO("Saved uncompressed full map to %s", filepath.c_str());
    }

    if (save_submaps_)
    {
        // Prepare csv for submap info
        filepath = out_dir / "submaps" / "submap.csv";
        std::ofstream ofs(filepath);

        // Save submaps
        ROS_INFO("Saving submaps to %s", (out_dir / "submaps").c_str());
        for (auto submap_with_info : submap_map_)
        {
            if (!ros::ok())
                return;

            // Save submap info
            ofs << submap_with_info.first << ",";
            ofs << submap_with_info.second.position_matrix(0, 3) << ",";
            ofs << submap_with_info.second.position_matrix(1, 3) << ",";
            ofs << submap_with_info.second.position_matrix(2, 3) << std::endl;

            // Save submap points
            out_file = "submap" + std::to_string(submap_with_info.first) + ".pcd";
            filepath = out_dir / "submaps" / out_file;
            pcl::io::savePCDFileBinaryCompressed(filepath, submap_with_info.second.points);
        }
        ROS_INFO("Saved submaps to %s", (out_dir / "submaps").c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_mapper");
    NDTMapper ndt_mapper;
    ros::spin();

    return 0;
}
