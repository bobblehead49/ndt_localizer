#include <ndt_localizer/ndt_tools.hpp>

#include <string>
#include <map>
#include <filesystem>
#include <fstream>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// Clamps radians between -pi and pi
double diff_radian(const double rad_1, const double rad_2)
{
    double rad_diff;
    rad_diff = rad_1 - rad_2;
    while (rad_diff >= M_PI)
        rad_diff = rad_diff - 2 * M_PI;
    while (rad_diff < -M_PI)
        rad_diff = rad_diff + 2 * M_PI;
    return rad_diff;
}

// Gets transform from source_frame to target_frame.
// Returns true if successful, false otherwise.
bool get_tf(const tf2_ros::Buffer& tf_buffer, const std::string& target_frame, const std::string& source_frame, geometry_msgs::TransformStamped& transform)
{
    try
    {
        transform = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_DELAYED_THROTTLE(5, "%s", ex.what());
        return false;
    }
}

// Converts method string to enum.
// Terminates the program if an invalid string is given.
PredictionMethod convert_prediction_method(const std::string& prediction_method_str)
{
    PredictionMethod prediction_method;
    for (int i = 0; i < (int)prediction_method_candidates.size(); i++)
    {
        if (prediction_method_str == prediction_method_candidates[i])
        {
            prediction_method = static_cast<PredictionMethod>(i);
            return prediction_method;
        }
    }

    std::stringstream ss;
    ss << "[";
    for (auto it = prediction_method_candidates.begin(); it != prediction_method_candidates.end(); it++)
    {
        if (it != prediction_method_candidates.begin())
            ss << ", ";
        ss << *it;
    }
    ss << "]";
    ROS_ERROR_STREAM("Invalid prediction method: " << prediction_method_str << ". Please select from: " << ss.str());
    ros::shutdown();
    exit(1);
}

// Converts eigen transform matrix to tf.
geometry_msgs::TransformStamped convert_matrix2tf(const Eigen::Matrix4f& matrix)
{
    return tf2::eigenToTransform(Eigen::Affine3f(matrix).cast<double>());
}

// Converts tf to eigen transform matrix.
Eigen::Matrix4f convert_tf2matrix(const geometry_msgs::TransformStamped& transform)
{
    return tf2::transformToEigen(transform).matrix().cast<float>();
}

// Converts eigen transform matrix to pose.
Pose convert_matrix2pose(const Eigen::Matrix4f& matrix)
{
    tf2::Matrix3x3 tf_matrix;
    tf_matrix.setValue(static_cast<float>(matrix(0, 0)), static_cast<float>(matrix(0, 1)), static_cast<float>(matrix(0, 2)),
                       static_cast<float>(matrix(1, 0)), static_cast<float>(matrix(1, 1)), static_cast<float>(matrix(1, 2)),
                       static_cast<float>(matrix(2, 0)), static_cast<float>(matrix(2, 1)), static_cast<float>(matrix(2, 2)));
    Pose pose;
    pose.x = matrix(0, 3);
    pose.y = matrix(1, 3);
    pose.z = matrix(2, 3);
    tf_matrix.getRPY(pose.roll, pose.pitch, pose.yaw, 1);
    return pose;
}

// Converts pose to eigen transform matrix.
Eigen::Matrix4f convert_pose2matrix(const Pose& pose)
{
    Eigen::Translation3f translation(pose.x, pose.y, pose.z);
    Eigen::AngleAxisf rotation_x(pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotation_y(pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotation_z(pose.yaw, Eigen::Vector3f::UnitZ());
    return (translation * rotation_z * rotation_y * rotation_x).matrix();
}

// Get linear prediction of next pose based on twist.
Pose get_linear_prediction_pose(const Pose& current_pose, const geometry_msgs::Twist& twist, const double dt)
{
    if (dt < 0.0)
        return current_pose;

    // Calculate next rotation
    Pose next_pose;
    next_pose.roll = current_pose.roll + twist.angular.x * dt;
    next_pose.pitch = current_pose.pitch + twist.angular.y * dt;
    next_pose.yaw = current_pose.yaw + twist.angular.z * dt;

    // Calculate next translation
    Pose translation_pose, rotation_pose;
    Eigen::Matrix4f translation_matrix, rotation_matrix;
    translation_pose.x = twist.linear.x * dt;
    translation_pose.y = twist.linear.y * dt;
    translation_pose.z = twist.linear.z * dt;
    translation_pose.roll = translation_pose.pitch = translation_pose.yaw = 0.0;
    translation_matrix = convert_pose2matrix(translation_pose);
    rotation_pose.x = rotation_pose.y = rotation_pose.z = 0.0;
    rotation_pose.roll = next_pose.roll;
    rotation_pose.pitch = next_pose.pitch;
    rotation_pose.yaw = next_pose.yaw;
    rotation_matrix = convert_pose2matrix(rotation_pose);
    translation_matrix = rotation_matrix * translation_matrix;
    translation_pose = convert_matrix2pose(translation_matrix);
    next_pose.x = current_pose.x + translation_pose.x;
    next_pose.y = current_pose.y + translation_pose.y;
    next_pose.z = current_pose.z + translation_pose.z;

    return next_pose;
}

// Gets neighbor submap ids.
std::unordered_set<int> get_neighbor_ids(const std::map<int, SubmapWithInfo>& submap_map, const float include_distance)
{
    std::unordered_set<int> neighbor_id_set;
    for (auto submap_with_info : submap_map)
    {
        if (submap_with_info.second.distance <= include_distance)
            neighbor_id_set.insert(submap_with_info.first);
    }
    return neighbor_id_set;
}

// Point cloud range filter.
void apply_range_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan, pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_filtered, const float min_scan_range, const float max_scan_range)
{
    for (auto it = scan->begin(); it != scan->end(); ++it)
    {
        float range_squared;
        range_squared = it->x * it->x + it->y * it->y + it->z * it->z;
        if (range_squared < max_scan_range * max_scan_range
            || range_squared > min_scan_range * min_scan_range)
            scan_filtered->push_back(*it);
    }
}

// Point cloud voxel grid filter.
void apply_voxel_grid_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan, pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_filtered, const float voxel_leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan);
    voxel_grid_filter.filter(*scan_filtered);
}

// Load full map from pcd file.
void load_full_map(const std::string& filepath, pcl::PointCloud<pcl::PointXYZI>& map)
{
    ROS_INFO_STREAM("Loading pcd file: " << filepath << ". Please wait.");
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filepath, map) != 0)
    {
        ROS_ERROR_STREAM("Failed to load pcd file: " << filepath);
        ros::shutdown();
        exit(1);
    }
    else
        ROS_INFO_STREAM("Finished loading " << map.width << " points from " << filepath);
}

// Load submaps from pcd files.
void load_submaps(const std::string& submap_dir, std::map<int, SubmapWithInfo>& submap_map)
{
    ROS_INFO_STREAM("Loading pcd files from " << submap_dir << ". Please wait.");
    std::filesystem::path target_dir(submap_dir), filepath;
    filepath = target_dir / "submap.csv";
    std::ifstream ifs(filepath);

    if (!ifs)
    {
        ROS_ERROR_STREAM("Failed to open submap.csv");
        ros::shutdown();
        exit(1);
    }

    std::string line, element;
    int id;
    SubmapWithInfo submap_with_info;
    submap_with_info.in_group = true;
    submap_with_info.group_id = 0;
    submap_with_info.distance = 0.0;
    submap_with_info.position_matrix = Eigen::Matrix4f::Identity();
    while (ifs && !ifs.eof())
    {
        std::getline(ifs, line);
        if (line.empty())
            continue;
        std::stringstream ss(line);
        std::getline(ss, element, ',');
        id = std::stoi(element);
        submap_map.insert(std::make_pair(id, submap_with_info));
        std::getline(ss, element, ',');
        submap_map[id].position_matrix(0, 3) = std::stof(element);
        std::getline(ss, element, ',');
        submap_map[id].position_matrix(1, 3) = std::stof(element);
        std::getline(ss, element, ',');
        submap_map[id].position_matrix(2, 3) = std::stof(element);

        filepath = target_dir / ("submap" + (std::to_string(id) + ".pcd"));
        pcl::PointCloud<pcl::PointXYZI> submap_points;
        if (pcl::io::loadPCDFile(filepath, submap_points) != 0)
            ROS_WARN_STREAM("Failed to load pcd file: " << filepath);

        submap_map[id].points = submap_points;
    }

    ROS_INFO_STREAM("Finished loading " << submap_map.size() << " submaps from " << submap_dir);
}

// Update submap distances.
void update_submap_distances(const Pose& current_pose, std::map<int, SubmapWithInfo>& submap_map)
{
    for (auto& submap_with_info : submap_map)
    {
        double diff_x, diff_y, diff_z;
        diff_x = current_pose.x - submap_with_info.second.position_matrix(0, 3);
        diff_y = current_pose.y - submap_with_info.second.position_matrix(1, 3);
        diff_z = current_pose.z - submap_with_info.second.position_matrix(2, 3);
        submap_with_info.second.distance = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    }
}
