# ndt_localizer

## Overview
This package is for **3D mapping and localization using NDT** (normal-distributions transform).
Includes a cheap edition of **loop closure** without any total optimization.  

This package has been developed and tested under **ROS noetic** on **Ubuntu 20.04**.  

### Demo video of 3D mapping and loop closure 

https://github.com/bobblehead49/ndt_localizer/assets/85610470/6e9ebc63-43fb-465c-b07b-4efb202852db

---

## Usage

### Mapping
1. Launch mapping nodes.
    ```bash
    roslaunch ndt_localizer ndt_mapper.launch
    ```

1. Save maps.
    ```bash
    roslaunch ndt_localizer map_saver.launch
    ```

- launch arguments for `ndt_mapper.launch`
    |arg name                           |description                                                                                                |
    |:-                                 |:-                                                                                                         |
    |map_frame                          |frame name of map                                                                                          |
    |base_frame                         |frame name of robot's origin for odometry                                                                  |
    |lidar_frame                        |frame name of lidar                                                                                        |
    |points_topic                       |topic name to subscribe lidar points                                                                       |
    |odom_topic                         |topic name to subscribe odometry                                                                           |
    |map_topic                          |topic name to publish created map                                                                          |
    |map_publish_interval               |minimal interval to publish the full map [sec]                                                             |
    |min_scan_range                     |minimal scan range [m]                                                                                     |
    |max_scan_range                     |maximum scan range [m]                                                                                     |
    |voxel_leaf_size                    |voxel leaf size for scan [m]                                                                               |
    |ndt_max_iterations                 |max iteration counts for optimization [-]                                                                  |
    |ndt_resolution                     |voxel leaf size for target map [m]                                                                         |
    |ndt_step_size                      |maximum step length of newton line search [m]                                                              |
    |ndt_transformation_epsilon         |maximum translation squared difference for the optimization to be considered as having converged [m^2]     |
    |prediction_method                  |method to predict pose for next scan; zero, linear or odom                                                 |
    |translation_error_tolerance        |maximum limit of translation error between the predicted pose and the NDT optimized pose [m]               |
    |rotation_error_tolerance           |maximum limit of rotation error between the predicted pose and the NDT optimized pose [deg]                |
    |map_add_shift                      |minimal shift length to add current scan to the map [m]                                                    |
    |submap_scan_size                   |number of scans to include in a single submap [-]                                                          |
    |submap_include_distance            |submaps within this distance will be added to the target map for localization [m]                          |
    |submap_connect_distance            |submaps within this distance will be connected in the pose graph [m]                                       |
    |use_loop_closure                   |true to apply loop closure                                                                                 |
    |loop_connect_distance              |maximum distance from the root of a loop to attempt loop closure [m]                                       |
    |loop_closure_confirmation_error    |maximum correction error for the loop closure to be considered as having converged [m]                     |
    |save_uncompressed_map              |true to save the full map to a pcd file with no binary compression                                         |
    |save_submaps                       |true to save each submap as a single pcd file                                                              |
    |use_rviz                           |true to launch rviz                                                                                        |

- launch arguments for `map_saver.launch`
    |arg name                           |description                                                                                                |
    |:-                                 |:-                                                                                                         |
    |maps_directory                     |path of directory to save the maps                                                                         |
    |map_name                           |name of the map to be saved                                                                                |

---

### Localization
1. Launch localization nodes.
    ```bash
    roslaunch ndt_localizer ndt_localizer.launch
    ```

2. Set initial pose with `2D Pose Estimate` in rviz.

- launch arguments for `ndt_localizer.launch`
    |arg name                           |description                                                                                                |
    |:-                                 |:-                                                                                                         |
    |maps_directory                     |path of directory to find map files                                                                        |
    |map_name                           |name of the map to be used                                                                                 |
    |map_frame                          |frame name of map                                                                                          |
    |base_frame                         |frame name of robot's origin for odometry                                                                  |
    |lidar_frame                        |frame name of lidar                                                                                        |
    |points_topic                       |topic name to subscribe lidar points                                                                       |
    |odom_topic                         |topic name to subscribe odometry                                                                           |
    |min_scan_range                     |minimal scan range [m]                                                                                     |
    |max_scan_range                     |maximum scan range [m]                                                                                     |
    |voxel_leaf_size                    |voxel leaf size for scan [m]                                                                               |
    |ndt_max_iterations                 |max iteration counts for optimization [-]                                                                  |
    |ndt_resolution                     |voxel leaf size for target map [m]                                                                         |
    |ndt_step_size                      |maximum step length of newton line search [m]                                                              |
    |ndt_transformation_epsilon         |maximum translation squared difference for the optimization to be considered as having converged [m^2]     |
    |prediction_method                  |method to predict pose for next scan; zero, linear or odom                                                 |
    |translation_error_tolerance        |maximum limit of translation error between the predicted pose and the NDT optimized pose [m]               |
    |rotation_error_tolerance           |maximum limit of rotation error between the predicted pose and the NDT optimized pose [deg]                |
    |use_submaps                        |true to use submaps for localization                                                                       |
    |submap_include_distance            |submaps within this distance will be added to the target map for localization [m]                          |
    |submap_update_shift                |minimal shift length for the target map to be updated [m]                                                  |
    |use_rviz                           |true to launch rviz                                                                                        |
