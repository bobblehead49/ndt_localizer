// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <iostream>
#include <string>
#include <filesystem>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace fs = std::filesystem;


int main(int argc, char** argv)
{
    // Initialize node
    ros::init(argc, argv, "map_saver");

    // Create node handle
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Get parameters
    std::string maps_directory, map_name;
    pnh.param<std::string>("maps_directory", maps_directory, "./");
    pnh.param<std::string>("map_name", map_name, "map");

    // Create publisher and subscriber
    ros::Publisher map_save_request_pub;
    map_save_request_pub = nh.advertise<std_msgs::String>("/ndt_mapper/map_save_request", 1);

    // Wait for initialization of the publisher
    ros::Duration(0.5).sleep();

    // Prepare directory and filepath
    fs::path save_dir = fs::path(maps_directory);
    std::time_t t = std::time(nullptr);
    char buff[256];
    std::strftime(buff, sizeof(buff), "%Y%m%d%H%M%S", std::localtime(&t));
    std::string time_str(buff);
    save_dir.append(map_name + "_" + time_str);

    // Check if directory already exists
    if (fs::exists(save_dir))
    {
        std::string input;
        std::cout << save_dir << " already exists. Do you want to overwrite it? [y/n] " << std::flush;
        std::cin >> input;
        if (input != "y" && input != "yes")
        {
            std::cout << "Aborted." << std::endl;
            return 0;
        }
    }

    // Create directory
    fs::create_directory(save_dir);
    fs::create_directory(save_dir / "submaps");

    // Publish map save request
    std_msgs::String msg;
    msg.data = save_dir.string();
    map_save_request_pub.publish(msg);

    // Shutdown node
    ros::shutdown();

    return 0;
}
