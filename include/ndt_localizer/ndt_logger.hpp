// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#ifndef NDT_LOGGER_HPP_
#define NDT_LOGGER_HPP_

#include <string>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>


class NDTLogger
{
public:
    /**
     * @brief Constructor
     */
    NDTLogger();

    /**
     * @brief Destructor
     */
    ~NDTLogger();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber queue_sub_;
    ros::Subscriber deque_sub_;

    std::string queue_topic_;
    std::string deque_topic_;

    unsigned int queue_count_;
    unsigned int deque_count_;

    std::queue<ros::Time> process_time_queue_;

    /**
     * @brief Executed when a point cloud topic is received. This method is used to calculate ETA.
     * @param msg The received message.
     */
    void queue_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    /**
     * @brief Executed when a empty topic is received. This method is used to calculate ETA.
     * @param msg The received message.
     */
    void deque_callback(const std_msgs::Empty::ConstPtr& msg);
};

#endif // NDT_LOGGER_HPP_
