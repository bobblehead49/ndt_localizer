#include <ndt_localizer/ndt_logger.hpp>

#include <string>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>


NDTLogger::NDTLogger(): nh_(""), pnh_("~")
{
    // Get parameters
    pnh_.param<std::string>("queue_topic", queue_topic_, "queue");
    pnh_.param<std::string>("deque_topic", deque_topic_, "deque");

    // Initialize variables
    queue_count_ = 0;
    deque_count_ = 0;

    // Setup subscribers
    queue_sub_ = nh_.subscribe(queue_topic_, 10000, &NDTLogger::queue_callback, this);
    deque_sub_ = nh_.subscribe(deque_topic_, 10000, &NDTLogger::deque_callback, this);
}

NDTLogger::~NDTLogger(){}

void NDTLogger::queue_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    queue_count_++;
}

void NDTLogger::deque_callback(const std_msgs::Empty::ConstPtr& msg)
{
    deque_count_++;

    // Calculate ETA
    process_time_queue_.push(ros::Time::now());
    unsigned int eta = 0, hour = 0, minute = 0, second = 0;
    if (process_time_queue_.size() > 100)
    {
        eta = (unsigned int)((process_time_queue_.back() - process_time_queue_.front()).toSec() / process_time_queue_.size() * (queue_count_ - deque_count_));
        hour = eta / 3600;
        minute = (eta % 3600) / 60;
        second = (eta % 3600) % 60;
        process_time_queue_.pop();
    }

    // Print log
    char eta_buff[256];
    sprintf(eta_buff, "%02d:%02d:%02d", hour, minute, second);
    ROS_INFO_STREAM_THROTTLE(1.0, "ETA: " << eta_buff << " (" << deque_count_ << "/" << queue_count_ << ")");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_logger");
    NDTLogger ndt_logger;
    ros::spin();

    return 0;
}
