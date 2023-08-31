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
    NDTLogger();
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

    void queue_callback(const sensor_msgs::PointCloud2::ConstPtr&);
    void deque_callback(const std_msgs::Empty::ConstPtr&);
};

#endif // NDT_LOGGER_HPP_
