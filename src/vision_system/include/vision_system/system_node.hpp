#ifndef _SYSTEM_NODE_HPP_
#define _SYSTEM_NODE_HPP_

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <ros_msg/msg/log.hpp>
#include <std_msgs/msg/string.hpp>

// yaml
#include <yaml-cpp/yaml.h>

// qt
#include <QObject>

//
#include "vision_utils.hpp"

class SystemNode
    :public QObject, public rclcpp::Node
{
    Q_OBJECT
public:
    SystemNode();
    ~SystemNode();

private:
    rclcpp::TimerBase::SharedPtr m_log_timer;
    rclcpp::Subscription<ros_msg::msg::Log>::SharedPtr m_log_subscription;
    image_transport::Subscriber m_processed_iamge_subscriber;
    YAML::Node m_config = YAML::LoadFile("../../config.yaml");
Q_SIGNALS:
    void processed_image_received(const sensor_msgs::msg::Image::ConstSharedPtr& image);
Q_SIGNALS:
    void log_received(const ros_msg::msg::Log::ConstSharedPtr& log);
};

#endif // !_SYSTEM_NODE_HPP_