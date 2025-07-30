#ifndef _SYSTEM_NODE_HPP_
#define _SYSTEM_NODE_HPP_

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros_msgs/msg/log.hpp"
#include "ros_msgs/msg/camera_data.hpp"

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
    using color_image_t = sensor_msgs::msg::Image;
    using depth_image_t = sensor_msgs::msg::Image;
    using point_cloud_t = sensor_msgs::msg::PointCloud2;
    void camera_data_received_cbfn(const color_image_t::ConstSharedPtr& color_image, const depth_image_t::ConstSharedPtr& depth_image, const point_cloud_t::ConstSharedPtr& point_cloud);
private:
    using sync_policy = message_filters::sync_policies::ApproximateTime<color_image_t, depth_image_t, point_cloud_t>;
    rclcpp::TimerBase::SharedPtr m_log_timer;
    rclcpp::Subscription<ros_msgs::msg::Log>::SharedPtr m_log_text_subscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_result_text_subscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_mode_text_subscription;
    rclcpp::Publisher<ros_msgs::msg::CameraData>::SharedPtr m_camera_data_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_processed_image_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_color_depth_image_subscription;
    message_filters::Subscriber<color_image_t> m_color_image_subscriber;
    message_filters::Subscriber<depth_image_t> m_depth_image_subscriber;
    message_filters::Subscriber<point_cloud_t> m_point_cloud_subscriber;
    std::shared_ptr<message_filters::Synchronizer<sync_policy>> m_sync;
    YAML::Node m_config;
Q_SIGNALS:
    void processed_image_received(const sensor_msgs::msg::Image::ConstSharedPtr& image);
Q_SIGNALS:
    void color_depth_image_received(const sensor_msgs::msg::Image::ConstSharedPtr& color_detph_image);
Q_SIGNALS:
    void log_text_received(const ros_msgs::msg::Log::ConstSharedPtr& log);
Q_SIGNALS:
    void result_text_received(const std_msgs::msg::String::ConstSharedPtr& result);
Q_SIGNALS:
    void mode_text_received(const std_msgs::msg::String::ConstSharedPtr& mode);
};

#endif // !_SYSTEM_NODE_HPP_