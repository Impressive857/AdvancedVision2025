#ifndef _IDENTIFY_HPP_
#define _IDENTIFY_HPP_

#include "fastsam.hpp"
#include "segformer.hpp"

// std
#include <unordered_map>
#include <vector>
#include <atomic>

// ros
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "ros_msgs/msg/camera_data.hpp"
#include "ros_msgs/msg/log.hpp"

// yaml
#include <yaml-cpp/yaml.h>

//cv_bridge
#include <cv_bridge/cv_bridge.h>

// acl
#include "acl_utils.hpp"

class Identify
    :public rclcpp::Node
{
public:
    Identify();
    void run();
    ~Identify();
private:
    using detection_t = std::unordered_map<int, int>;
    using detections_t = std::vector<detection_t>;
    void init();
    void finalize();
    void camera_data_received_cbfn(const ros_msgs::msg::CameraData::ConstSharedPtr& camera_data);
private:
    FastSAM m_fastsam;
    Segformer m_segformer;
    rclcpp::Subscription<ros_msgs::msg::CameraData>::SharedPtr m_camera_data_subscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_result_text_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_mode_text_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_processed_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_color_depth_image_publisher;
    rclcpp::Publisher<ros_msgs::msg::Log>::SharedPtr m_logger;
    float m_min_distance;
    float m_max_distance;
    std::atomic<bool> m_ready;
    detections_t m_detections;
    YAML::Node m_config;
};

#endif // !_IDENTIFY_HPP_