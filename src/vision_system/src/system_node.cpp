#include "system_node.hpp"

SystemNode::SystemNode()
    :rclcpp::Node("system_node")
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config.yaml");

    qRegisterMetaType<sensor_msgs::msg::Image>("sensor_msgs::msg::Image");
    qRegisterMetaType<ros_msg::msg::Log>("ros_msg::msg::Log");
    qRegisterMetaType<std_msgs::msg::String>("std_msgs::msg::String");


    m_processed_iamge_subscriber = image_transport::create_subscription(
        this,
        m_config["topic"]["processed_image"].as<std::string>(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) {Q_EMIT processed_image_received(image);},
        "raw",
        rmw_qos_profile_sensor_data
    );
    m_log_subscription = this->create_subscription<ros_msg::msg::Log>(
        m_config["topic"]["log_text"].as<std::string>(),
        1,
        [this](const ros_msg::msg::Log::ConstSharedPtr& log) {Q_EMIT log_received(log);}
    );
    m_result_subscription = this->create_subscription<std_msgs::msg::String>(
        m_config["topic"]["result_text"].as<std::string>(),
        1,
        [this](const std_msgs::msg::String::ConstSharedPtr& result) {Q_EMIT result_received(result);}
    );
}

SystemNode::~SystemNode() {

}