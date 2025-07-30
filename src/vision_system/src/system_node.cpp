#include "system_node.hpp"

SystemNode::SystemNode()
    :rclcpp::Node("system_node")
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");

    qRegisterMetaType<ros_msgs::msg::Log::ConstSharedPtr>("ros_msgs::msg::Log::ConstSharedPtr");
    qRegisterMetaType<std_msgs::msg::String::ConstSharedPtr>("std_msgs::msg::String::ConstSharedPtr");
    qRegisterMetaType<sensor_msgs::msg::Image::ConstSharedPtr>("sensor_msgs::msg::Image::ConstSharedPtr");


    m_processed_image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        m_config["topic"]["processed_image"].as<std::string>(),
        10,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) {Q_EMIT processed_image_received(image);}
    );

    m_color_depth_image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        m_config["topic"]["color_depth_image"].as<std::string>(),
        10,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) {Q_EMIT color_depth_image_received(image);}
    );

    m_log_text_subscription = this->create_subscription<ros_msgs::msg::Log>(
        m_config["topic"]["log_text"].as<std::string>(),
        1,
        [this](const ros_msgs::msg::Log::ConstSharedPtr& log) {Q_EMIT log_text_received(log);}
    );

    m_result_text_subscription = this->create_subscription<std_msgs::msg::String>(
        m_config["topic"]["result_text"].as<std::string>(),
        1,
        [this](const std_msgs::msg::String::ConstSharedPtr& result) {Q_EMIT result_text_received(result);}
    );

    m_mode_text_subscription = this->create_subscription<std_msgs::msg::String>(
        m_config["topic"]["mode_text"].as<std::string>(),
        1,
        [this](const std_msgs::msg::String::ConstSharedPtr& mode) {Q_EMIT mode_text_received(mode);}
    );

    m_camera_data_publisher = this->create_publisher<ros_msgs::msg::CameraData>(
        m_config["topic"]["camera_data"].as<std::string>(),
        10
    );

    m_color_image_subscriber.subscribe(
        this,
        m_config["topic"]["color_img_raw"].as<std::string>()
    );

    m_depth_image_subscriber.subscribe(
        this,
        m_config["topic"]["depth_img_raw"].as<std::string>()
    );

    m_point_cloud_subscriber.subscribe(
        this,
        m_config["topic"]["point_cloud"].as<std::string>()
    );


    m_sync = std::make_shared<message_filters::Synchronizer<sync_policy>>(sync_policy(10), m_color_image_subscriber, m_depth_image_subscriber, m_point_cloud_subscriber);
    m_sync->registerCallback(
        std::bind(&SystemNode::camera_data_received_cbfn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );
}

void SystemNode::camera_data_received_cbfn(const color_image_t::ConstSharedPtr& color_image, const depth_image_t::ConstSharedPtr& depth_image, const point_cloud_t::ConstSharedPtr& point_cloud) {
    ros_msgs::msg::CameraData camera_data;
    camera_data.color_image = *color_image;
    camera_data.depth_image = *depth_image;
    camera_data.point_cloud = *point_cloud;
    m_camera_data_publisher->publish(std::move(camera_data));
}

SystemNode::~SystemNode() {

}