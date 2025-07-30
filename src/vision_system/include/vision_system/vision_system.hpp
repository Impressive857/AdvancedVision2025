#ifndef _VISION_SYSTEM_HPP_
#define _VISION_SYSTEM_HPP_

// std
#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <chrono>

// qt
#include <QWidget>
#include <QPushButton>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QGridLayout>
#include <QImage>
#include <QLabel>
#include <QTime>
#include <QTimer>
#include <QTcpSocket>
#include <QDataStream>

// cv_bridge
#include <cv_bridge/cv_bridge.h>

//
#include "system_node.hpp"

class VisionSystem
    :public QWidget
{
    Q_OBJECT
public:
    VisionSystem() = delete;
    VisionSystem(const std::shared_ptr<SystemNode>& system_node);
    void run();
    ~VisionSystem();
private:
    using color_image_t = sensor_msgs::msg::Image;
    using depth_image_t = sensor_msgs::msg::Image;
    using point_cloud_t = sensor_msgs::msg::PointCloud2;
    void setup_ui();
    void init_network();
    void add_log(const ros_msgs::msg::Log& log);
    void set_video_image(const cv::Mat& image, QLabel* video);
private:
    const std::unordered_map<vision_utils::LogLevel, std::string> m_log_level2str{
        {vision_utils::LogLevel::DEBUG, "DEBUG"},
        {vision_utils::LogLevel::INFO, "INFO"},
        {vision_utils::LogLevel::WARNING, "WARNING"},
        {vision_utils::LogLevel::ERROR, "ERROR"},
    };
    QGridLayout* m_vision_system_layout = new QGridLayout(this);
    const std::unordered_map<std::string, QPushButton*> m_buttons{
        {"reconnect", new QPushButton()},
        {"stop", new QPushButton()}
    };
    const std::unordered_map<std::string, QListWidgetItem*> m_status{
        {"mode", new QListWidgetItem("mode : unknow")},
        {"running", new QListWidgetItem("running : waiting")},
        {"network",  new QListWidgetItem("network : unconnected")}
    };
    const std::unordered_map<std::string, QWidget*> m_widgets{
        {"processed_image_video", new QWidget()},
        {"color_depth_image_video", new QWidget()},
        {"status_list", new QWidget()},
        {"logger", new QWidget()},
        {"control", new QWidget()}
    };
    const std::unordered_map<std::string, QLayout*> m_layouts{
        {"processed_image_video", new QVBoxLayout()},
        {"color_depth_image_video", new QVBoxLayout()},
        {"status_list", new QVBoxLayout()},
        {"logger", new QVBoxLayout()},
        {"control", new QVBoxLayout()}
    };
    const std::unordered_map<std::string, QWidget*> m_objects{
        {"processed_image_video", new QLabel()},
        {"color_depth_image_video", new QLabel()},
        {"status_list", new QListWidget()},
        {"logger", new QPlainTextEdit()}
    };
    const std::unordered_map<std::string, QLabel*> m_labels{
        {"processed_image_video", new QLabel()},
        {"color_depth_image_video", new QLabel()},
        {"status_list", new QLabel()},
        {"logger", new QLabel()},
        {"control", new QLabel()}
    };
    std::vector<QString> m_log_buffer;
    std::mutex m_log_buffer_mtx;
    QTimer* m_log_timer = new QTimer(this);
    QTcpSocket* m_socket = new QTcpSocket(this);
    bool m_network_connected = false;
    std::atomic<bool> m_system_is_running = true;
    std::string m_current_mode = "unknow";
    YAML::Node m_config;
    std::shared_ptr<SystemNode> m_system_node;
private slots:
    void on_processed_image_received(const sensor_msgs::msg::Image::ConstSharedPtr& processed_image);
    void on_color_depth_image_received(const sensor_msgs::msg::Image::ConstSharedPtr& color_depth_image);
    void on_log_text_received(const ros_msgs::msg::Log::ConstSharedPtr& log);
    void on_result_text_received(const std_msgs::msg::String::ConstSharedPtr& result);
    void on_mode_text_received(const std_msgs::msg::String::ConstSharedPtr& mode);
    void on_log_timer_timeout();
};

#endif // !_VISION_SYSTEM_HPP_