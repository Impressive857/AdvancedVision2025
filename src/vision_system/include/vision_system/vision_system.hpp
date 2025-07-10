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
    void setup_ui();
    void add_log(const ros_msg::msg::Log& log);
private:
    const std::unordered_map<LogLevel, std::string> m_log_level2str{
        {LogLevel::DEBUG, "DEBUG"},
        {LogLevel::INFO, "INFO"},
        {LogLevel::WARNING, "WARNING"},
        {LogLevel::ERROR, "ERROR"},
    };
    QGridLayout* m_layout = new QGridLayout(this);
    const std::unordered_map<std::string, QPushButton*> m_buttons{
        {"identify_round_one", new QPushButton("identify_round_one", this)},
        {"identify_round_two", new QPushButton("identify_round_two", this)},
        {"measure_round_one", new QPushButton("measure_round_one", this)},
        {"measure_round_two", new QPushButton("measure_round_two", this)},
        {"stop", new QPushButton("stop", this)},
        {"exit", new QPushButton("exit", this)},
    };
    QListWidget* m_status_list = new QListWidget(this);
    const std::unordered_map<std::string, QListWidgetItem*> m_status{
        {"mode_status", new QListWidgetItem("unknow")},
        {"running_status", new QListWidgetItem("waiting")},
        {"network_status",  new QListWidgetItem("unconnected")}
    };
    QPlainTextEdit* m_logger = new QPlainTextEdit(this);
    QPlainTextEdit* m_result_logger = new QPlainTextEdit(this);
    std::vector<QString> m_log_buffer;
    std::mutex m_log_buffer_mtx;
    QTimer* m_log_timer = new QTimer(this);
    QLabel* m_color_video = new QLabel(this);
    std::atomic<bool> m_system_is_running = true;
    YAML::Node m_config;
    std::shared_ptr<SystemNode> m_system_node;
private slots:
    void on_processed_image_received(const sensor_msgs::msg::Image::ConstSharedPtr& processed_image);
    void on_log_received(const ros_msg::msg::Log::ConstSharedPtr& log);
    void on_result_received(const std_msgs::msg::String::ConstSharedPtr& result);
    void on_log_timer_timeout();
};

#endif // !_VISION_SYSTEM_HPP_