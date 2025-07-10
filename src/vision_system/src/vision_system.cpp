#include "vision_system.hpp"

VisionSystem::VisionSystem(const std::shared_ptr<SystemNode>& system_node)
    :QWidget(), m_system_node(system_node)
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config.yaml");

    connect(m_system_node.get(), &SystemNode::processed_image_received, this, &VisionSystem::on_processed_image_received);
    connect(m_system_node.get(), &SystemNode::log_received, this, &VisionSystem::on_log_received);
    connect(m_system_node.get(), &SystemNode::result_received, this, &VisionSystem::on_result_received);
    connect(m_log_timer, &QTimer::timeout, this, &VisionSystem::on_log_timer_timeout);

    m_log_timer->start(1000 / m_config["log"]["log_rate"].as<int>());
}

void VisionSystem::run() {
    this->setup_ui();
    this->show();
}

VisionSystem::~VisionSystem() {

}

void VisionSystem::setup_ui()
{
    this->setFixedSize(m_config["ui"]["ui_wh"][0].as<int>(), m_config["ui"]["ui_wh"][1].as<int>());

    m_logger->setMaximumBlockCount(m_config["ui"]["max_log_num"].as<int>());
    m_logger->setReadOnly(true);
    m_logger->setFixedSize(m_config["ui"]["logger_wh"][0].as<int>(), m_config["ui"]["logger_wh"][1].as<int>());
    m_layout->addWidget(m_logger, m_config["layout"]["logger_rc"][0].as<int>(), m_config["layout"]["logger_rc"][1].as<int>());
    m_logger->appendPlainText("logger");

    m_color_video->setFixedSize(m_config["ui"]["color_video_wh"][0].as<int>(), m_config["ui"]["color_video_wh"][1].as<int>());
    m_layout->addWidget(m_color_video, m_config["layout"]["color_video_rc"][0].as<int>(), m_config["layout"]["color_video_rc"][1].as<int>());

    m_status_list->setFixedSize(m_config["ui"]["status_list_wh"][0].as<int>(), m_config["ui"]["status_list_wh"][1].as<int>());
    for (const auto& [status_text, status] : m_status) {
        m_status_list->addItem(status);
    }
    m_layout->addWidget(m_status_list, m_config["layout"]["status_list_rc"][0].as<int>(), m_config["layout"]["status_list_rc"][1].as<int>());

    m_result_logger->setFixedSize(m_config["ui"]["result_logger_wh"][0].as<int>(), m_config["ui"]["result_logger_wh"][1].as<int>());
    m_layout->addWidget(m_result_logger, m_config["layout"]["result_logger_rc"][0].as<int>(), m_config["layout"]["result_logger_rc"][1].as<int>());
    m_result_logger->appendPlainText("result_logger");

    for (const auto& [button_text, button] : m_buttons) {
        button->setFixedSize(m_config["ui"]["button_wh"][0].as<int>(), m_config["ui"]["button_wh"][1].as<int>());
        m_layout->addWidget(button);
    }
}

void VisionSystem::add_log(const ros_msg::msg::Log& log)
{
    LogLevel log_level = static_cast<LogLevel>(log.level);
    switch (log_level) {
    case LogLevel::DEBUG: {
        break;
    }
    case LogLevel::INFO: {
        break;
    }
    case LogLevel::WARNING: {
        break;
    }
    case LogLevel::ERROR: {
        break;
    }
    }
    int64_t seconds = rclcpp::Time(log.header.stamp).nanoseconds() / 1e9;
    QTime qtime = QTime(0, 0).addSecs(seconds);
    QString log_text = QString("[%1][%2]%3")
        .arg(qtime.toString())
        .arg(QString::fromStdString(m_log_level2str.at(log_level)))
        .arg(QString::fromStdString(log.text));

    std::lock_guard<std::mutex> lock(m_log_buffer_mtx);
    m_log_buffer.push_back(log_text);
}

void VisionSystem::on_processed_image_received(const sensor_msgs::msg::Image::ConstSharedPtr& processed_image) {
    try {
        cv_bridge::CvImagePtr cv_img_ptr = cv_bridge::toCvCopy(processed_image, "rgb8");
        QImage q_img(cv_img_ptr->image.data, cv_img_ptr->image.cols, cv_img_ptr->image.rows, cv_img_ptr->image.step, QImage::Format::Format_RGB888);
        QMetaObject::invokeMethod(
            this,
            [this, moved_img = std::move(q_img)] {m_color_video->setPixmap(QPixmap::fromImage(moved_img));},
            Qt::QueuedConnection
        );
    }
    catch (cv_bridge::Exception& e) {
        this->add_log(create_log(m_system_node->now(), LogLevel::ERROR, "failed to receive processed image!"));
    }
}

void VisionSystem::on_log_received(const ros_msg::msg::Log::ConstSharedPtr& log)
{
    this->add_log(*log);
}

void VisionSystem::on_result_received(const std_msgs::msg::String::ConstSharedPtr& result)
{
}

void VisionSystem::on_log_timer_timeout()
{
    std::lock_guard<std::mutex> lock(m_log_buffer_mtx);
    if (!m_log_buffer.empty()) {
        for (const auto& text : m_log_buffer) {
            m_logger->appendPlainText(text);
        }
        m_log_buffer.clear();
    }
}