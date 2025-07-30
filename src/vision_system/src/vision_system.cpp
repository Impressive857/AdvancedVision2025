#include "vision_system.hpp"

VisionSystem::VisionSystem(const std::shared_ptr<SystemNode>& system_node)
    :QWidget(), m_system_node(system_node)
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");

    connect(m_system_node.get(), &SystemNode::processed_image_received, this, &VisionSystem::on_processed_image_received);
    connect(m_system_node.get(), &SystemNode::color_depth_image_received, this, &VisionSystem::on_color_depth_image_received);
    connect(m_system_node.get(), &SystemNode::log_text_received, this, &VisionSystem::on_log_text_received);
    connect(m_system_node.get(), &SystemNode::result_text_received, this, &VisionSystem::on_result_text_received);
    connect(m_system_node.get(), &SystemNode::mode_text_received, this, &VisionSystem::on_mode_text_received);
    connect(m_log_timer, &QTimer::timeout, this, &VisionSystem::on_log_timer_timeout);
    connect(m_buttons.at("reconnect"), &QPushButton::clicked, this, &VisionSystem::init_network);

    m_log_timer->start(1000 / m_config["log"]["log_rate"].as<int>());
}

void VisionSystem::run() {
    this->setup_ui();
    this->init_network();
    this->show();

    // show之后才能set_video_image
    cv::Mat no_image = cv::imread(ament_index_cpp::get_package_share_directory("vision_system") + "/image/no_image.jpg");
    cv::cvtColor(no_image, no_image, cv::COLOR_BGR2RGB);
    QLabel* processed_image_video = dynamic_cast<QLabel*>(m_objects.at("processed_image_video"));
    QLabel* color_depth_image_video = dynamic_cast<QLabel*>(m_objects.at("color_depth_image_video"));
    set_video_image(no_image, processed_image_video);
    set_video_image(no_image, color_depth_image_video);
}

VisionSystem::~VisionSystem() {
    m_socket->close();
}

void VisionSystem::setup_ui()
{
    this->setFixedSize(m_config["ui"]["ui_wh"][0].as<int>(), m_config["ui"]["ui_wh"][1].as<int>());

    for (auto& [text, layout] : m_layouts) {
        auto label = m_labels.at(text);
        label->setText(QString::fromStdString(text));
        if ("control" != text) {
            auto object = m_objects.at(text);
            layout->addWidget(label);
            object->setFixedSize(m_config["ui"][text + "_wh"][0].as<int>(), m_config["ui"][text + "_wh"][1].as<int>());
            layout->addWidget(object);
        }
        else {
            layout->addWidget(label);
            for (auto& [button_text, button] : m_buttons) {
                button->setText(QString::fromStdString(button_text));
                button->setFixedSize(m_config["ui"]["button_wh"][0].as<int>(), m_config["ui"]["button_wh"][1].as<int>());
                layout->addWidget(button);
            }
        }
    }

    QPlainTextEdit* logger = dynamic_cast<QPlainTextEdit*>(m_objects.at("logger"));
    logger->setMaximumBlockCount(m_config["ui"]["max_log_num"].as<int>());
    logger->setReadOnly(true);

    QListWidget* status_list = dynamic_cast<QListWidget*>(m_objects.at("status_list"));
    for (const auto& [status_text, status] : m_status) {
        status->setForeground(Qt::red);
        status_list->addItem(status);
    }

    for (auto& [text, widget] : m_widgets) {
        widget->setLayout(m_layouts.at(text));
        m_vision_system_layout->addWidget(widget, m_config["layout"][text + "_rc"][0].as<int>(), m_config["layout"][text + "_rc"][1].as<int>());
    }
}

void VisionSystem::init_network()
{
    m_socket->connectToHost(QString::fromStdString(m_config["network"]["host_ip"].as<std::string>()), m_config["network"]["port"].as<quint16>());
    if (!m_socket->waitForConnected(m_config["network"]["timeout"].as<int>())) {
        this->add_log(vision_utils::create_log(m_system_node->now(), vision_utils::LogLevel::ERROR, "failed to connect network!"));
        return;
    }
    m_network_connected = true;
    m_status.at("network")->setText("network : connected");
    m_status.at("network")->setForeground(Qt::green);
    QDataStream stream(m_socket);
    stream.setByteOrder(QDataStream::BigEndian);
    stream << m_config["network"]["data_type"]["start"].as<int32_t>() << static_cast<int32_t>(0);
}

void VisionSystem::add_log(const ros_msgs::msg::Log& log)
{
    vision_utils::LogLevel log_level = static_cast<vision_utils::LogLevel>(log.level);
    int64_t seconds = rclcpp::Time(log.header.stamp).nanoseconds() / 1e9 + 8 * 3600; // UTC时差8小时
    QTime qtime = QTime(0, 0).addSecs(seconds);
    QString log_text = QString("[%1][%2]%3")
        .arg(qtime.toString())
        .arg(QString::fromStdString(m_log_level2str.at(log_level)))
        .arg(QString::fromStdString(log.text));

    std::lock_guard<std::mutex> lock(m_log_buffer_mtx);
    m_log_buffer.push_back(log_text);
}

void VisionSystem::set_video_image(const cv::Mat& image, QLabel* video)
{
    cv::Mat temp = image.clone();
    QImage q_img(temp.data, temp.cols, temp.rows, temp.step, QImage::Format::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(std::move(q_img)).scaled(video->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    QMetaObject::invokeMethod(
        this,
        [pixmap, video] {video->setPixmap(pixmap);},
        Qt::QueuedConnection
    );
}

void VisionSystem::on_processed_image_received(const sensor_msgs::msg::Image::ConstSharedPtr& processed_image) {
    try {
        cv_bridge::CvImagePtr cv_img_ptr = cv_bridge::toCvCopy(processed_image, "rgb8");
        QLabel* processed_image_video = dynamic_cast<QLabel*>(m_objects.at("processed_image_video"));
        set_video_image(cv_img_ptr->image, processed_image_video);
    }
    catch (cv_bridge::Exception& e) {
        this->add_log(vision_utils::create_log(m_system_node->now(), vision_utils::LogLevel::ERROR, "failed to receive processed image!"));
    }
}

void VisionSystem::on_color_depth_image_received(const sensor_msgs::msg::Image::ConstSharedPtr& color_depth_image)
{
    try {
        cv_bridge::CvImagePtr cv_img_ptr = cv_bridge::toCvCopy(color_depth_image, "rgb8");
        QLabel* color_depth_image_video = dynamic_cast<QLabel*>(m_objects.at("color_depth_image_video"));
        set_video_image(cv_img_ptr->image, color_depth_image_video);
    }
    catch (cv_bridge::Exception& e) {
        this->add_log(vision_utils::create_log(m_system_node->now(), vision_utils::LogLevel::ERROR, "failed to receive color depth image!"));
    }
}

void VisionSystem::on_log_text_received(const ros_msgs::msg::Log::ConstSharedPtr& log)
{
    this->add_log(*log);
}

void VisionSystem::on_result_text_received(const std_msgs::msg::String::ConstSharedPtr& result)
{
    if ("identify" != m_current_mode || "measure" != m_current_mode) {
        this->add_log(vision_utils::create_log(m_system_node->now(), vision_utils::LogLevel::ERROR, "mode is unknow but result received!"));
        return;
    }
    qint32 data_type = m_config["network"]["data_type"][m_current_mode].as<qint32>();
    std::string data = "START\n" + result->data + "END";
    qint32 data_length = data.size() + 1;

    QByteArray buffer;
    QDataStream stream(&buffer, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);

    stream << data_type << data_length;
    stream.writeRawData(data.c_str(), data_length);

    qint64 bytesWritten = m_socket->write(buffer);
    if (bytesWritten == -1) {
        this->add_log(vision_utils::create_log(m_system_node->now(), vision_utils::LogLevel::ERROR, "failed to send result %s!", m_socket->errorString().toStdString().c_str()));
        return;
    }
    else if (bytesWritten != buffer.size()) {
        this->add_log(vision_utils::create_log(m_system_node->now(), vision_utils::LogLevel::ERROR, "result not fully send!"));
    }
    else {
        this->add_log(vision_utils::create_log(m_system_node->now(), vision_utils::LogLevel::INFO, "result send successfully!"));
    }

    m_socket->waitForBytesWritten();
}

void VisionSystem::on_mode_text_received(const std_msgs::msg::String::ConstSharedPtr& mode)
{
    m_current_mode = mode->data;
    m_status.at("mode")->setText(QString::fromStdString("mode : " + m_current_mode));
    m_status.at("mode")->setForeground(Qt::green);
    this->add_log(vision_utils::create_log(m_system_node->now(), vision_utils::LogLevel::INFO, "mode text received!"));
}

void VisionSystem::on_log_timer_timeout()
{
    std::lock_guard<std::mutex> lock(m_log_buffer_mtx);
    if (!m_log_buffer.empty()) {
        QPlainTextEdit* logger = dynamic_cast<QPlainTextEdit*>(m_objects.at("logger"));
        for (const auto& text : m_log_buffer) {
            logger->appendPlainText(text);
        }
        m_log_buffer.clear();
    }
}