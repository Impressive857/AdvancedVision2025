#include "identify.hpp"

aclrtStream acl_utils::model_stream = nullptr;
aclrtContext acl_utils::model_context = nullptr;

Identify::Identify()
    :rclcpp::Node("identify_node")
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");

    m_camera_data_subscription = this->create_subscription<ros_msgs::msg::CameraData>(
        m_config["topic"]["camera_data"].as<std::string>(),
        10,
        std::bind(&Identify::camera_data_received_cbfn, this, std::placeholders::_1)
    );

    m_result_text_publisher = this->create_publisher<std_msgs::msg::String>(
        m_config["topic"]["result_text"].as<std::string>(),
        1
    );

    m_mode_text_publisher = this->create_publisher<std_msgs::msg::String>(
        m_config["topic"]["mode_text"].as<std::string>(),
        1
    );

    m_processed_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        m_config["topic"]["processed_image"].as<std::string>(),
        10
    );

    m_color_depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        m_config["topic"]["color_depth_image"].as<std::string>(),
        10
    );

    m_logger = this->create_publisher<ros_msgs::msg::Log>(
        m_config["topic"]["log_text"].as<std::string>(),
        1
    );

    m_min_distance = m_config["identity"]["min_distance"].as<float>();
    m_max_distance = m_config["identity"]["max_distance"].as<float>();

    m_ready.store(false);
}

Identify::~Identify() {
    this->finalize();
}

void Identify::init() {
    bool success = true;
    success &= acl_utils::init_acl();
    m_logger->publish(vision_utils::create_log(this->now(), vision_utils::LogLevel::ERROR, "acl init %s!", success ? "success" : "failed"));
    const std::string fastsam_model_path = ament_index_cpp::get_package_share_directory("vision_utils") + "/model/fastsam.om";
    const std::string segformer_model_path = ament_index_cpp::get_package_share_directory("vision_utils") + "/model/segformer_identify.om";
    success &= m_fastsam.load_model(fastsam_model_path);
    success &= m_fastsam.init();
    m_logger->publish(vision_utils::create_log(this->now(), success ? vision_utils::LogLevel::INFO : vision_utils::LogLevel::ERROR, "fastsam load %s!", success ? "success" : "failed"));
    success &= m_segformer.load_model(segformer_model_path);
    success &= m_segformer.init();
    m_logger->publish(vision_utils::create_log(this->now(), success ? vision_utils::LogLevel::INFO : vision_utils::LogLevel::ERROR, "segformer load %s!", success ? "success" : "failed"));
    std_msgs::msg::String mode_text;
    mode_text.data = "identify";
    m_mode_text_publisher->publish(mode_text);
    m_ready.store(success);
}

void Identify::run() {
    this->init();
}

void Identify::finalize() {
    m_fastsam.unload_model();
    m_segformer.unload_model();
    acl_utils::finalize_acl();
}

void Identify::camera_data_received_cbfn(const ros_msgs::msg::CameraData::ConstSharedPtr& camera_data)
{
    if (!m_ready.load()) {
        return;
    }
    try {
        cv_bridge::CvImagePtr color_image_ptr = cv_bridge::toCvCopy(camera_data->color_image, "rgb8");
        cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(camera_data->depth_image);
        cv::Mat color_depth_image = vision_utils::depth2rgb(depth_image_ptr->image, m_min_distance, m_max_distance);
        FastSAM::result_t fastsam_result = m_fastsam.inference(color_depth_image);
        Segformer::result_t segformer_result = m_segformer.inference(color_image_ptr->image);
        detection_t detection;
        cv::Mat processed_image = color_image_ptr->image.clone();
        cv::cvtColor(processed_image, processed_image, cv::COLOR_RGB2BGR);
        for (const auto& [mask, bbox] : fastsam_result) {
            cv::rectangle(processed_image, bbox, cv::Scalar(0, 0, 255), 5);
            if (!segformer_result.empty()) {
                cv::Mat hist;
                const int hist_size = 256;
                std::vector<cv::Mat> hist_input{ segformer_result };
                cv::calcHist(hist_input, { 0 }, mask, hist, { hist_size }, { 0,hist_size });
                int max_class = 0;
                int max_count = 0;
                for (int i = 0;i < hist_size;++i) {
                    int count = static_cast<int>(hist.at<float>(i));
                    if (count > max_count) {
                        max_count = count;
                        max_class = i;
                    }
                }
                detection[max_class]++;
            }
        }
        if (!detection.empty()) {
            m_detections.push_back(detection);
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "processed_image";
        cv::cvtColor(processed_image, processed_image, cv::COLOR_BGR2RGB);
        sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_bridge::CvImage(header, "rgb8", processed_image).toImageMsg();
        m_processed_image_publisher->publish(*processed_image_msg);

        header.stamp = this->now();
        header.frame_id = "color_depth_image";
        sensor_msgs::msg::Image::SharedPtr color_depth_image_msg = cv_bridge::CvImage(header, "rgb8", color_depth_image).toImageMsg();
        m_color_depth_image_publisher->publish(*color_depth_image_msg);
    }
    catch (const cv_bridge::Exception& e) {
        m_logger->publish(vision_utils::create_log(this->now(), vision_utils::LogLevel::ERROR, e.what()));
    }
    catch (const cv::Exception& e) {
        m_logger->publish(vision_utils::create_log(this->now(), vision_utils::LogLevel::ERROR, e.what()));
    }
}