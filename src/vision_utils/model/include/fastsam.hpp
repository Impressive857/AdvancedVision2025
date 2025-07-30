#ifndef _FASTSAM_HPP_
#define _FASTSAM_HPP_

#include "model_base.hpp"
#include "vision_utils.hpp"

// opencv
#include <opencv2/opencv.hpp>

// ros
#include <ament_index_cpp/get_package_share_directory.hpp>

// yaml
#include <yaml-cpp/yaml.h>

class FastSAM
    :public ModelBase<cv::Mat, std::vector<std::pair<cv::Mat, cv::Rect>>>
{
public:
    using resource_t = cv::Mat;
    using mask_t = cv::Mat;
    using bbox_t = cv::Rect;
    using result_t = std::vector<std::pair<mask_t, bbox_t>>;
    FastSAM() = default;
    bool init() override;
    result_t inference(const resource_t& resource) override;
    bool finalize() override;
    ~FastSAM() override = default;
private:
    vision_utils::nchw_data_t preprocess_image(const cv::Mat& image);
    result_t process_output(const cv::Mat& prediction, const cv::Mat& protos);
private:
    float m_conf = 0.0f;
    float m_iou = 0.0f;
    int m_input_width = 0;
    int m_input_height = 0;
    std::array<float, 3> m_mean{};
    std::array<float, 3> m_std{};
    YAML::Node m_config;
};

#endif // !_FASTSAM_HPP_