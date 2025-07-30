#ifndef _OCR_DETECTOR_HPP_
#define _OCR_DETECTOR_HPP_

#include "model_base.hpp"
#include "vision_utils.hpp"

// opencv
#include <opencv2/opencv.hpp>

// ros
#include <ament_index_cpp/get_package_share_directory.hpp>

// yaml
#include <yaml-cpp/yaml.h>

class OcrDetector
    :public ModelBase<cv::Mat, std::vector<cv::Mat>>
{
public:
    using resource_t = cv::Mat;
    using result_t = std::vector<cv::Mat>;
    bool init() override;
    vision_utils::nchw_data_t preprocess_image(const cv::Mat& image);
    result_t inference(const resource_t& resource) override;
    bool finalize() override;
private:
    int m_input_width = 0;
    int m_input_height = 0;
    std::array<float, 3> m_mean{};
    std::array<float, 3> m_std{};
    YAML::Node m_config;
};

#endif // !_OCR_DETECTOR_HPP_