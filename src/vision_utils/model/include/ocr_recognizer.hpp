#ifndef _OCR_RECOGNIZER_HPP_
#define _OCR_RECOGNIZER_HPP_

#include "model_base.hpp"
#include "vision_utils.hpp"

// opencv
#include <opencv2/opencv.hpp>

// ros
#include <ament_index_cpp/get_package_share_directory.hpp>

// yaml
#include <yaml-cpp/yaml.h>

class OcrRecognizer
    : public ModelBase<std::vector<cv::Mat>, std::vector<std::string>>
{
public:
    using resource_t = std::vector<cv::Mat>;
    using result_t = std::vector<std::string>;
    bool init() override;
    result_t inference(const resource_t& resource) override;
    bool finalize() override;
private:
    vision_utils::nchw_data_t preprocess_image(const cv::Mat& image);
    std::string process_output(const cv::Mat& prediction);

private:
    int m_input_width = 0;
    int m_input_height = 0;
    std::vector<std::string> m_label_dict;
    std::array<float, 3> m_mean{};
    std::array<float, 3> m_std{};
    YAML::Node m_config;
};

#endif // !_OCR_RECOGNIZER_HPP_