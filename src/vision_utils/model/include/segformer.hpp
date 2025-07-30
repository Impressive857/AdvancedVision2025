#ifndef _SEGFORMER_HPP_
#define _SEGFORMER_HPP_

#include "model_base.hpp"
#include "vision_utils.hpp"

// ros
#include <ament_index_cpp/get_package_share_directory.hpp>

//opencv
#include <opencv2/opencv.hpp>

// yaml
#include <yaml-cpp/yaml.h>

class Segformer
    :public ModelBase<cv::Mat, cv::Mat>
{
public:
    using resource_t = cv::Mat;
    using result_t = cv::Mat;
    Segformer() = default;
    ~Segformer() override = default;
    bool init() override;
    result_t inference(const resource_t& resource) override;
    bool finalize() override;
private:
    vision_utils::nchw_data_t preprocess_image(const cv::Mat& image);
    result_t process_output(const cv::Mat& prediction);
private:
    int m_input_width = 0;
    int m_input_height = 0;
    std::array<float, 3> m_mean{};
    std::array<float, 3> m_std{};
    YAML::Node m_config;
};

#endif // !_SEGFORMER_HPP_