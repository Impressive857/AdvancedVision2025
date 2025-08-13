#ifndef _OCR_DETECTOR_HPP_
#define _OCR_DETECTOR_HPP_

#include "model_base.hpp"
#include "vision_utils.hpp"
#include "clipper.h"

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
    using point_t = std::vector<int>;
    using points_t = std::vector<point_t>;
    using pointf_t = std::vector<float>;
    using pointsf_t = std::vector<pointf_t>;
    bool init() override;
    result_t inference(const resource_t& resource) override;
    bool finalize() override;
private:
    vision_utils::nchw_data_t preprocess_image(const cv::Mat& image);
    result_t process_output(const cv::Mat& prediction, const cv::Mat& src_image);
    std::pair<vision_utils::mat_vec_fp32_t, float> get_mini_boxes(const cv::RotatedRect& box) noexcept;
    bool x_sort_fp32(const std::vector<float>& a, const std::vector<float>& b) noexcept;
    float box_score(const vision_utils::mat_vec_fp32_t& box_array, const cv::Mat& prediction) noexcept;
    cv::RotatedRect unclip(const pointsf_t& box, const float& unclip_ratio) noexcept;
    float get_unclip_distance(const pointsf_t& box, float unclip_ratio) noexcept;
    cv::Mat OcrDetector::get_rotate_crop_image(const cv::Mat& src_image, const points_t& box);
    template <class T> inline T clamp(T x, T min, T max) const noexcept {
        if (x > max)
            return max;
        if (x < min)
            return min;
        return x;
    }

    inline float clampf(float x, float min, float max) const noexcept {
        if (x > max)
            return max;
        if (x < min)
            return min;
        return x;
    }
private:
    float m_unclip_ratio = 0.f;
    float m_min_box_size = 0.f;
    float m_min_box_score = 0.f;
    float m_bitmap_thresh = 0.f;
    int m_input_width = 0;
    int m_input_height = 0;
    int m_dst_width = 0;
    int m_dst_height = 0;
    std::array<float, 3> m_mean{};
    std::array<float, 3> m_std{};
    YAML::Node m_config;
};

#endif // !_OCR_DETECTOR_HPP_