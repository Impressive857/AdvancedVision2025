#ifndef _VISION_UTILS_CPP_
#define _VISION_UTILS_CPP_

// std
#include <cstdio>

// ros
#include <rclcpp/rclcpp.hpp>
#include "ros_msgs/msg/log.hpp"

// opencv
#include <opencv2/opencv.hpp>

struct vision_utils {

    enum class LogLevel : int32_t {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };

    static ros_msgs::msg::Log create_log(const rclcpp::Time& time, LogLevel level, const std::string& text) {
        ros_msgs::msg::Log log;
        log.header.stamp = time;
        log.level = static_cast<int32_t>(level);
        log.text = text;
        return log;
    }

    template <typename... _Args>
    static ros_msgs::msg::Log create_log(const rclcpp::Time& time, LogLevel level, const std::string& format, _Args...args) {
        ros_msgs::msg::Log log;
        log.header.stamp = time;
        log.level = static_cast<int32_t>(level);
        int len = snprintf(nullptr, 0, format.c_str(), args...);
        if (len <= 0) {
            log.text = "failed format!";
            log.level = static_cast<int32_t>(LogLevel::ERROR);
            return log;
        }
        std::string text(len + 1, '\0');
        int res = snprintf(&text[0], len + 1, format.c_str(), args...);
        if (res < 0 || res >= static_cast<int>(text.size())) {
            log.text = "failed format!";
            log.level = static_cast<int32_t>(LogLevel::ERROR);
            return log;
        }
        text.resize(res);
        log.text = text;
        return log;
    }

    static cv::Mat resize_image(const cv::Mat& image, int width, int height) {
        if (image.cols == height && image.rows == width) {
            return image;
        }

        int image_width = image.cols;
        int image_height = image.rows;

        float scale = std::min(width / (float)image_width, height / (float)image_height);
        int new_width = image_width * scale;
        int new_height = image_height * scale;

        cv::Mat resized;
        cv::resize(image, resized, cv::Size(new_width, new_height));

        cv::Mat padded(height, width, image.type(), cv::Scalar(114));
        resized.copyTo(padded(cv::Rect(0, 0, new_width, new_height)));

        return padded;
    }

    static cv::Mat normalization(const cv::Mat& image, std::array<float, 3> mean, std::array<float, 3> std) {
        cv::Mat res;
        image.convertTo(res, CV_32FC3, 1.0 / 255.0);

        for (int y = 0; y < res.rows; ++y) {
            for (int x = 0; x < res.cols; ++x) {
                cv::Vec3f& pixel = res.at<cv::Vec3f>(y, x);

                pixel[0] = (pixel[0] - mean[0]) / std[0];
                pixel[1] = (pixel[1] - mean[1]) / std[1];
                pixel[2] = (pixel[2] - mean[2]) / std[2];
            }
        }

        return res;
    }

    static cv::Mat resize_with_padding(const cv::Mat& image, int width, int height) {
        float scale_w = (float)width / image.cols;
        float scale_h = (float)height / image.rows;
        float scale = std::min(scale_w, scale_h);

        cv::Mat scaled_img;
        cv::resize(image, scaled_img, cv::Size(), scale, scale, cv::INTER_LINEAR);

        int pad_w = width - scaled_img.cols;
        int pad_h = height - scaled_img.rows;
        int top = pad_h / 2;
        int bottom = pad_h - top;
        int left = pad_w / 2;
        int right = pad_w - left;

        cv::Mat padded_img;
        cv::copyMakeBorder(scaled_img, padded_img, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        return padded_img;
    }

    static cv::Mat depth2rgb(const cv::Mat& depth_image, float min_distance, float max_distance) {
        cv::Mat rgb_image;
        depth_image.convertTo(rgb_image, CV_32FC1, 0.001);

        cv::Mat mask = rgb_image < min_distance;
        rgb_image.setTo(min_distance, mask);
        mask = rgb_image > max_distance;
        rgb_image.setTo(max_distance, mask);

        cv::Mat normalized_depth;
        cv::normalize(rgb_image, normalized_depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        cv::applyColorMap(normalized_depth, rgb_image, cv::COLORMAP_JET);

        cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGR2RGB);

        return rgb_image;
    }

    using mat_vec_fp32_t = std::vector<std::vector<float>>;
    static mat_vec_fp32_t mat2vector_fp32(const cv::Mat& mat) noexcept {
        mat_vec_fp32_t image_vec;

        for (int i = 0; i < mat.rows; ++i) {
            std::vector<float> temp;
            for (int j = 0; j < mat.cols; ++j) {
                temp.emplace_back(mat.at<float>(i, j));
            }
            image_vec.emplace_back(std::move(temp));
        }
        return image_vec;
    }

    using nchw_data_t = std::vector<float>;
    static nchw_data_t mat2nchw(const cv::Mat& image) {
        size_t chw_size = 3 * image.cols * image.rows;
        nchw_data_t nchw_data(chw_size);

        int idx = 0;
        for (int c = 0; c < 3; ++c) {
            for (int h = 0; h < image.rows; ++h) {
                for (int w = 0; w < image.cols; ++w) {
                    nchw_data[idx++] = image.at<cv::Vec3f>(h, w)[c];
                }
            }
        }

        return nchw_data;
    }
};

#endif // !_VISION_UTILS_CPP_