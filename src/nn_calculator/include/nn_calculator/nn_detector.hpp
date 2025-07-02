#ifndef _NN_DETECTOR_HPP_
#define _NN_DETECTOR_HPP_

// std
#include <memory>

// opencv
#include <opencv2/opencv.hpp>

// ascendcl
#include "acl/acl.h"

// ros
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ros_msg/msg/nn_result.hpp"

// cv_bridge
#include <cv_bridge/cv_bridge.h>

// yaml
#include <yaml-cpp/yaml.h>

#include "vision_utils.hpp"

class NnDetector
{
public:
    NnDetector();
    bool load_model(const std::string& model_path);
    bool unload_model();
    ros_msg::msg::NnResult::ConstSharedPtr detect(const cv::Mat& img);
    bool is_ok() const;
    bool model_has_load() const;
    static bool init_environment();
    static bool destory_environment();
    ~NnDetector();
protected:
    bool init();
    const cv::Mat& preprocess_img(const cv::Mat& img);
    void release_resource();
    bool m_is_ok = false;
    bool m_model_has_load = false;
    uint32_t m_model_id = 0;
    int32_t m_decive_id = 0;
    aclmdlDesc* m_model_description = nullptr;
    aclrtContext m_context = nullptr;
    aclrtStream m_stream = nullptr;
    aclmdlDataset* m_input_dataset = nullptr;
    aclmdlDataset* m_output_dataset = nullptr;
    YAML::Node m_config = YAML::LoadFile("../../config.yaml");
};

#endif // !_NN_DETECTOR_HPP_