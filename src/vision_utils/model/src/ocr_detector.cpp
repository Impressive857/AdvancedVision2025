#include "ocr_detector.hpp"

bool OcrDetector::init()
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config.yaml");

    m_input_width = m_config["ocr_detector"]["input_width"].as<int>();
    m_input_height = m_config["ocr_detector"]["input_height"].as<int>();

    m_mean = { m_config["ocr_detector"]["mean"][0].as<float>(),m_config["ocr_detector"]["mean"][1].as<float>(),m_config["ocr_detector"]["mean"][2].as<float>() };
    m_std = { m_config["ocr_detector"]["std"][0].as<float>(),m_config["ocr_detector"]["std"][1].as<float>(),m_config["ocr_detector"]["std"][2].as<float>() };

    return true;
}

vision_utils::nchw_data_t OcrDetector::preprocess_image(const cv::Mat& image) {
    cv::Mat padded = vision_utils::resize_image(image, m_input_width, m_input_height);

    cv::cvtColor(padded, padded, cv::COLOR_BGR2RGB);

    padded = vision_utils::normalization(padded, m_mean, m_std);

    vision_utils::nchw_data_t nchw_data = vision_utils::mat2nchw(padded);

    return nchw_data;
}

OcrDetector::result_t OcrDetector::inference(const resource_t& resource)
{
    vision_utils::nchw_data_t nchw_data = preprocess_image(resource);

    aclDataBuffer* input_data = aclmdlGetDatasetBuffer(m_input_dataset, 0);
    if (nullptr == input_data) {
        return {};
    }

    void* input_buf = aclGetDataBufferAddr(input_data);
    if (nullptr == input_buf) {
        return {};
    }

    aclError ret = aclrtMemcpy(input_buf, nchw_data.size() * sizeof(float), nchw_data.data(), nchw_data.size() * sizeof(float), ACL_MEMCPY_HOST_TO_DEVICE);
    if (ret != ACL_SUCCESS) {
        return {};
    }

    ret = aclmdlExecute(m_model_id, m_input_dataset, m_output_dataset);
    if (ret != ACL_SUCCESS) {
        return {};
    }

    size_t output_num = aclmdlGetNumOutputs(m_model_description);
    if (1 != output_num) {
        return {};
    }
    aclDataBuffer* output_dataset_buffer = aclmdlGetDatasetBuffer(m_output_dataset, 0);
    if (nullptr == output_dataset_buffer) {
        return {};
    }

    void* output_data_buffer = aclGetDataBufferAddr(output_dataset_buffer);
    if (nullptr == output_data_buffer) {
        return {};
    }

    size_t output_data_size = aclGetDataBufferSizeV2(output_dataset_buffer);

    if (output_data_size <= 0) {
        return {};
    }

    std::shared_ptr<float> output = std::make_shared<float>(output_data_size / sizeof(float));

    ret = aclrtMemcpy(output.get(), output_data_size, output_data_buffer, output_data_size, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ACL_SUCCESS != ret) {
        return {};
    }

    aclmdlIODims output_dim;
    ret = aclmdlGetOutputDims(m_model_description, 0, &output_dim);
    if (ACL_SUCCESS != ret) {
        return {};
    }
}

bool OcrDetector::finalize()
{
    return false;
}