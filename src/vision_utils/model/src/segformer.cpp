#include "segformer.hpp"

bool Segformer::init()
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");

    m_input_width = m_config["segformer"]["input_shape"][3].as<int>();
    m_input_height = m_config["segformer"]["input_shape"][2].as<int>();

    m_mean = { m_config["segformer"]["mean"][0].as<float>(),m_config["segformer"]["mean"][1].as<float>(),m_config["segformer"]["mean"][2].as<float>() };
    m_std = { m_config["segformer"]["std"][0].as<float>(),m_config["segformer"]["std"][1].as<float>(),m_config["segformer"]["std"][2].as<float>() };

    return true;
}

vision_utils::nchw_data_t Segformer::preprocess_image(const cv::Mat& image) {
    cv::Mat padded = vision_utils::resize_image(image, m_input_width, m_input_height);

    padded = vision_utils::normalization(padded, m_mean, m_std);

    vision_utils::nchw_data_t nchw_data = vision_utils::mat2nchw(padded);

    return nchw_data;
}

Segformer::result_t Segformer::process_output(const cv::Mat& prediction)
{
    aclmdlIODims output_dim;
    aclError ret = aclmdlGetOutputDims(m_model_description, 0, &output_dim);
    if (ACL_SUCCESS != ret) {
        return {};
    }
    const int output_height = output_dim.dims[2];
    const int output_width = output_dim.dims[3];
    cv::Mat result(output_height, output_width, CV_8U, cv::Scalar(0));
    cv::Mat probabilities;
    cv::exp(prediction, probabilities);
    cv::Mat probabilities_sum;
    cv::reduce(probabilities, probabilities_sum, 0, cv::REDUCE_SUM);
    for(int r = 0; r < probabilities.rows; ++r){
        probabilities.row(r) /= probabilities_sum;
    }
    for (int c = 0; c < probabilities.cols;++c) {
        int max_class = 0;
        float max_conf = probabilities.at<float>(0, c);
        for (int r = 0;r < probabilities.rows;++r) {
            float conf = probabilities.at<float>(r, c);
            if (conf > max_conf) {
                max_conf = conf;
                max_class = r;
            }
        }
        result.at<uchar>(c / output_width, c % output_width) = max_class;
    }

    cv::resize(result, result, cv::Size(m_input_width, m_input_height), cv::INTER_NEAREST);

    return result;
}

Segformer::result_t Segformer::inference(const resource_t& resource)
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

    std::vector<float> output(output_data_size / sizeof(float));

    ret = aclrtMemcpy(output.data(), output_data_size, output_data_buffer, output_data_size, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ACL_SUCCESS != ret) {
        return {};
    }

    aclmdlIODims output_dim;
    ret = aclmdlGetOutputDims(m_model_description, 0, &output_dim);
    if (ACL_SUCCESS != ret) {
        return {};
    }

    cv::Mat prediction = cv::Mat(output_dim.dims[1], output_dim.dims[2] * output_dim.dims[3], CV_32F, output.data());

    result_t result = process_output(prediction);

    return result;
}

bool Segformer::finalize()
{
    return false;
}