#include "ocr_recognizer.hpp"

bool OcrRecognizer::init()
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");

    m_input_width = m_config["ocr_recognizer"]["input_shape"][3].as<int>();
    m_input_height = m_config["ocr_recognizer"]["input_shape"][2].as<int>();

    m_mean = { m_config["ocr_recognizer"]["mean"][0].as<float>(),m_config["ocr_recognizer"]["mean"][1].as<float>(),m_config["ocr_recognizer"]["mean"][2].as<float>() };
    m_std = { m_config["ocr_recognizer"]["std"][0].as<float>(),m_config["ocr_recognizer"]["std"][1].as<float>(),m_config["ocr_recognizer"]["std"][2].as<float>() };

    YAML::Node dict_yaml = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/ocr_dict.yaml");
    for (const auto& label : dict_yaml["character_dict"]) {
        m_label_dict.emplace_back(std::move(label.as<std::string>()));
    }
    m_label_dict.emplace(m_label_dict.begin(), "#"); // ctc解码所需的空字符
    m_label_dict.emplace_back(" ");
    return true;
}

vision_utils::nchw_data_t OcrRecognizer::preprocess_image(const cv::Mat& image) {
    cv::Mat padded = vision_utils::resize_image(image, m_input_width, m_input_height);

    cv::cvtColor(padded, padded, cv::COLOR_BGR2RGB);

    padded = vision_utils::normalization(padded, m_mean, m_std);

    vision_utils::nchw_data_t nchw_data = vision_utils::mat2nchw(padded);

    return nchw_data;
}

OcrRecognizer::result_t OcrRecognizer::inference(const resource_t& resource)
{
    result_t result;
    for (int i = 0;i < resource.size();++i) {
        vision_utils::nchw_data_t nchw_data = preprocess_image(resource[i]);

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

        cv::Mat prediction(output_dim.dims[1], output_dim.dims[2], CV_32F, output.data());

        result.emplace_back(std::move(process_output(prediction)));
    }
    return result;
}

std::string OcrRecognizer::process_output(const cv::Mat& prediction) {
    std::string str_res;
    int argmax_idx;
    int last_index = 0;
    float score = 0.f;
    int count = 0;
    double max_value = 0.0;
    for (int r = 0;r < prediction.rows;++r) {
        cv::Point temp;
        cv::minMaxLoc(prediction.row(r), 0, &max_value, 0, &temp);
        argmax_idx = temp.x;

        if (argmax_idx > 0 && (!(r > 0 && argmax_idx == last_index))) {
            score += max_value;
            count += 1;
            str_res += m_label_dict[argmax_idx];
        }
        last_index = argmax_idx;
    }

    score /= count;

    if (std::isnan(score)) {
        return std::string();
    }

    return str_res;
}

bool OcrRecognizer::finalize()
{
    return false;
}
