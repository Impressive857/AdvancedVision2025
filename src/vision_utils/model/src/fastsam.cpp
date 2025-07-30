#include "fastsam.hpp"

bool FastSAM::init() {
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");

    m_input_width = m_config["fastsam"]["input_shape"][3].as<int>();
    m_input_height = m_config["fastsam"]["input_shape"][2].as<int>();

    m_conf = m_config["fastsam"]["conf"].as<float>();
    m_iou = m_config["fastsam"]["iou"].as<float>();

    m_mean = { m_config["fastsam"]["mean"][0].as<float>(),m_config["fastsam"]["mean"][1].as<float>(),m_config["fastsam"]["mean"][2].as<float>() };
    m_std = { m_config["fastsam"]["std"][0].as<float>(),m_config["fastsam"]["std"][1].as<float>(),m_config["fastsam"]["std"][2].as<float>() };

    return true;
}

FastSAM::result_t FastSAM::inference(const FastSAM::resource_t& resource) {
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
    if (2 != output_num) {
        return {};
    }
    aclDataBuffer* output_dataset_buffer[2]{ aclmdlGetDatasetBuffer(m_output_dataset, 0), aclmdlGetDatasetBuffer(m_output_dataset, 1) };
    if (nullptr == output_dataset_buffer[0] || nullptr == output_dataset_buffer[1]) {
        return {};
    }

    void* output_data_buffer[2]{ aclGetDataBufferAddr(output_dataset_buffer[0]),aclGetDataBufferAddr(output_dataset_buffer[1]) };
    if (nullptr == output_data_buffer[0] || nullptr == output_data_buffer[1]) {
        return {};
    }

    size_t output_data_size[2]{ aclGetDataBufferSizeV2(output_dataset_buffer[0]),aclGetDataBufferSizeV2(output_dataset_buffer[1]) };
    if (output_data_size[0] <= 0 || output_data_size[1] <= 0) {
        return {};
    }

    std::vector<float> output[2]{ std::vector<float>(output_data_size[0] / sizeof(float)), std::vector<float>(output_data_size[1] / sizeof(float)) };

    ret = aclrtMemcpy(output[0].data(), output_data_size[0], output_data_buffer[0], output_data_size[0], ACL_MEMCPY_DEVICE_TO_HOST);
    if (ACL_SUCCESS != ret) {
        return {};
    }

    ret = aclrtMemcpy(output[1].data(), output_data_size[1], output_data_buffer[1], output_data_size[1], ACL_MEMCPY_DEVICE_TO_HOST);
    if (ACL_SUCCESS != ret) {
        return {};
    }

    aclmdlIODims output_dim[2];
    ret = aclmdlGetOutputDims(m_model_description, 0, &output_dim[0]);
    if (ACL_SUCCESS != ret) {
        return {};
    }

    cv::Mat prediction = cv::Mat(output_dim[0].dims[1], output_dim[0].dims[2], CV_32F, output[0].data()).t();

    ret = aclmdlGetOutputDims(m_model_description, 1, &output_dim[1]);
    if (ACL_SUCCESS != ret) {
        return {};
    }

    cv::Mat proto = cv::Mat(output_dim[1].dims[1], output_dim[1].dims[2] * output_dim[1].dims[3], CV_32F, output[1].data());
    result_t result = process_output(prediction, proto);

    return result;
}

bool FastSAM::finalize() {
    return true;
}

vision_utils::nchw_data_t FastSAM::preprocess_image(const cv::Mat& image) {
    cv::Mat padded = vision_utils::resize_image(image, m_input_width, m_input_height);

    padded = vision_utils::normalization(padded, m_mean, m_std);

    vision_utils::nchw_data_t nchw_data = vision_utils::mat2nchw(padded);

    return nchw_data;
}

FastSAM::result_t FastSAM::process_output(const cv::Mat& prediction, const cv::Mat& protos) {
    std::vector<bbox_t> bboxes_temp;
    std::vector<float> confs_temp;
    const int prediction_num = prediction.rows;
    bboxes_temp.reserve(prediction_num);
    confs_temp.reserve(prediction_num);
    for (int i = 0; i < prediction_num;++i) {
        const float* ptr = prediction.ptr<float>(i);
        float w = ptr[2];
        float h = ptr[3];
        float x = ptr[0] - w / 2;
        float y = ptr[1] - h / 2;
        bboxes_temp.push_back(cv::Rect(x, y, w, h));
        confs_temp.push_back(ptr[4]);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(bboxes_temp, confs_temp, m_conf, m_iou, indices);
    const int mask_num = indices.size();
    if (mask_num <= 0) {
        return {};
    }
    std::vector <bbox_t> bboxes;
    std::vector <float> confs;
    bboxes.reserve(mask_num);
    confs.reserve(mask_num);
    mask_t mask_temp;
    for (int i = 0;i < mask_num;++i) {
        int index = indices[i];
        bbox_t& bbox_temp = bboxes_temp[index];
        bbox_temp.x = std::max(bbox_temp.x, 0);
        bbox_temp.y = std::max(bbox_temp.y, 0);
        bbox_temp.width = std::min(bbox_temp.width, m_input_width);
        bbox_temp.height = std::min(bbox_temp.height, m_input_height);
        bboxes.push_back(bbox_temp);
        confs.push_back(confs_temp[index]);
        mask_temp.push_back(prediction.colRange(5, prediction.cols).rowRange(index, index + 1));
    }
    bboxes_temp.clear();
    bboxes_temp.shrink_to_fit();
    confs_temp.clear();
    confs_temp.shrink_to_fit();
    aclmdlIODims output_dim;
    aclmdlGetOutputDims(m_model_description, 1, &output_dim);
    int h = output_dim.dims[2];
    int w = output_dim.dims[3];
    mask_temp = (mask_temp * protos).t();
    mask_temp = mask_temp.reshape(mask_num, { h,w });
    std::vector<mask_t> masks;
    cv::split(mask_temp, masks);
    cv::Rect padding_roi(0, 0, w, h);
    result_t result;
    cv::Size input_size{ m_input_width, m_input_height };
    for (int i = 0;i < mask_num;++i) {
        cv::Mat temp;
        cv::exp(-masks[i], temp);
        temp = 1.0 / (1.0 + temp);
        temp = temp(padding_roi);
        cv::resize(temp, mask_temp, input_size, cv::INTER_LINEAR);

        const bbox_t& bbox = bboxes[i];
        mask_temp = mask_temp(bbox);

        mask_t mask = cv::Mat(input_size, mask_temp.type(), cv::Scalar(0.0f));

        float rx = std::max(static_cast<float>(bbox.x), 0.0f);
        float ry = std::max(static_cast<float>(bbox.y), 0.0f);

        for (int y = ry, my = 0; my < mask_temp.rows; y++, my++) {
            float* mask_temp_ptr = mask_temp.ptr<float>(my);
            float* mask_ptr = mask.ptr<float>(y);
            for (int x = rx, mx = 0; mx < mask_temp.cols; x++, mx++) {
                mask_ptr[x] = mask_temp_ptr[mx] > 0.5 ? 1.0 : 0.0;
            }
        }

        mask.convertTo(mask, CV_8U);

        result.push_back({ mask, bbox });
    }
    return result;
}