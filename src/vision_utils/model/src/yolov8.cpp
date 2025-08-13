#include "yolov8.hpp"

bool YoloV8::init()
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");

    m_input_width = m_config["yolov8"]["input_shape"][3].as<int>();
    m_input_height = m_config["yolov8"]["input_shape"][2].as<int>();

    m_conf = m_config["yolov8"]["conf"].as<float>();
    m_iou = m_config["yolov8"]["iou"].as<float>();

    m_mean = { m_config["yolov8"]["mean"][0].as<float>(),m_config["yolov8"]["mean"][1].as<float>(),m_config["yolov8"]["mean"][2].as<float>() };
    m_std = { m_config["yolov8"]["std"][0].as<float>(),m_config["yolov8"]["std"][1].as<float>(),m_config["yolov8"]["std"][2].as<float>() };

    return true;
}

vision_utils::nchw_data_t YoloV8::preprocess_image(const cv::Mat& image) {
    cv::Mat padded = vision_utils::resize_image(image, m_input_width, m_input_height);

    padded = vision_utils::normalization(padded, m_mean, m_std);

    vision_utils::nchw_data_t nchw_data = vision_utils::mat2nchw(padded);

    return nchw_data;
}
YoloV8::result_t YoloV8::process_output(const cv::Mat& prediction) {
    std::vector<bbox_t> bboxes_temp;
    std::vector<class_id_t> class_ids_temp;
    std::vector<float> confs_temp;
    const int prediction_num = prediction.rows;
    bboxes_temp.reserve(prediction_num);
    for (int i = 0; i < prediction_num;++i) {
        const float* ptr = prediction.ptr<float>(i);
        float w = ptr[2];
        float h = ptr[3];
        float x = ptr[0] - w / 2;
        float y = ptr[1] - h / 2;
        bboxes_temp.push_back(cv::Rect(x, y, w, h));

        double max_conf;
        cv::Point class_id;
        cv::Mat scores = prediction.row(i).colRange(4, prediction.cols);
        cv::minMaxLoc(scores, 0, &max_conf, 0, &class_id);
        class_ids_temp.push_back(class_id.x);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(bboxes_temp, confs_temp, m_conf, m_iou, indices);
    const int object_num = indices.size();
    if (object_num <= 0) {
        return {};
    }

    result_t result;
    result.reserve(object_num);
    for (int i = 0;i < object_num;++i) {
        const int index = indices[i];
        bbox_t& bbox = bboxes_temp[index];
        bbox.x = std::max(bbox.x, 0);
        bbox.y = std::max(bbox.y, 0);
        bbox.width = std::min(bbox.width, m_input_width);
        bbox.height = std::min(bbox.height, m_input_height);

        class_id_t class_id = class_ids_temp[index];

        result.push_back({ class_id, bbox });
    }

    return result;
}

YoloV8::result_t YoloV8::inference(const resource_t& resource)
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

    cv::Mat prediction = cv::Mat(output_dim.dims[1], output_dim.dims[2], CV_32F, output.data()).t();

    return process_output(prediction);
}

bool YoloV8::finalize()
{
    return false;
}

