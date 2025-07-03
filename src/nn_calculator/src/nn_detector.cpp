#include "nn_detector.hpp"

NnDetector::NnDetector()
{
    init();
}

NnDetector::~NnDetector() {
    release_resource();
}

bool NnDetector::init() {
    // 设置设备
    aclError ret = aclrtSetDevice(m_decive_id);
    if (ret != ACL_SUCCESS) {
        return false;
    }

    // 创建上下文
    ret = aclrtCreateContext(&m_context, m_decive_id);
    if (ret != ACL_SUCCESS) {
        return false;
    }

    // 创建流
    ret = aclrtCreateStream(&m_stream);
    if (ret != ACL_SUCCESS) {
        return false;
    }

    return true;
}

bool NnDetector::load_model(const std::string& model_path)
{
    // 加载模型
    aclError ret = aclmdlLoadFromFile(model_path.c_str(), &m_model_id);
    if (ret != ACL_SUCCESS) {
        return false;
    }

    // 获取模型描述
    m_model_description = aclmdlCreateDesc();
    ret = aclmdlGetDesc(m_model_description, m_model_id);
    if (ret != ACL_SUCCESS) {
        return false;
    }

    // 准备输入输出数据集
    m_input_dataset = aclmdlCreateDataset();
    if (nullptr == m_input_dataset) {
        return false;
    }

    m_output_dataset = aclmdlCreateDataset();
    if (nullptr == m_output_dataset) {
        return false;
    }

    // 分配输入输出内存
    size_t input_num = aclmdlGetNumInputs(m_model_description);
    if (input_num < 1) {
        return false;
    }

    // 准备输入
    for (size_t i = 0; i < input_num; ++i) {
        aclmdlIODims input_dims;
        if (aclmdlGetInputDims(m_model_description, i, &input_dims) != ACL_SUCCESS) {
            return false;
        }

        size_t input_size = aclmdlGetInputSizeByIndex(m_model_description, i);
        void* input_buf = nullptr;
        aclrtMalloc(&input_buf, input_size, ACL_MEM_MALLOC_HUGE_FIRST);

        if (nullptr == input_buf) {
            return false;
        }

        aclDataBuffer* input_data = aclCreateDataBuffer(input_buf, input_size);
        if (nullptr == input_data) {
            aclrtFree(input_buf);
            return false;
        }
        aclmdlAddDatasetBuffer(m_input_dataset, input_data);
    }

    // 准备输出
    size_t output_num = aclmdlGetNumOutputs(m_model_description);
    for (size_t i = 0; i < output_num; ++i) {
        size_t output_size = aclmdlGetOutputSizeByIndex(m_model_description, i);
        void* output_buf = nullptr;
        aclrtMalloc(&output_buf, output_size, ACL_MEM_MALLOC_HUGE_FIRST);
        if (nullptr == output_buf) {
            return false;
        }

        aclDataBuffer* output_data = aclCreateDataBuffer(output_buf, output_size);
        if (nullptr == output_data) {
            aclrtFree(output_buf);
            return false;
        }
        aclmdlAddDatasetBuffer(m_output_dataset, output_data);
    }

    m_model_has_load = true;
    m_is_ok = true;

    return true;
}

bool NnDetector::unload_model()
{
    if (nullptr != m_input_dataset) {
        for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(m_input_dataset); ++i) {
            aclDataBuffer* data_buffer = aclmdlGetDatasetBuffer(m_input_dataset, i);
            void* data = aclGetDataBufferAddr(data_buffer);
            if (data) aclrtFree(data);
            aclDestroyDataBuffer(data_buffer);
        }
        aclmdlDestroyDataset(m_input_dataset);
        m_input_dataset = nullptr;
    }

    if (nullptr != m_output_dataset) {
        for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(m_output_dataset); ++i) {
            aclDataBuffer* data_buffer = aclmdlGetDatasetBuffer(m_output_dataset, i);
            void* data = aclGetDataBufferAddr(data_buffer);
            if (data) aclrtFree(data);
            aclDestroyDataBuffer(data_buffer);
        }
        aclmdlDestroyDataset(m_output_dataset);
        m_output_dataset = nullptr;
    }

    if (nullptr != m_model_description) {
        aclmdlDestroyDesc(m_model_description);
        m_model_description = nullptr;
    }

    if (0 != m_model_id) {
        aclmdlUnload(m_model_id);
        m_model_id = 0;
    }

    m_is_ok = false;
    m_model_has_load = false;

    return true;
}

cv::Mat NnDetector::preprocess_img(const cv::Mat& img) {
    const int model_width = m_config["model"]["input_width"].as<int>();
    const int model_height = m_config["model"]["input_height"].as<int>();

    // 调整大小并保持宽高比
    int img_width = img.cols;
    int img_height = img.rows;

    float scale = std::min(model_width / (float)img_width, model_height / (float)img_height);
    int new_width = img_width * scale;
    int new_height = img_height * scale;

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_width, new_height));

    // 填充到模型输入尺寸
    // 144是灰色
    cv::Mat padded(model_height, model_width, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(0, 0, new_width, new_height)));

    // 转换为RGB
    cv::cvtColor(padded, padded, cv::COLOR_BGR2RGB);

    // 归一化
    padded.convertTo(padded, CV_32FC3, 1.0 / 255.0);

    return padded;
}

ros_msg::msg::NnResult::ConstSharedPtr NnDetector::detect(const cv::Mat& img) {
    // 预处理
    cv::Mat processed = preprocess_img(img);

    // 准备输入数据
    aclDataBuffer* input_data = aclmdlGetDatasetBuffer(m_input_dataset, 0);
    if (nullptr == input_data) {
        return {};
    }

    void* input_buf = aclGetDataBufferAddr(input_data);
    if (nullptr == input_buf) {
        return {};
    }

    // 将图像数据复制到设备内存
    aclrtMemcpy(input_buf, processed.total() * processed.elemSize(),
        processed.data, processed.total() * processed.elemSize(),
        ACL_MEMCPY_HOST_TO_DEVICE);

    // 执行推理
    aclError ret = aclmdlExecute(m_model_id, m_input_dataset, m_output_dataset);
    if (ret != ACL_SUCCESS) {
        return {};
    }

    // 处理输出
    // std::vector<Detection> detections;

    // 获取输出数据
    size_t output_num = aclmdlGetNumOutputs(m_model_description);
    for (size_t i = 0; i < output_num; ++i) {
        aclDataBuffer* output_data = aclmdlGetDatasetBuffer(m_output_dataset, i);
        void* output_buf = aclGetDataBufferAddr(output_data);
        size_t output_size = aclGetDataBufferSizeV2(output_data);

        // 将输出数据复制到主机内存
        float* output = new float[output_size / sizeof(float)];
        aclrtMemcpy(output, output_size, output_buf, output_size, ACL_MEMCPY_DEVICE_TO_HOST);

        // 解析YOLOv11输出 (假设输出格式为 [batch, num_det, 6] 其中6是x1,y1,x2,y2,conf,class)
        // int num_det = static_cast<int>(output[0]);
        // float* detections_ptr = output + 1;

        // for (int j = 0; j < num_det; ++j) {
        //     float* det = detections_ptr + j * 6;

        //     float conf = det[4];
        //     if (conf < 0.5) continue; // 置信度阈值

        //     Detection detection;
        //     detection.bbox.x = static_cast<int>(det[0] * orig_size.width);
        //     detection.bbox.y = static_cast<int>(det[1] * orig_size.height);
        //     detection.bbox.width = static_cast<int>((det[2] - det[0]) * orig_size.width);
        //     detection.bbox.height = static_cast<int>((det[3] - det[1]) * orig_size.height);
        //     detection.confidence = conf;
        //     detection.class_id = static_cast<int>(det[5]);

        //     detections.push_back(detection);
        // }

        delete[] output;
    }

    return {};
}

bool NnDetector::is_ok() const
{
    return this->m_is_ok;
}

bool NnDetector::model_has_load() const
{
    return this->m_model_has_load;
}

bool NnDetector::init_environment()
{
    aclError ret = aclInit(nullptr);
    if (ret != ACL_SUCCESS) {
        return false;
    }

    return true;
}

bool NnDetector::destory_environment()
{
    aclError ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        return false;
    }

    return true;
}

void NnDetector::release_resource() {
    if (m_model_has_load) {
        unload_model();
    }

    if (nullptr != m_stream) {
        aclrtDestroyStream(m_stream);
        m_stream = nullptr;
    }

    if (nullptr != m_context) {
        aclrtDestroyContext(m_context);
        m_context = nullptr;
    }

    m_is_ok = false;

    aclrtResetDevice(m_decive_id);
}