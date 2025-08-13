#include "ocr_detector.hpp"

bool OcrDetector::init()
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");

    m_input_width = m_config["ocr_detector"]["input_shape"][3].as<int>();
    m_input_height = m_config["ocr_detector"]["input_shape"][2].as<int>();

    m_mean = { m_config["ocr_detector"]["mean"][0].as<float>(),m_config["ocr_detector"]["mean"][1].as<float>(),m_config["ocr_detector"]["mean"][2].as<float>() };
    m_std = { m_config["ocr_detector"]["std"][0].as<float>(),m_config["ocr_detector"]["std"][1].as<float>(),m_config["ocr_detector"]["std"][2].as<float>() };

    m_min_box_size = m_config["ocr_detector"]["min_box_size"].as<float>();
    m_min_box_score = m_config["ocr_detector"]["min_box_score"].as<float>();
    m_unclip_ratio = m_config["ocr_detector"]["unclip_ratio"].as<float>();
    m_bitmap_thresh = m_config["ocr_detector"]["bitmap_thresh"].as<float>();

    m_dst_width = m_config["ocr_recognizer"]["input_shape"][3].as<int>();
    m_dst_height = m_config["ocr_recognizer"]["input_shape"][2].as<int>();

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

    cv::Mat prediction(output_dim.dims[2], output_dim.dims[3], CV_32F, output.data());

    return process_output(prediction, resource);
}

OcrDetector::result_t OcrDetector::process_output(const cv::Mat& prediction, const cv::Mat& src_image) {
    cv::Mat bitmap;
    prediction.convertTo(bitmap, CV_8U, 255);
    cv::threshold(bitmap, bitmap, (int)m_bitmap_thresh * 255, 255, cv::THRESH_BINARY);
    cv::dilate(bitmap, bitmap, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bitmap, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    int contour_num = contours.size();
    std::vector<points_t> boxes;

    for (int i = 0;i < contour_num;++i) {
        const auto& contour = contours[i];
        // 轮廓小于等于两个点没办法形成平面
        if (contour.size() <= 2) {
            continue;
        }

        cv::RotatedRect box = cv::minAreaRect(contour);

        float ssid = 0.f;
        vision_utils::mat_vec_fp32_t array;
        std::tie(array, ssid) = get_mini_boxes(box);

        if (ssid < m_min_box_size) {
            continue;
        }

        float score = box_score(array, prediction);

        if (score < m_min_box_score) {
            continue;
        }

        cv::RotatedRect points = unclip(array, m_unclip_ratio);
        if (points.size.height < 1.001 && points.size.width < 1.001) {
            continue;
        }

        std::tie(array, ssid) = get_mini_boxes(points);

        if (ssid < m_min_box_size + 2) {
            continue;
        }

        points_t int_clip_array;
        for (int point_num = 0; point_num < 4;++point_num) {
            point_t temp{ (int)array[point_num][0], (int)array[point_num][0] };
            int_clip_array.emplace_back(std::move(temp));
        }
        boxes.emplace_back(std::move(int_clip_array));
    }

    result_t result;
    for (const auto& box : boxes) {
        cv::Mat clip_image = get_rotate_crop_image(src_image, box);
        result.emplace_back(std::move(clip_image));
    }
    return result;
}

std::pair<vision_utils::mat_vec_fp32_t, float> OcrDetector::get_mini_boxes(const cv::RotatedRect& box) noexcept {
    float ssid = std::max(box.size.width, box.size.height);

    cv::Mat points;
    cv::boxPoints(box, points);

    vision_utils::mat_vec_fp32_t array = vision_utils::mat2vector_fp32(points);
    std::sort(array.begin(), array.end(), x_sort_fp32);

    std::vector<float> idx1 = array[0], idx2 = array[1], idx3 = array[2],
        idx4 = array[3];
    if (array[3][1] <= array[2][1]) {
        idx2 = array[3];
        idx3 = array[2];
    }
    else {
        idx2 = array[2];
        idx3 = array[3];
    }
    if (array[1][1] <= array[0][1]) {
        idx1 = array[1];
        idx4 = array[0];
    }
    else {
        idx1 = array[0];
        idx4 = array[1];
    }

    array[0] = idx1;
    array[1] = idx2;
    array[2] = idx3;
    array[3] = idx4;

    return { array, ssid };
}

bool OcrDetector::x_sort_fp32(const std::vector<float>& a, const std::vector<float>& b) noexcept {
    if (a[0] != b[0])
        return a[0] < b[0];
    return false;
}

float OcrDetector::box_score(const vision_utils::mat_vec_fp32_t& box_array, const cv::Mat& prediction) noexcept {
    const auto& array = box_array;

    float box_x[4] = { array[0][0], array[1][0], array[2][0], array[3][0] };
    float box_y[4] = { array[0][1], array[1][1], array[2][1], array[3][1] };

    int xmin = clamp(int(std::floor(*(std::min_element(box_x, box_x + 4)))), 0,
        m_input_width - 1);
    int xmax = clamp(int(std::ceil(*(std::max_element(box_x, box_x + 4)))), 0,
        m_input_width - 1);
    int ymin = clamp(int(std::floor(*(std::min_element(box_y, box_y + 4)))), 0,
        m_input_height - 1);
    int ymax = clamp(int(std::ceil(*(std::max_element(box_y, box_y + 4)))), 0,
        m_input_height - 1);

    cv::Mat mask;
    mask = cv::Mat::zeros(ymax - ymin + 1, xmax - xmin + 1, CV_8UC1);

    cv::Point root_point[4];
    root_point[0] = cv::Point(int(array[0][0]) - xmin, int(array[0][1]) - ymin);
    root_point[1] = cv::Point(int(array[1][0]) - xmin, int(array[1][1]) - ymin);
    root_point[2] = cv::Point(int(array[2][0]) - xmin, int(array[2][1]) - ymin);
    root_point[3] = cv::Point(int(array[3][0]) - xmin, int(array[3][1]) - ymin);
    const cv::Point* ppt[1] = { root_point };
    int npt[] = { 4 };
    cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(1));

    cv::Mat croppedImg;
    prediction(cv::Rect(xmin, ymin, xmax - xmin + 1, ymax - ymin + 1)).copyTo(croppedImg);

    auto score = cv::mean(croppedImg, mask)[0];
    return score;
}

cv::RotatedRect OcrDetector::unclip(const pointsf_t& box, const float& unclip_ratio) noexcept {
    float distance = get_unclip_distance(box, unclip_ratio);

    ClipperLib::ClipperOffset offset;
    ClipperLib::Path p;
    p.emplace_back(int(box[0][0]), int(box[0][1]));
    p.emplace_back(int(box[1][0]), int(box[1][1]));
    p.emplace_back(int(box[2][0]), int(box[2][1]));
    p.emplace_back(int(box[3][0]), int(box[3][1]));
    offset.AddPath(p, ClipperLib::jtRound, ClipperLib::etClosedPolygon);

    ClipperLib::Paths soln;
    if (!offset.Execute(soln, distance))
        return cv::RotatedRect();

    std::vector<cv::Point2f> points;

    for (size_t j = 0; j < soln.size(); ++j) {
        for (size_t i = 0; i < soln[soln.size() - 1].size(); ++i) {
            points.emplace_back(soln[j][i].X, soln[j][i].Y);
        }
    }
    cv::RotatedRect res;
    if (points.size() <= 0) {
        res = cv::RotatedRect(cv::Point2f(0, 0), cv::Size2f(1, 1), 0);
    }
    else {
        res = cv::minAreaRect(points);
    }
    return res;
}

float OcrDetector::get_unclip_distance(const pointsf_t& box, float unclip_ratio) noexcept {
    int pts_num = 4;
    float area = 0.0f;
    float dist = 0.0f;
    for (int i = 0; i < pts_num; ++i) {
        area += box[i][0] * box[(i + 1) % pts_num][1] -
            box[i][1] * box[(i + 1) % pts_num][0];
        dist += sqrtf((box[i][0] - box[(i + 1) % pts_num][0]) *
            (box[i][0] - box[(i + 1) % pts_num][0]) +
            (box[i][1] - box[(i + 1) % pts_num][1]) *
            (box[i][1] - box[(i + 1) % pts_num][1]));
    }
    area = fabs(float(area / 2.0));

    float distance = area * unclip_ratio / dist;

    return distance;
}

cv::Mat OcrDetector::get_rotate_crop_image(const cv::Mat& src_image, const points_t& box) noexcept {
    cv::Mat image;
    src_image.copyTo(image);
    std::vector<std::vector<int>> points = box;

    int x_collect[4] = { box[0][0], box[1][0], box[2][0], box[3][0] };
    int y_collect[4] = { box[0][1], box[1][1], box[2][1], box[3][1] };
    int left = int(*std::min_element(x_collect, x_collect + 4));
    int right = int(*std::max_element(x_collect, x_collect + 4));
    int top = int(*std::min_element(y_collect, y_collect + 4));
    int bottom = int(*std::max_element(y_collect, y_collect + 4));

    cv::Mat img_crop;
    image(cv::Rect(left, top, right - left, bottom - top)).copyTo(img_crop);

    for (size_t i = 0; i < points.size(); ++i) {
        points[i][0] -= left;
        points[i][1] -= top;
    }

    int img_crop_width = int(sqrt(pow(points[0][0] - points[1][0], 2) +
        pow(points[0][1] - points[1][1], 2)));
    int img_crop_height = int(sqrt(pow(points[0][0] - points[3][0], 2) +
        pow(points[0][1] - points[3][1], 2)));

    const cv::Point2f pts_std[4] = {
        {0., 0.},
        {(float)img_crop_width, 0.},
        {(float)img_crop_width, (float)img_crop_height},
        {0.f, (float)img_crop_height} };

    const cv::Point2f pointsf[4] = { {(float)points[0][0], (float)points[0][1]},
                                    {(float)points[1][0], (float)points[1][1]},
                                    {(float)points[2][0], (float)points[2][1]},
                                    {(float)points[3][0], (float)points[3][1]} };

    cv::Mat M = cv::getPerspectiveTransform(pointsf, pts_std);

    cv::Mat dst_img;
    cv::warpPerspective(img_crop, dst_img, M, cv::Size(img_crop_width, img_crop_height), cv::BORDER_REPLICATE);

    cv::Mat result;

    // 如果是竖版文字就转为横向
    if (float(dst_img.rows) >= float(dst_img.cols) * 1.5) {
        cv::Mat src_copy(dst_img.rows, dst_img.cols, dst_img.depth());
        cv::transpose(dst_img, src_copy);
        cv::flip(src_copy, result, 0);
    }
    return vision_utils::resize_with_padding(result, m_dst_width, m_dst_height);
}

bool OcrDetector::finalize()
{
    return false;
}