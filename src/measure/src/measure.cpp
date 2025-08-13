#include "measure.hpp"

aclrtStream acl_utils::model_stream = nullptr;
aclrtContext acl_utils::model_context = nullptr;

Measure::Measure()
    :rclcpp::Node("measure_node")
{
    m_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");

    m_camera_data_subscription = this->create_subscription<ros_msgs::msg::CameraData>(
        m_config["topic"]["camera_data"].as<std::string>(),
        10,
        std::bind(&Measure::camera_data_received_cbfn, this, std::placeholders::_1)
    );

    m_result_text_publisher = this->create_publisher<std_msgs::msg::String>(
        m_config["topic"]["result_text"].as<std::string>(),
        1
    );

    m_processed_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        m_config["topic"]["processed_image"].as<std::string>(),
        10
    );

    m_color_depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        m_config["topic"]["color_depth_image"].as<std::string>(),
        10
    );

    m_logger = this->create_publisher<ros_msgs::msg::Log>(
        m_config["topic"]["log_text"].as<std::string>(),
        1
    );

    m_result_timer = this->create_wall_timer(
        std::chrono::seconds(m_config["measure"]["result_time"].as<int>()),
        std::bind(&Measure::result_timer_cbfn, this)
    );

    m_min_distance = m_config["measure"]["min_distance"].as<float>();
    m_max_distance = m_config["measure"]["max_distance"].as<float>();

    this->declare_parameter("round", 1);
    m_current_round = this->get_parameter("round").as_int();

    m_ready.store(false);
}

void Measure::init() {
    bool success = true;
    success &= acl_utils::init_acl();
    m_logger->publish(vision_utils::create_log(this->now(), vision_utils::LogLevel::ERROR, "acl init %s!", success ? "success" : "failed"));
    const std::string yolov8_model_path = ament_index_cpp::get_package_share_directory("vision_utils") + "/model/yolov8.om";
    success &= m_yolov8.load_model(yolov8_model_path);
    success &= m_yolov8.init();
    m_logger->publish(vision_utils::create_log(this->now(), success ? vision_utils::LogLevel::INFO : vision_utils::LogLevel::ERROR, "yolov8 load %s!", success ? "success" : "failed"));
    m_ready.store(success);
}

void Measure::run() {
    this->init();
}

void Measure::finalize() {

}

void Measure::result_timer_cbfn() {
    this->calcResult();
    std_msgs::msg::String result_text_msg;
    result_text_msg.data = this->getResult();
    m_result_text_publisher->publish(result_text_msg);
    m_logger->publish(vision_utils::create_log(this->now(), vision_utils::LogLevel::INFO, "measure result send!"));
}

void Measure::camera_data_received_cbfn(const ros_msgs::msg::CameraData::ConstSharedPtr& camera_data) {
    if (!m_ready.load()) {
        return;
    }
    try {
        cv_bridge::CvImagePtr color_image_ptr = cv_bridge::toCvCopy(camera_data->color_image, "rgb8");
        cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(camera_data->depth_image);
        cv::Mat color_depth_image = vision_utils::depth2rgb(depth_image_ptr->image, m_min_distance, m_max_distance);

        YoloV8::result_t yolov8_result = m_yolov8.inference(color_image_ptr->image);

        if (!yolov8_result.empty()) {
            detections detects;
            cv::Mat image = color_image_ptr->image;
            cv::Mat processed_image = color_image_ptr->image.clone();
            if (points_mat.cols() < 800000) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg<pcl::PointXYZ>(camera_data->point_cloud, *cloud);
                pixelPointMap(cloud);
            }
            for (const auto& [label, bbox] : yolov8_result) {
                cv::rectangle(processed_image, bbox, cv::Scalar(0, 0, 0));
                detection detect;
                detect.label = label;
                int x, y, w, h;
                x = bbox.x;
                y = bbox.y;
                w = bbox.width;
                h = bbox.height;
                if (x < 0) {
                    x = 0;
                }
                if (x > image.cols) {
                    x = image.cols - 1;
                }
                if (x + w > image.cols) {
                    w = image.cols - x;
                }
                if (y < 0) {
                    y = 0;
                }
                if (y > image.rows) {
                    y = image.rows - 1;
                }
                if (y + h > image.rows) {
                    h = image.rows - y;
                }

                cv::Rect roi(x, y, w, h);
                cv::Mat clip = image(roi);
                std::vector<cv::Point2f> temp;
                temp.emplace_back(bbox.x, bbox.y);
                temp.emplace_back(bbox.x + bbox.width, bbox.y);
                temp.emplace_back(bbox.x + bbox.width, bbox.y + bbox.height);
                temp.emplace_back(bbox.x, bbox.y + bbox.height);
                detect.bbox.swap(temp);
                getContours(clip, detect.contours, x, y);
                if (!(detect.contours[0].empty() && detect.contours[1].empty()))
                    detects.push_back(detect);
            }
            info.push_back(detects);
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "processed_image";
        sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_bridge::CvImage(header, "rgb8", processed_image).toImageMsg();
        m_processed_image_publisher->publish(*processed_image_msg);

        header.stamp = this->now();
        header.frame_id = "color_depth_image";
        sensor_msgs::msg::Image::SharedPtr color_depth_image_msg = cv_bridge::CvImage(header, "rgb8", color_depth_image).toImageMsg();
        m_color_depth_image_publisher->publish(*color_depth_image_msg);
    }
    catch (const cv_bridge::Exception& e) {
        m_logger->publish(vision_utils::create_log(this->now(), vision_utils::LogLevel::ERROR, e.what()));
    }
    catch (const cv::Exception& e) {
        m_logger->publish(vision_utils::create_log(this->now(), vision_utils::LogLevel::ERROR, e.what()));
    }
}

double Measure::calcGoalLen(const cv::Point3f& point1, const cv::Point3f& point2) {
    return (pow(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2), 0.5));
}

bool Measure::calcFeatPoint(const cv::Point2f& point, int type, int method, cv::Point3f& feat_point) {
    //type0为长度，1为直径，method0为遍历
    if (method == 0) {
        /*double min_square_distance = pow(point.x - pixels_mat(0, 0), 2)
                                     + pow(point.y - pixels_mat(1, 0), 2);
        int min_label = 0;
        for (int i = 0; i < pixels_mat.cols(); i++){
            double square_distance = pow(point.x - pixels_mat(0, i), 2)
                                     + pow(point.y - pixels_mat(1, i), 2);
            if (square_distance < min_square_distance){
                min_square_distance = square_distance;
                min_label = i;
            }
        }
        //test_start
        //std::cout << "min_label" << min_label << "\n";
        //std::cout << "min_distance" << min_square_distance << "\n";
        //test_end
        cv::Point3f feat_point;
        if (type == 0){
            feat_point.x = points_mat(0, min_label);
            feat_point.y = points_mat(1, min_label);
            feat_point.z = points_mat(2, min_label);
            return feat_point;
        }
        if (type == 1){
            feat_point.x = points_mat(0, min_label);
            feat_point.y = points_mat(1, min_label);
            feat_point.z = points_mat(2, min_label);
            return feat_point;
        }*/
    }
    else if (method == 1)
    {
        /*cv::Point3f feat_point;
        int x = (int )point.x;
        int y = (int )point.y;
        int i = 0;
        int index = pix_mat.at<int>(y,x);
        while(true)
        {
            if(index < 0)
            {
                i++;
                index = pix_mat.at<int>(y+i,x+i);
            }
            else
            {
                break;
            }
        }
        feat_point.x = points_mat(0,index);
        feat_point.y = points_mat(1,index);
        feat_point.z = points_mat(2,index);
        return feat_point;*/
    }
    else {
        std::vector<Eigen::Vector3f> box;
        std::vector<Eigen::Vector3f> _box;
        Eigen::Vector3f pointd;
        int i1 = (int)point.x / 4;
        int i2 = (int)point.y / 4;
        box = pro_points[i1][i2];
        if (i2 - 1 >= 0) {
            _box = pro_points[i1][i2 - 1];
            box.insert(box.end(), _box.begin(), _box.end());
        }
        if (i1 - 1 >= 0) {
            _box = pro_points[i1 - 1][i2];
            box.insert(box.end(), _box.begin(), _box.end());
        }
        if (i1 - 1 >= 0 && i2 - 1 >= 0) {
            _box = pro_points[i1 - 1][i2 - 1];
            box.insert(box.end(), _box.begin(), _box.end());
        }
        if (i2 + 1 < 480) {
            _box = pro_points[i1][i2 + 1];
            box.insert(box.end(), _box.begin(), _box.end());
        }
        if (i1 - 1 > 0 && i2 + 1 < 480) {
            _box = pro_points[i1 - 1][i2 + 1];
            box.insert(box.end(), _box.begin(), _box.end());
        }
        if (i1 + 1 < 640 && i2 + 1 < 480) {
            _box = pro_points[i1 + 1][i2 + 1];
            box.insert(box.end(), _box.begin(), _box.end());
        }
        if (i1 + 1 < 640) {
            _box = pro_points[i1 + 1][i2];
            box.insert(box.end(), _box.begin(), _box.end());
        }
        if (i1 + 1 < 640 && i2 - 1 > 0) {
            _box = pro_points[i1 + 1][i2 - 1];
            box.insert(box.end(), _box.begin(), _box.end());
        }
        if (box.empty())
            return false;
        pointd = box[0];
        double min_square_distance = pow(point.x - box[0](0, 0), 2)
            + pow(point.y - box[0](1, 0), 2);
        int min_label = 0;
        for (int i = 0; i < box.size(); i++) {
            double square_distance = pow(point.x - box[i](0, 0), 2)
                + pow(point.y - box[i](1, 0), 2);
            if (square_distance < min_square_distance) {
                min_square_distance = square_distance;
                min_label = i;
            }
        }
        int x = round(box[min_label](0, 0));
        int y = round(box[min_label](1, 0));
        /*pix_mat.at<int>(y,x) = -5;
        pix_mat.at<int>(y-1,x) = -5;
        pix_mat.at<int>(y,x-1) = -5;
        pix_mat.at<int>(y-1,x-1) = -5;*/

        int index = (int)box[min_label](2, 0);
        feat_point.x = points_mat(0, index);
        feat_point.y = points_mat(1, index);
        feat_point.z = points_mat(2, index);
        return true;
    }
}


void Measure::pixelPointMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pix_mat = cv::Mat(480, 640, CV_32SC1, -1);
    Eigen::Matrix3Xf pixels_mat;
    intrin_mat << intrin_fx, 0, intrin_cx, 0,
        0, intrin_fy, intrin_cy, 0,
        0, 0, 1, 0;
    Eigen::Vector4f point_vec;
    for (auto& point : cloud->points) {
        point_vec[0] = point.x;
        point_vec[1] = point.y;
        point_vec[2] = point.z;
        point_vec[3] = 1;
        points_mat.conservativeResize(points_mat.rows(), points_mat.cols() + 1);
        points_mat.col(points_mat.cols() - 1) = point_vec;
    }
    pixels_mat = intrin_mat * points_mat;
    int lost_cnt = 0;
    for (int i = 0; i < pixels_mat.cols(); i++) {
        pixels_mat.col(i) = pixels_mat.col(i) / pixels_mat(2, i);
        float x = pixels_mat(0, i);
        float y = pixels_mat(1, i);
        if (x > 0 && x < 640 && y>0 && y < 480) {
            int i1 = ((int)x) / 4;
            int i2 = ((int)y) / 4;
            int i3 = (int)round(x);
            int i4 = (int)round(y);
            pix_mat.at<int>(i4, i3) = i;
            pixels_mat.col(i)[2] = i;
            pro_points[i1][i2].push_back(pixels_mat.col(i));
        }
        else {
            lost_cnt++;
        }
    }
    std::cout << "g";
}
//#define d_SCREW
bool Measure::getGoals(detection& result, screw& s1) {
    using namespace std;
    using namespace cv;
    vector<Point2f> contour(result.contours[0]); //contorus[0] outline; contours[1] inline;
    vector<Point> contour_test;
    vector<Point> contour_test1;
    vector<vector<Point>> cont;
    for (auto pt : contour)
    {
        contour_test.push_back(Point((int)pt.x, (int)pt.y));
    }
    cont.push_back(contour_test);
    if (!result.contours[1].empty())
    {
        for (auto pt : result.contours[1])
        {
            contour_test1.push_back(Point((int)pt.x, (int)pt.y));
        }
        cont.push_back(contour_test1);
    }
    cv::Mat mat(480, 640, CV_8UC1, 1);
    cv::drawContours(mat, cont, -1, 255, 1);
    int sz = (int)(contour.size());
    if (contour.empty())
    {
        return false;
    }
    cv::Mat data_pts(sz, 2, CV_32FC1);
    auto* _data_pts = (float*)data_pts.data;
    for (int i = 0; i < data_pts.rows; i++, _data_pts += 2) {
        _data_pts[0] = (float)contour[i].x;
        _data_pts[1] = (float)contour[i].y;
    }
    cv::PCA pca;
    try
    {
        cv::PCA _pca(data_pts, cv::Mat(), PCA::DATA_AS_ROW);
        pca = _pca;
    }
    catch (cv::Exception& e)
    {
        std::cerr << "Failed to build pca" << " reason:" << e.what() << std::endl;
        return false;
    }
    Point2f dir1, dir2, center;
    center.x = (pca.mean.at<float>(0, 0));
    center.y = (pca.mean.at<float>(0, 1));
    dir1.x = (pca.eigenvectors.at<float>(0, 0));
    dir1.y = (pca.eigenvectors.at<float>(0, 1));
    dir2.x = (pca.eigenvectors.at<float>(1, 0));
    dir2.y = (pca.eigenvectors.at<float>(1, 1));
    dir2.y = (pca.eigenvectors.at<float>(1, 1));
    Mat eig = pca.eigenvectors;
    Mat data_ptss = pca.project(data_pts);
    s1.eig = eig;
    Point cent((result.bbox[0].x + result.bbox[2].x) / 2, (result.bbox[0].y + result.bbox[2].y) / 2);
    // if (dt == SQUIRE55)
    // {
    if (cent.x <= 1100 && cent.y <= 1100)
    {
        result.goalC = 1;
    }
    else if (cent.x > 1100 && cent.y <= 1100)
    {
        result.goalC = 2;
    }
    else if (cent.x >= 1100 && cent.y > 1100)
    {
        result.goalC = 3;
    }
    else if (cent.x < 1100 && cent.y>1100)
    {
        result.goalC = 4;
    }
    if (cent.x < -220 || cent.x>2420 || cent.y < -220 || cent.y>2420)
    {
        return false;
    }
    // }
    // else if (dt == CIRCLE60)
    // {
    //     double dis = sqrt(pow(cent.x - 1200, 2) + pow(cent.y - 1200, 2));
    //     if (dis < 400)
    //     {
    //         result.goalC = 5;
    //     }
    //     else if (dis < 800)
    //     {
    //         result.goalC = 6;
    //     }
    //     else
    //     {
    //         result.goalC = 7;
    //     }
    //     if (cent.x < -240 || cent.x>2640 || cent.y < -240 || cent.y>2640)
    //     {
    //         return false;
    //     }
    // }

    if (result.label == 0) {
        float goal_a, goal_b;
        cv::Mat test(640, 480, CV_8UC1, 1);
        float x_min = 10000, x_max = -10000, y_max = -10000, y_min = 10000;
        Point2f keypts[4];
        Point3f keypoints[4];
        for (int i = 0; i < data_ptss.rows; i++)
        {
            float x = data_ptss.at<float>(i, 0);
            if (x < x_min)
            {
                x_min = x;
                keypts[0] = Point2f(data_pts.at<float>(i, 0), data_pts.at<float>(i, 1));
            }
            if (x > x_max)
            {
                x_max = x;
                keypts[1] = Point2f(data_pts.at<float>(i, 0), data_pts.at<float>(i, 1));
            }
        }
        if (calcFeatPoint(keypts[0], 0, 2, keypoints[0]) &&
            calcFeatPoint(keypts[1], 0, 2, keypoints[1]))
            result.goalB = calcGoalLen(keypoints[0], keypoints[1]) * 1000;
        else
            result.goalB = x_max - x_min;
        goal_b = x_max - x_min;
        float x_last = data_ptss.at<float>(0, 0);
        float y_last = data_ptss.at<float>(0, 1);
        std::vector<cv::Point2f> p;
        for (int i = 1; i < data_ptss.rows; i++)
        {
            float x = data_ptss.at<float>(i, 0);
            float y = data_ptss.at<float>(i, 1);
            if (x_last <= 0 && x > 0)
            {
                p.push_back(Point2f(x, y));
                p.push_back(Point2f(x_last, y_last));
                //keypts[2] = Point2f(data_pts.at<float>(i,0),data_pts.at<float>(i,1));
            }
            if (x_last >= 0 && x < 0)
            {
                p.push_back(Point2f(x, y));
                p.push_back(Point2f(x_last, y_last));
                //keypts[3] = Point2f(data_pts.at<float>(i,0),data_pts.at<float>(i,1));
            }

            x_last = x;
            y_last = y;
        }

        /*float min_dis = 10000;
        Point2f d;
        for(auto pt : result.contours[0])
        {
            float dis = sqrt(pow(center.x - pt.x,2)+pow(center.y -pt.y,2));
            if(dis <min_dis)
            {
                min_dis = dis;
                d=pt;
            }
        }*/
        float rid = 0;
        for (auto pt : p)
        {
            rid = rid + abs(pt.y);
        }
        rid = rid / 4;
        keypts[2] = center + rid * dir2;
        keypts[3] = center - rid * dir2;
        for (int i = 0; i < 4;i++)
            cv::circle(mat, keypts[i], 2, 255, 1);
        if (calcFeatPoint(keypts[2], 1, 2, keypoints[2]) &&
            calcFeatPoint(keypts[3], 1, 2, keypoints[3]))
            result.goalA = calcGoalLen(keypoints[2], keypoints[3]) * 1000;
        else
            result.goalA = rid * 2;
        goal_a = rid * 2;
        if ((result.goalA * goal_b) / (result.goalB * goal_a) > 1.5) {
            result.goalA = rid * 2 * 1.2;
            if (result.goalA < 2)
                result.goalA = 2;
        }
        //cout<<"1";
        /*double amin, amax, bmax, bmin;
        bmin = data_ptss.at<double>(0, 0);
        bmax = data_ptss.at<double>(0, 0);

        for (int i = 1; i < data_ptss.rows; i++) {
            if (data_ptss.at<double>(i, 0) < bmin) {
                bmin = data_ptss.at<double>(i, 0);
            }
            if (data_ptss.at<double>(i, 0) > bmax) {
                bmax = data_ptss.at<double>(i, 0);
            }
        }
        result.goalB = bmax - bmin;

        const int divide = 10;
        vector<double> amax_p(divide);
        vector<double> amin_p(divide);
        double tmp = (result.goalB + 1) / divide;
        for (int i = 1; i < data_ptss.rows; i++) {
            int index = (int)((data_ptss.at<double>(i, 0) - bmin) / tmp);
            if (data_ptss.at<double>(i, 1) > amax_p[index]) {
                amax_p[index] = data_ptss.at<double>(i, 1);
            }
            if (data_ptss.at<double>(i, 1) < amin_p[index]) {
                amin_p[index] = data_ptss.at<double>(i, 1);
            }
        }
        sort(amax_p.begin(), amax_p.end());
        sort(amin_p.begin(), amin_p.end());
        amax = amax_p[(int)(divide / 2)];
        amin = amin_p[(int)(divide / 2)];
        result.goalA = amax - amin;
        double maxLength = maxEdge(result.bbox);
        if (result.goalA < maxLength * 0.85) {
            result.goalA = maxLength * 0.85;
        }*/
    }
    else {
        float x_min = 10000, x_max = -10000;
        Point2f keypts[4];
        Point3f keypoints[4];
        for (int i = 0; i < data_ptss.rows; i++)
        {
            float x = data_ptss.at<float>(i, 0);
            if (x <= x_min)
            {
                x_min = x;
                keypts[0] = Point2f(data_pts.at<float>(i, 0), data_pts.at<float>(i, 1));
            }
            if (x >= x_max)
            {
                x_max = x;
                keypts[1] = Point2f(data_pts.at<float>(i, 0), data_pts.at<float>(i, 1));
            }
        }
        if (calcFeatPoint(keypts[0], 1, 2, keypoints[0]) &&
            calcFeatPoint(keypts[1], 1, 2, keypoints[1]))
            result.goalA = calcGoalLen(keypoints[0], keypoints[1]) * 1000;
        else
            result.goalA = x_max - x_min;
        if (!result.contours[1].empty()) {
            float max_range = -999999;
            float min_range = 999999;
            for (int i = 0; i < result.contours[1].size(); i++) {
                Point2f pt(result.contours[1][i]);
                Point2f p = pt - center;
                float x = p.x * dir1.x + p.y * dir1.y;//dot(x,dir)=x*y*cos(theta), project pix on dir
                if (x < min_range) {
                    min_range = x;
                    keypts[2] = pt;
                }
                if (x > max_range) {
                    max_range = x;
                    keypts[3] = pt;
                }
            }
            for (int i = 0; i < 4; i++)
                cv::circle(mat, keypts[i], 2, 100, 1);
            if (calcFeatPoint(keypts[2], 1, 2, keypoints[2]) &&
                calcFeatPoint(keypts[3], 1, 2, keypoints[3]))
                result.goalB = calcGoalLen(keypoints[2], keypoints[3]) * 1000;
            else
                result.goalB = max_range - min_range;
        }
        else
        {
            result.goalB = result.goalA * 0.6;
        }


    }
    return true;
}

void Measure::calculateGoals(detections& res, cv::Mat warpmatrix) {
    using namespace std;
    using namespace cv;
    if (!res.empty()) {
        vector<screw> screws;
        for (int k = 0; k < res.size(); k++) {
            detection& result = res[k];
            screw s1;
            s1.index = k;
            bool ok = getGoals(result, s1);
            if (result.label == 0 && ok) {
                screws.push_back(s1);
            }
        }
#ifdef ADD_SCREW
        cv::Mat eigCompare((int)screws.size(), (int)screws.size(), CV_64FC1, MEASURE_MIN_ANGLE);
        for (int i = 0; i + 1 < screws.size(); i++) {
            for (int j = i + 1; j < screws.size(); j++) {
                double angle = cv::norm(screws[i].eig - screws[j].eig);
                // 解除注释允许90度夹角螺丝匹配
//                if (angle > 1.14) {
//                    angle -= 1.414;
//                }
                eigCompare.at<double>(i, j) = angle;

            }
        }
        int n = (int)screws.size();
        while (n--) {
            double minValue;
            cv::Point minLoc;
            cv::minMaxLoc(eigCompare, &minValue, nullptr, &minLoc, nullptr);
            if (minValue >= MEASURE_MIN_ANGLE) {
                break;
            }
            double shortest = getShortest(res[screws[minLoc.x].index].contours, res[screws[minLoc.y].index].contours);
            if (shortest < MEASURE_MIN_SCREW_LEN) {
                for (int i = 0; i < eigCompare.cols; i++) {
                    eigCompare.at<double>(minLoc.x, i) = MEASURE_MIN_ANGLE;
                    eigCompare.at<double>(i, minLoc.y) = MEASURE_MIN_ANGLE;
                }
                res[screws[minLoc.x].index].goalA += res[screws[minLoc.y].index].goalA + shortest;
                res[screws[minLoc.x].index].goalB += res[screws[minLoc.y].index].goalB;
                res[screws[minLoc.x].index].goalB /= 2;
                res[screws[minLoc.x].index].score = std::max(res[screws[minLoc.x].index].score, res[screws[minLoc.y].index].score);
                res[screws[minLoc.y].index].score = 0;
            }
        }
#endif
    }
}

void getContours(const cv::Mat& image, std::vector<cv::Point2f>* _contours, double x, double y) {
    cv::Mat gray_img;
    cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);
    cv::Scalar mea = cv::mean(gray_img);
    cv::threshold(gray_img, gray_img, mea[0], 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(gray_img, gray_img, cv::MORPH_OPEN, kernel1);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(gray_img, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(x, y));  //提取轮廓
    /*cv::Mat test(640,640,CV_8UC1,1);
    cv::drawContours(test,contours,-1,cv::Scalar(255),1);*/
    if (contours.empty()) {
        return;
    }
    double min_dis = 10000;
    int id = 0;
    cv::Point cont_mid(0, 0);
    cv::Point cent(x + gray_img.cols / 2, y + gray_img.rows / 2);
    for (size_t i = 0; i < contours.size(); i++)
    {
        for (auto p : contours[i])
        {
            cont_mid = cont_mid + p;
        }
        cont_mid = cont_mid / (float)contours[i].size();
        double dis = pow(cont_mid.x - cent.x, 2) + pow(cont_mid.y - cent.y, 2);
        dis = sqrt(dis);
        if (dis < min_dis) {
            id = (int)i;
            min_dis = dis;
        }
    }
    if (hierarchy[id][2] == -1 && hierarchy[id][3] == -1)
    {
        std::vector<cv::Point2f> contour;
        for (auto p : contours[id])
        {
            contour.emplace_back(cv::Point2f(p.x, p.y));
        }
        _contours[0] = contour;
    }
    else if (hierarchy[id][2] != -1)
    {
        int in_id = hierarchy[id][2];
        std::vector<cv::Point2f> contour;
        for (auto p : contours[id])
        {
            contour.emplace_back(cv::Point2f(p.x, p.y));
        }
        _contours[0] = contour;
        contour.clear();
        for (auto p : contours[in_id])
        {
            contour.emplace_back(cv::Point2f(p.x, p.y));
        }
        _contours[1] = contour;
    }
    else if (hierarchy[id][3] != -1)
    {
        int out_id = hierarchy[id][3];
        std::vector<cv::Point2f> contour;
        for (auto p : contours[out_id])
        {
            contour.emplace_back(cv::Point2f(p.x, p.y));
        }
        _contours[0] = contour;
        contour.clear();
        for (auto p : contours[id])
        {
            contour.emplace_back(cv::Point2f(p.x, p.y));
        }
        _contours[1] = contour;
    }
}

void Measure::calcResult() {
    calAvgPoints();
    //    cv::Mat test(640, 480, CV_8UC1, 0);
    //    cv::Mat test = cv::Mat::zeros(640, 480, CV_8UC1);
    //    std::vector<cv::Point2f> pps;
    //    for (const auto& it: res) {
    //        pps.emplace_back(it.x, it.y);
    //    }
    //    cv::fillPoly(test, pps, cv::Scalar(255));
    //    cv::imshow("test", test);
    //    cv::waitKey(1);
    std::vector<cv::Point2f> src, dst;
    for (const auto& it : res) {
        src.emplace_back(it.x, it.y);
    }
    // if (dt == SQUIRE55) {
    dst = std::vector<cv::Point2f>{ {0, 0}, {2200, 0}, {2200, 2200}, {0, 2200} };
    // }
    // else if (dt == CIRCLE60) {
    //     dst = std::vector<cv::Point2f>{ {0, 1200}, {1200, 0}, {2400, 1200}, {1200, 2400} };
    // }
    cv::Mat warpMatrix = getPerspectiveTransform(src, dst);
    int kt = 0;
    for (auto& detect : info) {
        for (auto& it : detect) {
            std::vector<cv::Point2f> temp(it.contours[0]);
            std::vector<cv::Point2f> temp2;
            //cv::perspectiveTransform(it.contours[0], temp, warpMatrix);
            cv::perspectiveTransform(it.bbox, temp2, warpMatrix);
            // 获取从源点到目标点的透视变换矩阵
            // 该矩阵用于描述从一个平面到另一个平面的映射关系
            // 参数src: 源图像中的四个顶点坐标
            // 参数dst: 目标图像中对应的四个顶点坐标
            // 返回值: 一个3x3的矩阵，表示源点到目标点的透视变换
            it.contours[0].swap(temp);
            it.bbox.swap(temp2);
            /*if(!it.contours[1].empty()) {
                std::vector<cv::Point2f> temp1(it.contours[1]);
                cv::perspectiveTransform(it.contours[1], temp1, warpMatrix);
                it.contours[1].swap(temp1);
            }*/
        }
        calculateGoals(detect, warpMatrix);
        if (result.empty()) {
            result = detect;
        }
        else {
            cv::Mat comp((int)result.size(), (int)detect.size(), CV_32FC1, MEASURE_MIN_GOAL_LEN);
            for (int i = 0; i < result.size(); i++) {
                for (int j = 0; j < detect.size(); j++) {
                    comp.at<float>(i, j) = (float)getGoalLen(result[i], detect[j]);
                }
            }
            int n = std::min((int)result.size(), (int)detect.size());
            while (n--) {
                double minValue;
                cv::Point minPoint;
                cv::minMaxLoc(comp, &minValue, nullptr, &minPoint, nullptr);
                if (minValue >= MEASURE_MIN_GOAL_LEN) {
                    break;
                }
                else {
                    for (int i = 0; i < comp.rows; i++) {
                        comp.at<float>(i, minPoint.y) = MEASURE_MIN_GOAL_LEN;
                    }
                    for (int i = 0; i < comp.cols; i++) {
                        comp.at<float>(minPoint.x, i) = MEASURE_MIN_GOAL_LEN;
                    }
                    result[minPoint.x].score = std::max(result[minPoint.x].score, detect[minPoint.y].score);
                }
            }
            for (int j = 0; j < comp.cols; j++) {
                float minValue;
                for (int i = 0; i < comp.rows; i++) {
                    if (comp.at<float>(i, j) < minValue) {
                        minValue = comp.at<float>(i, j);
                        minValue = comp.at<float>(i, j);
                    }
                }
                if (minValue > MEASURE_MIN_GOAL_LEN) {
                    result.push_back(detect[j]);
                }
            }
        }
        kt++;
        std::cout << "measure finished:" << kt << "/" << (int)info.size() << std::endl;
    }
    std::sort(result.begin(), result.end(), compareScore);
}

double Measure::getGoalLen(const Measure::detection& d1, const Measure::detection& d2) {
    return sqrt(pow(d1.goalB - d2.goalB, 2));
}

bool Measure::compareScore(const Measure::detection& d1, const Measure::detection& d2) {
    return d1.score > d2.score;
}

double Measure::getShortest(const std::vector<cv::Point2f>& c1, const std::vector<cv::Point2f>& c2) {
    using point_t = boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian>;
    using polygon_t = boost::geometry::model::polygon<point_t>;
    polygon_t poly1, poly2;
    for (const auto& it : c1) {
        poly1.outer().push_back(point_t(it.x, it.y));
    }
    for (const auto& it : c2) {
        poly2.outer().push_back(point_t(it.x, it.y));
    }
    return boost::geometry::distance(poly1, poly2);
}

inline double getLength(const cv::Point2f& p1, const cv::Point2f& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double Measure::maxEdge(const std::vector<cv::Point2f>& ps) {
    double l1 = getLength(ps[0], ps[1]);
    double l2 = getLength(ps[1], ps[2]);
    double l3 = getLength(ps[2], ps[3]);
    double l4 = getLength(ps[3], ps[0]);
    double l13 = (l1 + l3) / 2;
    double l24 = (l2 + l4) / 2;
    return fmax(l13, l24);
}

void Measure::clear() {
    clearCalibrate();
    info.clear();
}

std::string Measure::getResult() {
    std::stringstream ss;
    ss << "START" << std::endl << std::setiosflags(std::ios::fixed) << std::setprecision(1);
    for (const auto& it : result) {
        if (it.goalA < 0.1 && it.goalB < 0.1) {
            continue;
        }
        ss << "Goal_ID=" << it.label + 1 << ";";
        ss << "Goal_A=" << it.goalA << ";";
        ss << "Goal_B=" << it.goalB << ";";
        ss << "Goal_C=" << it.goalC << std::endl;
    }
    ss << "END" << std::endl;
    return ss.str();
}

void Measure::calAvgPoints() {
    res.clear();
    if (!pps[0].empty()) {
        for (int i = 0; i < 4; i++) {
            float xSum = 0, ySum = 0;
            int count = 0;
            for (const auto& it : pps[i]) {
                xSum += it.x;
                ySum += it.y;
                count++;
            }
            res.emplace_back(xSum / (float)count, ySum / (float)count);
        }
    }
    else {
        res.emplace_back(168, 99);
        res.emplace_back(472, 99);
        res.emplace_back(538, 356);
        res.emplace_back(118, 356);
    }
}

void Measure::addPoints(const points_t& points) {
    pps[0].push_back(points[0]);
    pps[1].push_back(points[1]);
    pps[2].push_back(points[2]);
    pps[3].push_back(points[3]);
}

void Measure::clearCalibrate() {
    pps.clear();
}

bool Measure::empty() {
    return pps.empty();
}

Measure::~Measure() {
    finalize();
}