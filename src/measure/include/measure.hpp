#ifndef _MEASURE_HPP_
#define _MEASURE_HPP_

// model
#include "yolov8.hpp"

// std
#include <unordered_map>
#include <vector>
#include <atomic>
#include <cmath>
#include <numeric>
#include <utility>

// ros
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "ros_msgs/msg/camera_data.hpp"
#include "ros_msgs/msg/log.hpp"

// yaml
#include <yaml-cpp/yaml.h>

//cv_bridge
#include <cv_bridge/cv_bridge.h>

// acl
#include "acl_utils.hpp"

// pcl
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/distances.h>

// eigen
#include <Eigen/Core>

// boost
#include <boost/geometry.hpp>

#define MEASURE_MIN_GOAL_LEN 20
#define MEASURE_MIN_SCREW_LEN 20
#define MEASURE_MIN_ANGLE 0.4       // 这是25度，计算公式: 2*sin(angle/2)

class Measure
    :public rclcpp::Node
{
public:
    struct point_t {
        float x = 0, y = 0;
        point_t(float a, float b) {
            x = a;
            y = b;
        }
    };
    using points_t = std::vector<point_t>;
    struct detection {
        std::vector<cv::Point2f> contours[2];
        std::vector<cv::Point2f> bbox;
        float score;
        int label;
        double goalA = -1;
        double goalB = -1;
        int goalC = 0;
        ~detection();
    };
    using detections = std::vector<detection>;
    struct screw {
        int index;
        cv::Mat eig;
    };
public:
    Measure();
    void run();
    ~Measure();
private:
    void init();
    void finalize();
    void addPoints(const points_t& points);
    bool empty();
    void calcResult();
    void clear();
    std::string getResult();
    void calCalibrate(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud);
    void clearCalibrate();
    void calAvgPoints();
    void pixelPointMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool calcFeatPoint(const cv::Point2f& point, int type, int method, cv::Point3f& feat_point);
    void calculateGoals(detections& res, cv::Mat warpmatrix);
    double getGoalLen(const detection& d1, const detection& d2);
    double calcGoalLen(const cv::Point3f& point1, const cv::Point3f& point2);
    bool getGoals(detection& result, screw& s1);
    double getShortest(const std::vector<cv::Point2f>& c1, const std::vector<cv::Point2f>& c2);
    double maxEdge(const std::vector<cv::Point2f>& ps);
    void camera_data_received_cbfn(const ros_msgs::msg::CameraData::ConstSharedPtr& camera_data);
    void result_timer_cbfn();
private:
    static bool compareScore(const detection& d1, const detection& d2);
private:
    std::vector<points_t> pps = std::vector<points_t>(4);
    points_t res;
    std::vector<detections> info;
    detections result;
    const float camera_factor = 5000;
    const float intrin_cx = 305.57;
    const float intrin_cy = 234.345;
    const float intrin_fx = 618.675;
    const float intrin_fy = 618.675;
    Eigen::Matrix<float, 3, 4> intrin_mat;
    cv::Mat pix_mat;
    std::vector<Eigen::Vector3f> pro_points[160][120];
    Eigen::Matrix4Xf points_mat;
    YoloV8 m_yolov8;
    rclcpp::Subscription<ros_msgs::msg::CameraData>::SharedPtr m_camera_data_subscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_result_text_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_processed_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_color_depth_image_publisher;
    rclcpp::Publisher<ros_msgs::msg::Log>::SharedPtr m_logger;
    rclcpp::TimerBase::SharedPtr m_result_timer;
    int m_current_round;
    std::atomic<bool> m_ready;
    float m_min_distance;
    float m_max_distance;
    YAML::Node m_config;
};

#endif // !_MEASURE_HPP_