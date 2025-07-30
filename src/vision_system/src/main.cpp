#include <QApplication>
#include "vision_system.hpp"

int main(int argc, char** argv) {
    YAML::Node config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("vision_utils") + "/config/config.yaml");
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto system_node = std::make_shared<SystemNode>();
    VisionSystem vision_system(system_node);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(system_node);
    std::thread spin_thread([executor]() {executor->spin();});
    vision_system.run();
    int result = app.exec();
    rclcpp::shutdown();
    if(spin_thread.joinable()){
        spin_thread.join();
    }
    return result;
}