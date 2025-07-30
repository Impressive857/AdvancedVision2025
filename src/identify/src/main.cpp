#include "identify.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto identify_node = std::make_shared<Identify>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(identify_node);
    identify_node->run();
    executor->spin();
    rclcpp::shutdown();
    return 0;
}