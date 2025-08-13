#include "measure.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto measure_node = std::make_shared<Measure>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(measure_node);
    measure_node->run();
    executor->spin();
    rclcpp::shutdown();
};