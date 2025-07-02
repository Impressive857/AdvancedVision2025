#ifndef _NN_CALCULATOR_HPP_
#define _NN_CALCULATOR_HPP_

// ros
#include <rclcpp/rclcpp.hpp>

//
#include "identify_mode.hpp"
#include "measure_mode.hpp"
#include "nn_detector.hpp"

class NnCalculator
    : public rclcpp::Node
{
public:
    NnCalculator();
    ~NnCalculator();
private:
    ModeBase* m_current_mode = nullptr;
    NnDetector m_nn_detector;
};

#endif // !_NN_CALCULATOR_HPP_