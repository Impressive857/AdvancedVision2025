#ifndef _MODE_BASE_HPP_
#define _MODE_BASE_HPP_

// std
#include <vector>

// ros
#include "ros_msg/msg/calculation_data.hpp"

//
#include "vision_utils.hpp"

class ModeBase {
public:
    ModeBase() = default;
    virtual void init() = 0;
    virtual void add_calculation_data(const ros_msg::msg::CalculationData& calculation_data) = 0;
    virtual void calculate_result() = 0;
    virtual std::string get_result() = 0;
    virtual ~ModeBase() = default;
protected:
    std::string m_result;
};

#endif // !_MODE_BASE_HPP_