#ifndef _IDENTIFY_MODE_HPP_
#define _IDENTIFY_MODE_HPP_

#include "mode_base.hpp"

class IdentifyMode
    :public ModeBase
{

public:
    IdentifyMode() = default;
    void init() override;
    void add_calculation_data(const ros_msg::msg::CalculationData& calculation_data) override;
    void calculate_result() override;
    std::string get_result() override;
    ~IdentifyMode() override;
private:
};

#endif // !_IDENTIFY_MODE_HPP_