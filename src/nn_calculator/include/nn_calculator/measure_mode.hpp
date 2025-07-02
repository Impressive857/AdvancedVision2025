#ifndef _MEASURE_MODE_HPP_
#define _MEASURE_MODE_HPP_

#include "mode_base.hpp"

class MeasureMode
    :public ModeBase
{
public:
    MeasureMode() = default;
    void init() override;
    void add_calculation_data(const ros_msg::msg::CalculationData& calculation_data) override;
    void calculate_result() override;
    const std::string& get_result() override;
    ~MeasureMode() override;
private:
};

#endif // !_MEASURE_MODE_HPP_