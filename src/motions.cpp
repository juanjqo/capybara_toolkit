#include "capybara/conversions.hpp"
#include <dqcapybara/motions.hpp>


namespace Capybara {

DQ rot(const double &angle, const DQ &axis)
{
    if (!is_unit(axis) or !is_quaternion(axis))
        throw std::runtime_error("Panic with Capybara::rot(angle, axis). The axis must be a unit quaternion.");

    return cos(angle/2)+axis*sin(angle/2);
}

DQ rotx(const double &angle_deg)
{
    return Capybara::rot(Capybara::Conversions::deg2rad(angle_deg), i_);
}

DQ roty(const double &angle_deg)
{
    return Capybara::rot(Capybara::Conversions::deg2rad(angle_deg), j_);
}

DQ rotz(const double &angle_deg)
{
    return Capybara::rot(Capybara::Conversions::deg2rad(angle_deg), k_);
}

}
