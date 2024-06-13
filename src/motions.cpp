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

DQ translate(const DQ &translation)
{
    if (!is_pure(translation) or !is_quaternion(translation))
        throw std::runtime_error("Panic with Capybara::translate(translation). The argument must be a pure quaternion.");
    return 1+0.5*E_*translation;
}

DQ translate_xyz(const double &x, const double &y, const double &z)
{
    return translate(x*i_+y*j_+z*k_);
}

DQ translate_x(const double &x)
{
    return translate(x*i_);
}

DQ translate_y(const double &y)
{
    return translate(y*j_);
}

DQ translate_z(const double &z)
{
    return translate(z*k_);
}

}
