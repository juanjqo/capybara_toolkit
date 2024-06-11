#include "capybara/conversions.hpp"
#include <capybara/motions.hpp>


namespace Capybara {

DQ rot(const double &angle, const DQ &axis)
{
    return cos(angle/2)+axis*sin(angle/2);
}

DQ rotx(const double &angle_deg)
{
    return Capybara::rot(Capybara::Conversions::deg2rad(angle_deg), i_);
}

}
