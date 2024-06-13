
#pragma once
#include <dqrobotics/DQ.h>
#include <capybara/conversions.hpp>

using namespace  DQ_robotics;
using namespace Eigen;

namespace Capybara {

    DQ rot(const double& angle, const DQ& axis);
    DQ rotx(const double& angle_deg);
    DQ roty(const double& angle_deg);
    DQ rotz(const double& angle_deg);
    DQ translate(const DQ& translation);
    DQ translate_xyz(const double& x, const double& y, const double& z);
    DQ translate_x(const double& x);
    DQ translate_y(const double& y);
    DQ translate_z(const double& z);
}

