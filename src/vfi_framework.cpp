#include <capybara/experimental/vfi_framework.hpp>

namespace Capybara {
VFI_Framework::DIRECTION VFI_Framework::map_string_to_direction(const std::string &str)
{
    DIRECTION direction;
    if (str == std::string("KEEP_ROBOT_OUTSIDE"))
    {
        direction = DIRECTION::KEEP_ROBOT_OUTSIDE;
    }
    else if  (str == std::string("KEEP_ROBOT_INSIDE"))
    {
        direction = DIRECTION::KEEP_ROBOT_INSIDE;
    }
    else
    {
        throw std::runtime_error("Error in map_string_to_direction. The argument: " + str + std::string(" is not valid. ")+
                                 "You are required to use: " + "KEEP_ROBOT_OUTSIDE or KEEP_ROBOT_INSIDE");
    }
    return direction;
}

VFI_Framework::VFI_TYPE VFI_Framework::map_strings_to_vfiType(const std::string &entity_robot_primitive_type,
                                                          const std::string &entity_enviroment_primitive_type)
{
    auto robot_primitive = map_string_to_primitive(entity_robot_primitive_type);
    auto environment_primitive = map_string_to_primitive(entity_enviroment_primitive_type);
    VFI_TYPE vfy_type;

    switch (robot_primitive ) {
    case PRIMITIVE::POINT:
        switch (environment_primitive) {
        case PRIMITIVE::POINT:
            vfy_type = VFI_TYPE::RPOINT_TO_POINT;
            break;
        case PRIMITIVE::LINE:
            vfy_type = VFI_TYPE::RPOINT_TO_LINE;
            break;
        case PRIMITIVE::PLANE:
            vfy_type = VFI_TYPE::RPOINT_TO_PLANE;
            break;
        default:
            throw std::runtime_error("Error in map_string_to_vfiType. Combination of primitives "
                                     + map_primitive_to_string(robot_primitive) + "-"
                                     + map_primitive_to_string(environment_primitive)
                                     + "not supported.");
            break;
        }
        break;
    case PRIMITIVE::LINE_ANGLE:
        if (environment_primitive == PRIMITIVE::LINE_ANGLE)
        {
            vfy_type = VFI_TYPE::RLINE_TO_LINE_ANGLE;
        }else
        {
            throw std::runtime_error("Error in map_string_to_vfiType. Both primitives are required to be LINE_ANGLE for "
                                     "RLINE_TO_LINE_ANGLE.");
        }
        break;
    case PRIMITIVE::LINE:
        switch (environment_primitive) {
        case PRIMITIVE::POINT:
            vfy_type = VFI_TYPE::RLINE_TO_POINT;
            break;
        case PRIMITIVE::LINE:
            vfy_type = VFI_TYPE::RLINE_TO_LINE;
            break;
        default:
            throw std::runtime_error("Error in map_string_to_vfiType. Combination of primitives "
                                     + map_primitive_to_string(robot_primitive) + "-"
                                     + map_primitive_to_string(environment_primitive)
                                     + "not supported.");
            break;
        }
        break;
    default:
        throw std::runtime_error("Error in map_string_to_vfiType. Combination of primitives "
                                 + map_primitive_to_string(robot_primitive) + "-"
                                 + map_primitive_to_string(environment_primitive)
                                 + "not supported.");
        break;
    }
    return vfy_type;

}

std::string VFI_Framework::map_vfyType_to_string(const VFI_TYPE &vfi_type)
{
    std::string str;
    switch (vfi_type) {
    case VFI_TYPE::RPOINT_TO_POINT:
        str = std::string("RPOINT_TO_POINT");
        break;
    case VFI_TYPE::RPOINT_TO_LINE:
        str = std::string("RPOINT_TO_LINE");
        break;
    case VFI_TYPE::RPOINT_TO_PLANE:
        str = std::string("RPOINT_TO_PLANE");
        break;
    case VFI_TYPE::RLINE_TO_LINE_ANGLE:
        str = std::string("RLINE_TO_LINE_ANGLE");
        break;
    case VFI_TYPE::RLINE_TO_LINE:
        str = std::string("RLINE_TO_LINE");
        break;
    case VFI_TYPE::RLINE_TO_POINT:
        str = std::string("RLINE_TO_POINT");
    }
    return str;
}

std::string VFI_Framework::map_primitive_to_string(const PRIMITIVE &primitive)
{
    std::string str;
    switch (primitive) {
    case PRIMITIVE::POINT:
        str = std::string("POINT");
        break;
    case PRIMITIVE::LINE:
        str = std::string("LINE");
        break;
    case PRIMITIVE::PLANE:
        str = std::string("PLANE");
        break;
    case PRIMITIVE::LINE_ANGLE:
        str = std::string("LINE_ANGLE");
        break;
    }
    return str;

}

DQ VFI_Framework::map_attached_direction_string_to_dq(const std::string &str)
{
    DQ direction = 1.0*k_;

    if (str == std::string("k_"))
    {
        direction = k_;
    }
    else if (str == std::string("-k_"))
    {
        direction = -1*k_;
    }
    else if (str == std::string("j_"))
    {
        direction = j_;
    }
    else if (str == std::string("-j_"))
    {
        direction = -1*j_;
    }
    else if (str == std::string("i_"))
    {
        direction = i_;
    }
    else if (str == std::string("-i_"))
    {
        direction = -1*i_;
    }
    else
    {
        throw std::runtime_error("Error in map_attached_direction_string_to_dq. "
                                 "The argument: " + str + std::string(" is not valid. ")+
                                 "Valid options: " + "i_, j_, k_, -i_, -j_ or -k_ ");
    }

    return direction;

}

VFI_Framework::PRIMITIVE VFI_Framework::map_string_to_primitive(const std::string &str)
{
    PRIMITIVE primitive;
    if (str == std::string("LINE"))
    {
        primitive = PRIMITIVE::LINE;
    }
    else if (str == std::string("POINT"))
    {
        primitive = PRIMITIVE::POINT;
    }
    else if (str == std::string("PLANE"))
    {
        primitive = PRIMITIVE::PLANE;
    }
    else if (str == std::string("LINE_ANGLE"))
    {
        primitive = PRIMITIVE::LINE_ANGLE;
    }
    else
    {
        throw std::runtime_error("Error in map_string_to_primitive. The argument: " + str + std::string(" is not valid"));
    }
    return primitive;
}


}
