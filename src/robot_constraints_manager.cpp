#include <capybara/experimental/robot_constraints_manager.hpp>

namespace Capybara {


RobotConstraintsManager::RobotConstraintsManager(const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental> &coppelia_interface,
                                                 const std::shared_ptr<DQ_Kinematics> &robot,
                                                 const std::shared_ptr<DQ_CoppeliaSimRobot> &coppelia_robot, const VectorXd &q_min, const VectorXd &q_max, const VectorXd &q_min_dot, const VectorXd &q_max_dot,
                                                 const std::string &config_path,
                                                 const VFI_manager::LEVEL &level)
    :vi_{coppelia_interface}, config_path_{config_path}, level_{level}, robot_{robot}, coppelia_robot_{coppelia_robot},
    q_max_{q_max}, q_min_{q_min}, q_min_dot_{q_min_dot}, q_max_dot_{q_max_dot}
{
    VFI_M_ = std::make_shared<Capybara::VFI_manager>(robot->get_dim_configuration_space());
    _initial_settings();
}

DQ RobotConstraintsManager::_get_robot_primitive_offset_from_coppeliasim(const std::string &object_name, const int &joint_index)
{
    DQ x;
    DQ x_offset;
    DQ xprimitive;
    //std::cout<<"Object name: "<<object_name<<std::endl;
    for (int i=0;i<5;i++)
    {
        VectorXd q = coppelia_robot_->get_configuration_space();
        x = robot_->fkm(q, joint_index);
        xprimitive = vi_->get_object_pose(object_name);
        x_offset =  x.conj()*xprimitive;
    }
    return x_offset;

}

void RobotConstraintsManager::_initial_settings()
{
    try {
        YAML::Node config = YAML::LoadFile(config_path_);
        //const int size = config.size();
        std::cout << "----------------------------------------------" <<std::endl;
        std::cout << "Config file path: " << config_path_ <<std::endl;
        std::cout << "Constraints found in the config file: " << config.size() <<std::endl;
        std::cout << "----------------------------------------------" <<std::endl;
        //----------------------------------------------------------------------------------

        int i = 0;
        // The outer element is an array
        for(auto dict : config) {
            // The array element is a map containing the Description and Parameters keys:
            auto name = dict["Description"];
            //std::cout << "       " <<std::endl;
            //std::cout<<"Constraint: "<<i+1<<std::endl;
            //std::cout <<"Description: " << name << std::endl;
            auto rect = dict["Parameters"];

            for(auto pos : rect) {
                auto raw_vfi_mode = pos["vfi_mode"].as<std::string>();

                if (raw_vfi_mode == "ENVIRONMENT_TO_ROBOT")
                {
                    auto raw_cs_entity_environment = pos["cs_entity_environment"].as<std::string>();
                    auto raw_cs_entity_robot = pos["cs_entity_robot"].as<std::string>() ;
                    auto raw_entity_environment_primitive_type =  pos["entity_environment_primitive_type"].as<std::string>();
                    auto raw_entity_robot_primitive_type = pos["entity_robot_primitive_type"].as<std::string>();
                    //auto raw_robot_index = pos["robot_index"].as<double>();
                    auto raw_joint_index =  pos["joint_index"].as<double>();
                    auto raw_safe_distance = pos["safe_distance"].as<double>();
                    auto raw_direction =  pos["direction"].as<std::string>();
                    auto raw_entity_robot_attached_direction = pos["entity_robot_attached_direction"].as<std::string>();
                    auto raw_entity_environment_attached_direction = pos["entity_environment_attached_direction"].as<std::string>();

                    vfi_mode_list_.push_back(VFI_manager::VFI_MODE::ENVIRONMENT_TO_ROBOT);
                    vfi_type_list_.     push_back(VFI_Framework::map_strings_to_vfiType(raw_entity_robot_primitive_type,
                                                                                        raw_entity_environment_primitive_type));
                    direction_list_.    push_back(VFI_Framework::map_string_to_direction(raw_direction));
                    safe_distance_list_.push_back(raw_safe_distance);
                    joint_index_list_.  push_back(raw_joint_index);
                    dq_offset_list_.    push_back(_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_robot,
                                                                                               raw_joint_index));

                    robot_attached_dir_list_.push_back(VFI_Framework::map_attached_direction_string_to_dq(raw_entity_robot_attached_direction));
                    envir_attached_dir_list_.push_back(VFI_Framework::map_attached_direction_string_to_dq(raw_entity_environment_attached_direction));
                    workspace_derivative_list_.push_back(DQ(0));
                    cs_entity_environment_list_.push_back(raw_cs_entity_environment);

                    std::cout << raw_vfi_mode << ",\t"
                              << raw_cs_entity_environment  << ",\t"
                              << raw_cs_entity_robot << ",\t"
                              << raw_entity_robot_primitive_type << ",\t"
                              << raw_entity_environment_primitive_type << ",\t"
                              << raw_joint_index  << ",\t"
                              << raw_safe_distance<< ",\t"
                              << raw_direction << ",\t"
                              << raw_entity_robot_attached_direction << ",\t"
                              << raw_entity_environment_attached_direction << std::endl;
                }else if (raw_vfi_mode == "ROBOT_TO_ROBOT"){

                    auto raw_cs_entity_one = pos["cs_entity_one"].as<std::string>();
                    auto raw_cs_entity_two = pos["cs_entity_two"].as<std::string>();

                    auto raw_entity_one_primitive_type =  pos["entity_one_primitive_type"].as<std::string>();
                    auto raw_entity_two_primitive_type=   pos["entity_two_primitive_type"].as<std::string>();

                    auto raw_joint_index_one =  pos["joint_index_one"].as<double>();
                    auto raw_joint_index_two =  pos["joint_index_two"].as<double>();

                    auto raw_safe_distance = pos["safe_distance"].as<double>();

                    vfi_mode_list_.push_back(VFI_manager::VFI_MODE::ROBOT_TO_ROBOT);

                    std::cout << raw_vfi_mode << ",\t"
                              << raw_cs_entity_one  << ",\t"
                              << raw_cs_entity_two << ",\t"
                              << raw_entity_one_primitive_type << ",\t"
                              << raw_entity_two_primitive_type << ",\t"
                              << raw_joint_index_one << ",\t"
                              << raw_joint_index_two << ",\t"
                              << raw_safe_distance << std::endl;


                }else{
                    throw std::runtime_error("Wrong vfi mode. USE ENVIRONMENT_TO_ROBOT or ROBOT_TO_ROBOT");
                }








                //std::cout<<"VFI to be used: "<<map_vfyType_to_strig(map_strings_to_vfiType(raw_entity_robot_primitive_type,
                //                                                                        raw_entity_environment_primitive_type))<<std::endl;


                i++;
            }
        }

        std::cout << "----------------------------------------------" <<std::endl;

    } catch(const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
        //return 1;
    } catch(const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        //return 1;
    }

}

}
