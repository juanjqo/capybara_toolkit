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

std::tuple<MatrixXd, VectorXd> RobotConstraintsManager::get_inequality_constraints(const VectorXd &q)
{
    const int n = vfi_mode_list_.size();
    const int robot_dim = robot_->get_dim_configuration_space();

    for (int i = 0; i<n; i++)
    {
        if (vfi_mode_list_.at(i) == VFI_manager::VFI_MODE::ENVIRONMENT_TO_ROBOT)
        {
            DQ x = (robot_->fkm(q, joint_index_list_one_.at(i)))*dq_offset_list_one_.at(i);
            MatrixXd J = haminus8(dq_offset_list_one_.at(i))*robot_->pose_jacobian(q, joint_index_list_one_.at(i));
            if (J.cols() != robot_dim)
                J = Capybara::Numpy::resize(J, J.rows(), robot_dim);
            DQ x_workspace = cs_entity_environment_DQ_list_.at(i);

            VFI_M_->add_vfi_constraint(direction_list_.at(i),
                                       vfi_type_list_.at(i),
                                       safe_distance_list_.at(i),
                                       vfi_gain_,
                                       J,
                                       x,
                                       robot_attached_dir_list_.at(i),
                                       x_workspace,
                                       envir_attached_dir_list_.at(i),
                                       workspace_derivative_list_.at(i));

        }
        else{ //vfi_mode_list_.at(i) == VFI_manager::VFI_MODE::ROBOT_TO_ROBOT


        }
    }
    return VFI_M_->get_inequality_constraints();

}

DQ RobotConstraintsManager::test_fkm(const VectorXd &q) const
{
    return robot_->fkm(q);
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
        //VFI_M_->
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
        std::cout<<"----------data----------------"<<std::endl;
        std::cout<<"      VFI MODE     "<<"   CS entity env/one  "<<" CS entity robot/two "<<" env/one type "<<
            " entv/two type " << " joint index 1 "<<" joint index 2 "<<" safe dist "<< "dir" <<" ent dir1 "<<" ent dir2 "<<std::endl;

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
                    joint_index_list_one_.push_back(raw_joint_index);
                    joint_index_list_two_.push_back(-1);

                    dq_offset_list_one_.    push_back(_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_robot,
                                                                                               raw_joint_index));
                    dq_offset_list_two_.push_back(DQ(-1));

                    robot_attached_dir_list_.push_back(VFI_Framework::map_attached_direction_string_to_dq(raw_entity_robot_attached_direction));
                    envir_attached_dir_list_.push_back(VFI_Framework::map_attached_direction_string_to_dq(raw_entity_environment_attached_direction));
                    workspace_derivative_list_.push_back(DQ(0));
                    cs_entity_environment_DQ_list_.push_back(vi_->get_object_pose(raw_cs_entity_environment));



                    /*
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
*/



                    std::cout << raw_vfi_mode << ",\t"
                              << raw_cs_entity_environment  << ",\t"
                              << raw_cs_entity_robot << ",\t"
                              << raw_entity_robot_primitive_type<< ",\t"
                              << raw_entity_environment_primitive_type <<  ",     \t"
                              << joint_index_list_one_.at(i) << ",    \t"
                              << joint_index_list_two_.at(i) << ",     \t"
                              << raw_safe_distance << ",\t"
                              << robot_attached_dir_list_.at(i) << ",\t"
                              << envir_attached_dir_list_.at(i) <<
                        std::endl;

                }else if (raw_vfi_mode == "ROBOT_TO_ROBOT"){

                    auto raw_cs_entity_one = pos["cs_entity_one"].as<std::string>();
                    auto raw_cs_entity_two = pos["cs_entity_two"].as<std::string>();

                    auto raw_entity_one_primitive_type =  pos["entity_one_primitive_type"].as<std::string>();
                    auto raw_entity_two_primitive_type=   pos["entity_two_primitive_type"].as<std::string>();

                    auto raw_joint_index_one =  pos["joint_index_one"].as<double>();
                    auto raw_joint_index_two =  pos["joint_index_two"].as<double>();

                    auto raw_safe_distance = pos["safe_distance"].as<double>();

                    vfi_mode_list_.push_back(VFI_manager::VFI_MODE::ROBOT_TO_ROBOT);
                    vfi_type_list_.push_back(VFI_Framework::map_strings_to_vfiType(raw_entity_one_primitive_type,
                                                                                   raw_entity_two_primitive_type));
                    //vfi_type_list_.push_back(VFI_Framework::VFI_TYPE::RPOINT_TO_POINT); // This is the only one supported

                    direction_list_.push_back(VFI_Framework::DIRECTION::KEEP_ROBOT_OUTSIDE);
                    safe_distance_list_.push_back(raw_safe_distance);

                    joint_index_list_one_.push_back(raw_joint_index_one);
                    joint_index_list_two_.push_back(raw_joint_index_two);

                    dq_offset_list_one_.push_back(_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_one,
                                                                                               raw_joint_index_one));
                    dq_offset_list_two_.push_back(_get_robot_primitive_offset_from_coppeliasim(raw_cs_entity_two,
                                                                                               raw_joint_index_two));

                    workspace_derivative_list_.push_back(DQ(0));


                    //The following variables are not used in this mode.
                    //However I want to keep all information in the same index.
                    //--------------------------------------------------
                    robot_attached_dir_list_.push_back(DQ(-1));
                    envir_attached_dir_list_.push_back(DQ(-1));
                    cs_entity_environment_DQ_list_.push_back(DQ(-1));
                    //--------------------------------------------------

/*
                    std::cout << raw_vfi_mode << ",\t"
                              << raw_cs_entity_one  << ",\t"
                              << raw_cs_entity_two << ",\t"
                              << raw_entity_one_primitive_type << ",\t"
                              << raw_entity_two_primitive_type << ",\t"
                              << raw_joint_index_one << ",\t"
                              << raw_joint_index_two << ",\t"
                              << raw_safe_distance << std::endl;
*/

                    std::cout << raw_vfi_mode << ",\t"
                              << raw_cs_entity_one  << ",\t"
                              << raw_cs_entity_two << ",  \t"
                              << raw_entity_one_primitive_type<< ",\t"
                              << raw_entity_two_primitive_type << ",     \t"
                              << joint_index_list_one_.at(i) << ",    \t"
                              << joint_index_list_two_.at(i) <<  ",        \t"
                              << raw_safe_distance << ",\t"
                              << robot_attached_dir_list_.at(i) << ",\t"
                              << envir_attached_dir_list_.at(i) <<
                        std::endl;


                }else{
                    throw std::runtime_error("Wrong vfi mode. USE ENVIRONMENT_TO_ROBOT or ROBOT_TO_ROBOT");
                }
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
