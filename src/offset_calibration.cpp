
#include <ros/ros.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>
#include <vector>
#include <yaml-cpp/yaml.h>


#ifndef YAML_DEST_PATH
#error YAML_DEST_PATH not defined.
#endif

Eigen::VectorXd g_tau_offset;
XBot::ModelInterface::Ptr g_model;
XBot::RobotInterface::Ptr g_robot;

bool compute_tau_offset()
{
    auto imu = g_robot->getImu().at("imu_link");
    
    g_tau_offset.setZero(g_model->getJointNum());
    Eigen::VectorXd tau, nl;
    
    ros::Rate rate(100);
    
    const int ITER = 300;
    
    for(int i = 0; i < ITER; i++)
    {
        g_robot->sense();
        g_model->syncFrom(*g_robot, XBot::Sync::All, XBot::Sync::MotorSide);
        g_model->setFloatingBaseState(imu);
        g_model->update();
        
        g_model->getJointEffort(tau);
        g_model->computeNonlinearTerm(nl);
        
        g_tau_offset += (nl - tau);
        
        rate.sleep();
        
    }
    
    std::cout << "Measured torques\n" << tau.transpose();
    std::cout << "Model torques\n" << nl.transpose() << std::endl;
    
    g_tau_offset /= ITER;
    
    g_tau_offset.head<6>().setZero();
    
    return true;
    
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "centauro_tau_offset_yaml");
    ros::NodeHandle nh, nh_priv("~");
    auto xbot_cfg = XBot::ConfigOptionsFromParamServer();
    xbot_cfg.set_parameter("is_model_floating_base", true);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");
    
    auto logger = XBot::MatLogger::getLogger("/tmp/centauro_tau_offset_yaml_log");
    auto robot = g_robot = XBot::RobotInterface::getRobot(xbot_cfg);
    auto model = g_model = XBot::ModelInterface::getModel(xbot_cfg);
    auto imu = robot->getImu().at("imu_link");
    
    compute_tau_offset();
   
    
    YAML::Emitter out;
    out << YAML::BeginMap;
    
    
    for ( int i = 0; i < model->getJointNum(); i++ ) {
        out << YAML::Key << model->getEnabledJointNames().at(i);
        out << YAML::Value << g_tau_offset(i);
        
        std::cout << model->getEnabledJointNames().at(i) << ":\t\t" << int(g_tau_offset(i)) << std::endl;
    }
    
    out << YAML::EndMap;
    
    std::cout << "Saving new tau offsets on: " << std::string(YAML_DEST_PATH) << std::endl;
    
    
    std::ofstream fout(std::string(YAML_DEST_PATH));
    fout << out.c_str();
         
    return 0;
}