#include "suture_simulation.h"
#include <ros/node_handle.h>


/******************
 * SIGNAL HANDLER
******************/
#include <signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

SutureSimulation* create_instance_from_ros_parameter_server(ros::NodeHandle& nodehandle, std::atomic_bool* kill_this_process)
{
    kai::SutureSimulationConfiguration cfg;
    ROS_INFO_STREAM("Trying to load parameters for node" + ros::this_node::getName());
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_ip", cfg.vrep_ip)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_port", cfg.vrep_port)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/realization_iteration_times", cfg.realization_iteration_times)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/continuity_iteration_times", cfg.continuity_iteration_times)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/sampling_time_sec", cfg.dt)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/gravity", cfg.gravity)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/eta_d", cfg.eta_d)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/link7_radius", cfg.link7_radius)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/link8_radius", cfg.link8_radius)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/link9_radius", cfg.link9_radius)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/head_name", cfg.head_name)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/end_name", cfg.end_name)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/element_name_prefix", cfg.element_name_prefix)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vfi_threshold", cfg.vfi_threshold)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/self_collision_check_threshold", cfg.self_collision_check_threshold)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/bed_safe_distance", cfg.bed_safe_distance)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/collision_check", cfg.collision_switch)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/self_collision_check", cfg.self_collision_switch)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/penetration_check", cfg.penetration_avoidance)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/penetration_check_threshold", cfg.penetration_check_threshold)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/suture_parameters_file", cfg.suture_parameters_file)){return nullptr;}

    kai::SutureSimulation* ps = new kai::SutureSimulation(cfg, kill_this_process);
    ROS_INFO_STREAM(ros::this_node::getName() + "::Parameter load OK.");
    return ps;
}

int main(int argc, char** argv)
{
    if (signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }
    ros::init(argc, argv, "suture", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodehandle;
    kai::SutureSimulation* ps = create_instance_from_ros_parameter_server(nodehandle, &kill_this_process);

    if (ps != nullptr)
    {
        try
        {
            ps->initialize();
            ps->control_loop();

            delete ps;
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
        }
    }
    else
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Unable to read parameters from the parameter server.");
    }
    return 0;
}