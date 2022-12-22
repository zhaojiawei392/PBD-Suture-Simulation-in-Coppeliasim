#include "suture_simulation.h"
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <ros/ros.h>
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
    SutureSimulationConfiguration cfg;
    ROS_INFO_STREAM("Trying to load parameters for node" + ros::this_node::getName());
    if(!nodehandle.getParam(ros::this_node::getName()+"/frequency_verbose", cfg.frequency_verbose)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_ip", cfg.vrep_ip)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_port", cfg.vrep_port)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/mass_amount", cfg.mass_amount)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/realization_iteration_times", cfg.realization_iteration_times)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/continuity_iteration_times", cfg.continuity_iteration_times)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/continuity_iteration_times_between_collision_avoidance", cfg.continuity_iteration_times_between_collision_avoidance)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/forward_priority", cfg.forward_priority)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/original_distance", cfg.original_distance)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/continuity_distance", cfg.continuity_distance)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/elastic_modulus", cfg.elastic_modulus)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/activity_level", cfg.activity_level)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/mass", cfg.mass)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/sampling_time_sec", cfg.dt)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/gravity", cfg.gravity)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/eta_d", cfg.eta_d)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/eta_d_ls", cfg.eta_d_ls)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/eta_d_self", cfg.eta_d_self)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/link7_radius", cfg.link7_radius)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/link8_radius", cfg.link8_radius)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/link9_radius", cfg.link9_radius)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/head_name", cfg.head_name)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/end_name", cfg.end_name)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/particle_name", cfg.particle_name)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/thread_sampling_time_nsec", cfg.thread_sampling_time_nsec)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/inverse_sending_speed", cfg.inverse_sending_speed)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vfi_threshold", cfg.vfi_threshold)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/self_collision_check_threshold", cfg.self_collision_check_threshold)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/bed_safe_distance", cfg.bed_safe_distance)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/collision_check", cfg.collision_switch)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/self_collision_check", cfg.self_collision_switch)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/penetration_avoidance", cfg.penetration_avoidance)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/penetration_check_threshold", cfg.penetration_check_threshold)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/collision_damping", cfg.collision_damping)){return nullptr;}
    SutureSimulation* ps = new SutureSimulation(cfg, kill_this_process);
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
    SutureSimulation* ps = create_instance_from_ros_parameter_server(nodehandle, &kill_this_process);

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