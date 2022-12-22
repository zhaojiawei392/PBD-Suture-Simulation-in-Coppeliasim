#include "suture_simulation.h"
#include <ros/ros.h>
#include <chrono>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

SutureSimulation::SutureSimulation(SutureSimulationConfiguration cfg, 
                    std::atomic_bool* kill_this_node):
    enabled_(false),
    cfg_(cfg),
    kill_this_node_(kill_this_node),
    vrep_interface_(kill_this_node),
    world_reference_(0)
{
}

void SutureSimulation::control_loop()
{
    try
    {   
        ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing pre-loop variables...");
        int size = cfg_.mass_amount + 2;
        particles_.resize(size);
        // particles_[0] = vrep_interface_.get_object_handle(cfg_.head_name);
        const int& head_handle = vrep_interface_.get_object_handle(cfg_.head_name);
        const int& end_handle = vrep_interface_.get_object_handle(cfg_.end_name);
        for (int i=0; i<particles_.size(); ++i)
        {
            particles_[i] = vrep_interface_.get_object_handle(cfg_.particle_name + std::to_string(i));
        }
        
        std::vector<DQ> curr(size); // current positions of particles_
        // initialize current positions 
        for (int i=0; i<size; ++i)
        {
            curr[i] = vrep_interface_.get_object_translation(particles_[i], world_reference_, DQ_VrepInterface::OP_STREAMING);
        } 
        // initialize past positions 
        std::vector<DQ> past = curr;
        std::vector<DQ> future = curr; // future positions of particles_
        
        // vrep_interface_.set_object_translation(particles_[size - 1], world_reference_, get_object_translation(end_handle, world_reference_, DQ_VrepInterface::OP_STREAMING), DQ_VrepInterface::OP_ONESHOT);
        CollisionCheckMaterialBox cc(vrep_interface_);
        CollisionCheckMaterialBox prev_cc = cc;
        prev_cc.update();
        ROS_INFO_STREAM(ros::this_node::getName() + "::pre-loop variables initialized.");
        ROS_INFO_STREAM(ros::this_node::getName() + "::Starting control looping...");
        auto start = std::chrono::steady_clock::now();
        while (not _should_shutdown())
        {
            if (cfg_.frequency_verbose)
            {
                std::chrono::duration<double> elapsed_seconds = std::chrono::steady_clock::now() - start;
                start = std::chrono::steady_clock::now();
                ROS_INFO_STREAM(1. / elapsed_seconds.count());
            }

            cc.update();

            // read head and end positions
            future[0] = curr[0] = vrep_interface_.get_object_translation(head_handle, world_reference_, DQ_VrepInterface::OP_STREAMING);
            future[size-1] = curr[size-1] = vrep_interface_.get_object_translation(end_handle, world_reference_, DQ_VrepInterface::OP_STREAMING);

            auto __realize_suture = [&](std::vector<DQ>& predicted, const std::vector<DQ>& now, int count)
            {
                while(count--)
                {
                    _continuity(predicted, cfg_.continuity_iteration_times);
                    _collision_avoidance(predicted, now, cc, prev_cc);
                    _penetration_avoidance(predicted, now, cc, prev_cc);
                    _continuity(predicted, cfg_.continuity_iteration_times_between_collision_avoidance);
                    _self_collision_avoidance(predicted, now, cfg_.original_distance);
                }
            };

            __realize_suture(curr, past, cfg_.realization_iteration_times);

            _calculate_future_positions(future, curr, past, cc);

            __realize_suture(future, curr, cfg_.realization_iteration_times);

            _move_suture(future);
            
            past = curr;
            curr = future;
            prev_cc = cc;
        }

    }
    catch(const std::exception& e)
    {
        ROS_WARN_STREAM(ros::this_node::getName()+"::Error or exeption caught::" + e.what());
    }
    catch(...)
    {
        ROS_WARN_STREAM(ros::this_node::getName()+"::Unexpected error or exception caught");
    }
}

void SutureSimulation::initialize()
{
    if (!vrep_interface_.connect(cfg_.vrep_ip, cfg_.vrep_port, 100, 10))
    {
        vrep_interface_.disconnect_all();
        throw std::runtime_error(ros::this_node::getName()+"::Unable to connect to Vrep "+cfg_.vrep_ip+" port: "+std::to_string(cfg_.vrep_port));
    }
    vrep_interface_.start_simulation();
    ROS_INFO_STREAM(ros::this_node::getName()+"::Waiting for vrep enabled...");
    while(!vrep_interface_.is_simulation_running())
    {
        vrep_interface_.start_simulation();
        if (_should_shutdown())
            throw std::runtime_error(ros::this_node::getName() + "::Shutdown while trying to connect to vrep.");
    }
    enabled_ = true;
}

bool SutureSimulation::enabled() const
{
    return enabled_;
}

void SutureSimulation::_continuity(std::vector<DQ>& positions, int count) const
{
    int size = positions.size();
    while(count-- > 0)
    {
        DQ diff = positions[0] - positions[1];
        double diff_norm = vec4(diff).norm();
        positions[1] = positions[1] + (diff_norm - cfg_.continuity_distance) * diff * (1 / diff_norm);
        diff = positions[2] - positions[1];
        diff_norm = vec4(diff).norm();
        positions[1] = positions[1] + (1-cfg_.forward_priority) * (diff_norm - cfg_.continuity_distance) * diff * (1 / diff_norm);
        for (int i=2; i<size-2; ++i)
        {
            diff = positions[i-1] - positions[i];
            diff_norm = vec4(diff).norm();
            positions[i] = positions[i] + cfg_.forward_priority * (diff_norm - cfg_.continuity_distance) * diff * (1 / diff_norm);
            diff = positions[i+1] - positions[i];
            diff_norm = vec4(diff).norm();
            positions[i] = positions[i] + (1-cfg_.forward_priority) * (diff_norm - cfg_.continuity_distance) * diff * (1 / diff_norm);
        }
        diff = positions[size-3] - positions[size-2];
        diff_norm = vec4(diff).norm();
        positions[size-2] = positions[size-2] + cfg_.forward_priority * (diff_norm - cfg_.continuity_distance) * diff * (1 / diff_norm);
        diff = positions[size-1] - positions[size-2];
        diff_norm = vec4(diff).norm();
        positions[size-2] = positions[size-2] + (diff_norm - cfg_.continuity_distance) * diff * (1 / diff_norm);
    }
}

void __penetration_avoidance(DQ& p1, DQ& p2, const MyLineSegment& line2, const MyLineSegment& past_line2, const bool& is_penetrated, const double& safe_distance)
{
    if (is_penetrated)
    {
        const MyLineSegment line1(p1, p2);
        DQ normal_dq = line1 - line2;
        const double& normal_norm = vec3(normal_dq).norm();
        DQ past_normal_dq = line1 - past_line2;
        const double& past_normal_norm = vec3(past_normal_dq).norm();

        const DQ& modified = normal_dq * (1 / normal_norm) * (normal_norm + past_normal_norm + safe_distance);
        p1 = p1 - modified;
        p2 = p2 - modified;
    }
}

bool SutureSimulation::_check_point_collision(const DQ& position, const DQ& obstacle, const double& safe_distance) const
{
    if (vec3(position - obstacle).norm() <= safe_distance + cfg_.original_distance)
    {
        return true;
    }
    return false;
}

bool SutureSimulation::_check_line_segment_collision(const DQ& position, const MyLineSegment& obstacle, const double& safe_distance) const
{
    if (!obstacle.is_projected_in_segment(position))
        return false;
    return _check_point_collision(position, obstacle.projection_point(position), safe_distance);
}

bool SutureSimulation::_check_plane_collision(const DQ& position, const DQ& obstacle, const double& safe_distance) const
{
    if (DQ_Geometry::point_to_plane_distance(position, obstacle) <= safe_distance + cfg_.original_distance)
    {
        return true;
    }
    return false;
}

void SutureSimulation::_point_collision_avoidance(DQ& future_point, const DQ& curr_point, const DQ& ws_point, const DQ& past_ws_point, const double& safe_distance) const
{
    const double& normal_norm = vec3(curr_point - ws_point).norm();
    if (normal_norm < cfg_.vfi_threshold * safe_distance)
    {
        const DQ& normal_dq = (curr_point - ws_point) * (1 / normal_norm);
        const Vector3d& normal_vec = vec3(normal_dq);
        const double& escape_speed = (double(vec3(future_point - curr_point).transpose() * normal_vec)
                                    - double(vec3(ws_point - past_ws_point).transpose() * normal_vec)) / cfg_.dt;
        const double& vfi_limit_speed = cfg_.eta_d * (safe_distance - normal_norm);
        if (escape_speed < vfi_limit_speed)
        {
            future_point = future_point + normal_dq * cfg_.dt * (vfi_limit_speed - escape_speed);
        }
    }
}

void SutureSimulation::_line_segment_collision_avoidance(DQ& future_point, const DQ& curr_point, const MyLineSegment& ws_ls, const MyLineSegment& past_ws_ls, const double& safe_distance) const
{
    double curr_normal_norm = ws_ls.distance(curr_point);
    double past_normal_norm = past_ws_ls.distance(curr_point);

    if (curr_normal_norm < cfg_.vfi_threshold * safe_distance && ws_ls.is_projected_in_segment(curr_point) && past_ws_ls.is_projected_in_segment(curr_point))
    {
        const DQ& pp = ws_ls.projection_point(curr_point);
        const DQ& normal_dq = (curr_point - pp) * (1 / curr_normal_norm);
        const double& escape_speed = (double(vec3(future_point - curr_point).transpose() * vec3(normal_dq))
                                    + (curr_normal_norm - past_normal_norm)) / cfg_.dt;
        const double& vfi_limit_speed = cfg_.eta_d * (safe_distance - curr_normal_norm);
        if (escape_speed < vfi_limit_speed)
        {
            future_point = future_point + normal_dq * cfg_.dt * (vfi_limit_speed - escape_speed);
        }         
    }
}

void SutureSimulation::_plane_collision_avoidance(DQ& future_point, const DQ& curr_point, const DQ& plane, const DQ& past_plane, const double& safe_distance) const
{
    const double& normal_norm = DQ_Geometry::point_to_plane_distance(curr_point, plane);
    if (normal_norm < cfg_.vfi_threshold * safe_distance)
    {
        const DQ& normal_dq = plane.P();
        const Vector3d& normal_vec = vec3(normal_dq);
        const double& escape_speed = (double(vec3(future_point - curr_point).transpose() * normal_vec)
                                    - double(vec3(plane.D() - past_plane.D()).transpose() * normal_vec)) / cfg_.dt;

        const double& vfi_limit_speed = cfg_.eta_d * (safe_distance - normal_norm);
        if (escape_speed < vfi_limit_speed)
        {
            future_point = future_point + normal_dq * cfg_.dt * (vfi_limit_speed - escape_speed);
        }         
    }
}

bool SutureSimulation::_check_penetration(const MyLineSegment& future_line1, const MyLineSegment& curr_line1, const MyLineSegment& future_line2, const MyLineSegment& curr_line2) const
{
    DQ curr_cp1;
    DQ curr_cp2;
    std::tie(curr_cp1, curr_cp2) = MLS::closest_points_between_lines(curr_line1, curr_line2);
    DQ future_cp1;
    DQ future_cp2;
    std::tie(future_cp1, future_cp2) = MLS::closest_points_between_lines(future_line1, future_line2);

    const DQ& curr_cross = cross(curr_line1.first_point() - curr_cp2, curr_line1.second_point() - curr_cp2);
    const DQ& future_cross = cross(future_line1.first_point() - future_cp2, future_line1.second_point() - future_cp2);
    if (curr_line1.is_cross(curr_line2) && future_line1.is_cross(future_line2)
     && vec3(curr_cp1 - curr_cp2).norm() < cfg_.penetration_check_threshold && vec3(curr_cross).dot(vec3(future_cross)) <= 0)
    {
        // penetration happened
        ROS_INFO_STREAM(ros::this_node::getName() + "::Penetration checked.");
        return true;
    }
    return false;
}

bool SutureSimulation::_self_penetration_avoidance(DQ& p1, DQ& p2, DQ& p3, DQ& p4, const MyLineSegment& future_line1, const MyLineSegment& curr_line1, const MyLineSegment& future_line2, const MyLineSegment& curr_line2, const double& safe_distance) const
{
    DQ curr_cp1;
    DQ curr_cp2;
    std::tie(curr_cp1, curr_cp2) = MLS::closest_points_between_lines(curr_line1, curr_line2);
    DQ future_cp1;
    DQ future_cp2;
    std::tie(future_cp1, future_cp2) = MLS::closest_points_between_lines(future_line1, future_line2);

    const DQ& curr_cross = cross(curr_line1.first_point() - curr_cp2, curr_line1.second_point() - curr_cp2);
    const DQ& future_cross = cross(future_line1.first_point() - future_cp2, future_line1.second_point() - future_cp2);
    if (curr_line1.is_cross(curr_line2) && future_line1.is_cross(future_line2)
     && vec3(curr_cp1 - curr_cp2).norm() < cfg_.penetration_check_threshold && vec3(curr_cross).dot(vec3(future_cross)) <= 0)
    {
        // penetration happened
        const double& normal_norm = vec3(future_cp1 - future_cp2).norm();
        const DQ& modified = (future_cp1 - future_cp2) * (1 / normal_norm) * 0.5 * (normal_norm + safe_distance);
        p1 = p1 - modified;
        p2 = p2 - modified;
        p3 = p3 + modified;
        p4 = p4 + modified;
        return true;
    }
    return false;
}

void SutureSimulation::_self_collision_avoidance(std::vector<DQ>& future, const std::vector<DQ>& curr, const double& safe_distance) const
{
    if (cfg_.self_collision_switch)
    {
        const int& size = curr.size();
        for (int i=2; i<size-3; ++i)
        {
            const MyLineSegment curr_ls1(curr[i-1], curr[i]);
            const MyLineSegment future_ls1(future[i-1], future[i]);
            for (int j=i+cfg_.self_collision_check_threshold; j<size-1; ++j)
            {
                const MyLineSegment curr_ls2(curr[j-1], curr[j]);
                const MyLineSegment future_ls2(future[j-1], future[j]);
                _self_penetration_avoidance(future[i-1], future[i], future[j-1], future[j], future_ls1, curr_ls1, future_ls2, curr_ls2, cfg_.original_distance);
            }
        }
    }

}

void SutureSimulation::_collision_avoidance(std::vector<DQ>& future, const std::vector<DQ>& curr, const CollisionCheckMaterialBox& cc, const CollisionCheckMaterialBox& prev_cc) const
{
    if (cfg_.collision_switch)
    {
        for (int i=2; i<future.size()-1; ++i)
        {
            _point_collision_avoidance(future[i], curr[i], cc.ta2, prev_cc.ta2, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.te2, prev_cc.te2, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.tf2, prev_cc.tf2, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.tg2, prev_cc.tg2, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.tb2, prev_cc.tb2, cfg_.link8_radius);
            _line_segment_collision_avoidance(future[i], curr[i], cc.robot2_link9, prev_cc.robot2_link9, cfg_.link9_radius);
            _line_segment_collision_avoidance(future[i], curr[i], cc.robot2_link7, prev_cc.robot2_link7, cfg_.link7_radius);

            _point_collision_avoidance(future[i], curr[i], cc.ta1, prev_cc.ta1, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.te1, prev_cc.te1, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.tf1, prev_cc.tf1, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.tg1, prev_cc.tg1, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.tb1, prev_cc.tb1, cfg_.link8_radius);
            _line_segment_collision_avoidance(future[i], curr[i], cc.robot1_link9, prev_cc.robot1_link9, cfg_.link9_radius);
            _line_segment_collision_avoidance(future[i], curr[i], cc.robot1_link7, prev_cc.robot1_link7, cfg_.link7_radius);
            _plane_collision_avoidance(future[i], curr[i], cc.bed, prev_cc.bed, cfg_.bed_safe_distance);
        }
    }
}

void SutureSimulation::_penetration_avoidance(std::vector<DQ>& future, const std::vector<DQ>& curr, const CollisionCheckMaterialBox& cc, const CollisionCheckMaterialBox& prev_cc) const
{
    if (cfg_.penetration_avoidance)
    {
        for (int i=2; i<future.size()-1; ++i)
        {
            MyLineSegment future_line(future[i-1], future[i]);
            MyLineSegment curr_line(curr[i-1], curr[i]);
            __penetration_avoidance(future[i-1], future[i], cc.robot1_link9, prev_cc.robot1_link9,
                            _check_penetration(future_line, curr_line, cc.robot1_link9, prev_cc.robot1_link9), cfg_.link9_radius);
            const MyLineSegment ae1(cc.ta1, cc.te1);
            const MyLineSegment past_ae1(prev_cc.ta1, prev_cc.te1);
            __penetration_avoidance(future[i-1], future[i], ae1, past_ae1,
                                    _check_penetration(future_line, curr_line, ae1, past_ae1), cfg_.link8_radius);
            const MyLineSegment ef1(cc.te1, cc.tf1);
            const MyLineSegment past_ef1(prev_cc.te1, prev_cc.tf1);
            __penetration_avoidance(future[i-1], future[i], ef1, past_ef1,
                                    _check_penetration(future_line, curr_line, ef1, past_ef1), cfg_.link8_radius);
            const MyLineSegment fg1(cc.tf1, cc.tg1);
            const MyLineSegment past_fg1(prev_cc.tf1, prev_cc.tg1);
            __penetration_avoidance(future[i-1], future[i], fg1, past_fg1,
                                    _check_penetration(future_line, curr_line, fg1, past_fg1), cfg_.link8_radius);
            const MyLineSegment gb1(cc.tg1, cc.tb1);
            const MyLineSegment past_gb1(prev_cc.tg1, prev_cc.tb1);
            __penetration_avoidance(future[i-1], future[i], gb1, past_gb1,
                                    _check_penetration(future_line, curr_line, gb1, past_gb1), cfg_.link8_radius);
            __penetration_avoidance(future[i-1], future[i], cc.robot1_link7, prev_cc.robot1_link7,
                            _check_penetration(future_line, curr_line, cc.robot1_link7, prev_cc.robot1_link7), cfg_.link7_radius);                  
        }
        for (int i=future.size()-2; i==2; --i)
        {
            MyLineSegment future_line(future[i-1], future[i]);
            MyLineSegment curr_line(curr[i-1], curr[i]);
            __penetration_avoidance(future[i-1], future[i], cc.robot1_link9, prev_cc.robot1_link9,
                            _check_penetration(future_line, curr_line, cc.robot1_link9, prev_cc.robot1_link9), cfg_.link9_radius);
            const MyLineSegment ae1(cc.ta1, cc.te1);
            const MyLineSegment past_ae1(prev_cc.ta1, prev_cc.te1);
            __penetration_avoidance(future[i-1], future[i], ae1, past_ae1,
                                    _check_penetration(future_line, curr_line, ae1, past_ae1), cfg_.link8_radius);
            const MyLineSegment ef1(cc.te1, cc.tf1);
            const MyLineSegment past_ef1(prev_cc.te1, prev_cc.tf1);
            __penetration_avoidance(future[i-1], future[i], ef1, past_ef1,
                                    _check_penetration(future_line, curr_line, ef1, past_ef1), cfg_.link8_radius);
            const MyLineSegment fg1(cc.tf1, cc.tg1);
            const MyLineSegment past_fg1(prev_cc.tf1, prev_cc.tg1);
            __penetration_avoidance(future[i-1], future[i], fg1, past_fg1,
                                    _check_penetration(future_line, curr_line, fg1, past_fg1), cfg_.link8_radius);
            const MyLineSegment gb1(cc.tg1, cc.tb1);
            const MyLineSegment past_gb1(prev_cc.tg1, prev_cc.tb1);
            __penetration_avoidance(future[i-1], future[i], gb1, past_gb1,
                                    _check_penetration(future_line, curr_line, gb1, past_gb1), cfg_.link8_radius);
            __penetration_avoidance(future[i-1], future[i], cc.robot1_link7, prev_cc.robot1_link7,
                            _check_penetration(future_line, curr_line, cc.robot1_link7, prev_cc.robot1_link7), cfg_.link7_radius);                  
        }

    }
}

bool SutureSimulation::_check_collision(const DQ& position, const CollisionCheckMaterialBox& cc) const
{
    if (_check_line_segment_collision(position, cc.robot1_link9, cfg_.link9_radius) ||
        _check_point_collision(position, cc.ta1, cfg_.link8_radius) ||
        _check_point_collision(position, cc.te1, cfg_.link8_radius) ||
        _check_point_collision(position, cc.tf1, cfg_.link8_radius) ||
        _check_point_collision(position, cc.tg1, cfg_.link8_radius) ||
        _check_point_collision(position, cc.tb1, cfg_.link8_radius) ||
        _check_line_segment_collision(position, cc.robot1_link7, cfg_.link7_radius) ||
        _check_plane_collision(position, cc.bed, cfg_.bed_safe_distance)
        )
        return true;
    return false;
}

void SutureSimulation::_calculate_future_positions(std::vector<DQ>& future, const std::vector<DQ>& curr, const std::vector<DQ>& past, const CollisionCheckMaterialBox& cc) const
{
    const double& original_lamda = 0.5 * pow(cfg_.dt, 2) * cfg_.elastic_modulus / cfg_.mass;
    std::vector<double> dl(future.size()-1);
    dl[0] = 1 - cfg_.original_distance / vec4(curr[0] - curr[1]).norm();
    for (int i=1; i<=cfg_.mass_amount; ++i)
    {
        dl[i] = 1 - cfg_.original_distance / vec4(curr[i] - curr[i+1]).norm();
        double lamda = original_lamda;
        if (_check_collision(curr[i], cc)) 
            continue;
            // lamda = original_lamda / cfg_.collision_damping;
        future[i] = lamda * dl[i-1] * curr[i-1] + (1 - lamda * (dl[i-1] + dl[i])) * curr[i] + lamda * dl[i] * curr[i+1] + cfg_.activity_level * (curr[i] - past[i]) - pow(cfg_.dt, 2) * k_ * cfg_.gravity;
    }
}

void SutureSimulation::_move_suture(const std::vector<DQ>& positions) const
{
    for (int i=0; i<positions.size(); ++i)
    {
        vrep_interface_.set_object_translation(particles_[i], world_reference_, positions[i], DQ_VrepInterface::OP_ONESHOT);
    }
}

bool SutureSimulation::_should_shutdown() const
{
    return (*kill_this_node_);
}

void CollisionCheckMaterialBox::update()
{
    int world_reference = 0;
    t1 = vi_.get_object_translation("x1", world_reference, DQ_VrepInterface::OP_STREAMING);
    t2 = vi_.get_object_translation("x2", world_reference, DQ_VrepInterface::OP_STREAMING);
    ta1 = vi_.get_object_translation("a1", world_reference, DQ_VrepInterface::OP_STREAMING);
    te1 = vi_.get_object_translation("e1", world_reference, DQ_VrepInterface::OP_STREAMING);
    tf1 = vi_.get_object_translation("f1", world_reference, DQ_VrepInterface::OP_STREAMING);
    tg1 = vi_.get_object_translation("g1", world_reference, DQ_VrepInterface::OP_STREAMING);
    tb1 = vi_.get_object_translation("b1", world_reference, DQ_VrepInterface::OP_STREAMING);
    tc1 = vi_.get_object_translation("c1", world_reference, DQ_VrepInterface::OP_STREAMING);
    ta2 = vi_.get_object_translation("a2", world_reference, DQ_VrepInterface::OP_STREAMING);
    te2 = vi_.get_object_translation("e2", world_reference, DQ_VrepInterface::OP_STREAMING);
    tf2 = vi_.get_object_translation("f2", world_reference, DQ_VrepInterface::OP_STREAMING);
    tg2 = vi_.get_object_translation("g2", world_reference, DQ_VrepInterface::OP_STREAMING);
    tb2 = vi_.get_object_translation("b2", world_reference, DQ_VrepInterface::OP_STREAMING);
    tc2 = vi_.get_object_translation("c2", world_reference, DQ_VrepInterface::OP_STREAMING);
    robot1_link9.reset(t1, ta1);
    robot2_link9.reset(t2, ta2);
    robot1_link7.reset(tb1, tc1);
    robot2_link7.reset(tb2, tc2);
    bed = k_ + E_ * dot(vi_.get_object_translation("bed"), k_);
}

CollisionCheckMaterialBox& CollisionCheckMaterialBox::operator=(const CollisionCheckMaterialBox& cc)
{
    t1 = cc.t1;
    t2 = cc.t2;
    ta1 = cc.ta1;
    ta2 = cc.ta2;
    tb1 = cc.tb1;
    tb2 = cc.tb2;
    tc1 = cc.tc1;
    tc2 = cc.tc2;
    te1 = cc.te1;
    te2 = cc.te2;
    tf1 = cc.tf1;
    tf2 = cc.tf2;
    tg1 = cc.tg1;
    tg2 = cc.tg2;
    robot1_link9 = cc.robot1_link9;
    robot2_link9 = cc.robot2_link9;
    robot1_link7 = cc.robot1_link7;
    robot2_link7 = cc.robot2_link7;
    bed = cc.bed;
    return *this;
}