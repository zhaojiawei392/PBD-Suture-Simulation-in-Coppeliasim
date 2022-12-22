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
    vrep_interface_(kill_this_node)
{
}

void SutureSimulation::control_loop()
{
    try
    {   
        ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing pre-loop variables...");

        particles_.resize(cfg_.mass_amount);
        // particles_[0] = vrep_interface_.get_object_handle(cfg_.head_name);
        const int& head_handle = vrep_interface_.get_object_handle(cfg_.head_name);
        const int& end_handle = vrep_interface_.get_object_handle(cfg_.end_name);
        for (int i=0; i<particles_.size(); ++i)
        {
            particles_[i] = vrep_interface_.get_object_handle(cfg_.particle_name + std::to_string(i));
        }
        
        std::vector<DQ> curr(cfg_.mass_amount); // current positions of particles_
        // initialize current positions 
        for (int i=0; i<curr.size(); ++i)
        {
            curr[i] = vrep_interface_.get_object_translation(particles_[i], DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_BLOCKING);
        } 
        // initialize past and future positions 
        std::vector<DQ> past = curr;
        std::vector<DQ> future = curr; 
        
        CollisionCheckMaterialBox cc1(vrep_interface_, "1");
        CollisionCheckMaterialBox cc2(vrep_interface_, "2");
        CollisionCheckMaterialBox past_cc1 = cc1;
        CollisionCheckMaterialBox past_cc2 = cc2;
        past_cc1.update();
        past_cc2.update();
        head_attachment_ = EndAttachment::attached;
        end_attachment_ = EndAttachment::attached;
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

            cc1.update();
            cc2.update();

            // read head and end positions
            if (head_attachment_ == EndAttachment::attached)
            {
                future[0] = curr[0] = vrep_interface_.get_object_translation(head_handle, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_BLOCKING);
            }
            if (end_attachment_ == EndAttachment::attached)
            {
                future[cfg_.mass_amount-1] = curr[cfg_.mass_amount-1] = vrep_interface_.get_object_translation(end_handle, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_BLOCKING);
            }
            
            auto __realize_suture = [&](std::vector<DQ>& predicted, const std::vector<DQ>& now, int count)
            {
                while(count--)
                {
                    _continuity(predicted, cfg_.continuity_iteration_times);

                    _collision_avoidance(predicted, now, cc2, past_cc2);
                    _collision_avoidance(predicted, now, cc1, past_cc1);

                    _penetration_avoidance(predicted, now, cc2, past_cc2);
                    _penetration_avoidance(predicted, now, cc1, past_cc1);

                    _self_collision_avoidance(predicted, now);
                }
            };

            __realize_suture(curr, past, cfg_.realization_iteration_times);
                
            _calculate_future_positions(future, curr, past, cc1, cc2);

            __realize_suture(future, curr, cfg_.realization_iteration_times);

            _move_suture(future);
            
            past = curr;
            curr = future;
            past_cc1 = cc1;
            past_cc2 = cc2;
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
    auto __jakobsen_method = [&](const int& index, const int& toward_index, const double& ratio)
    {
        const DQ& diff = positions[toward_index] - positions[index];
        const double& diff_norm = vec3(diff).norm();
        positions[index] = positions[index] + ratio * (1 - cfg_.continuity_distance  / diff_norm) * diff;      
    };
    while(count-- > 0)
    {
        __jakobsen_method(1, 0, 1);
        __jakobsen_method(1, 2, 1 - cfg_.forward_priority);
        for (int i=1; i<size-1; ++i)
        {
            __jakobsen_method(i, i-1, cfg_.forward_priority);
            __jakobsen_method(i, i+1, 1 - cfg_.forward_priority);
        }
        __jakobsen_method(size-2, size-3, cfg_.forward_priority);
        __jakobsen_method(size-2, size-1, 1);
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

void SutureSimulation::_self_collision_avoidance(std::vector<DQ>& future, const std::vector<DQ>& curr) const
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
                _self_penetration_avoidance(future[i-1], future[i], future[j-1], future[j], 
                                            future_ls1, curr_ls1, future_ls2, curr_ls2, cfg_.original_distance);
            }
        }
    }

}

void SutureSimulation::_collision_avoidance(std::vector<DQ>& future, const std::vector<DQ>& curr, const CollisionCheckMaterialBox& cc, const CollisionCheckMaterialBox& past_cc) const
{
    if (cfg_.collision_switch)
    {
        for (int i=0; i<future.size(); ++i)
        {
            _point_collision_avoidance(future[i], curr[i], cc.ta, past_cc.ta, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.te, past_cc.te, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.tf, past_cc.tf, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.tg, past_cc.tg, cfg_.link8_radius);
            _point_collision_avoidance(future[i], curr[i], cc.tb, past_cc.tb, cfg_.link8_radius);
            _line_segment_collision_avoidance(future[i], curr[i], cc.link9, past_cc.link9, cfg_.link9_radius);
            _line_segment_collision_avoidance(future[i], curr[i], cc.link7, past_cc.link7, cfg_.link7_radius);
            _plane_collision_avoidance(future[i], curr[i], cc.bed, past_cc.bed, cfg_.bed_safe_distance);
        }
    }
}

void SutureSimulation::_penetration_avoidance(std::vector<DQ>& future, const std::vector<DQ>& curr, const CollisionCheckMaterialBox& cc, const CollisionCheckMaterialBox& past_cc) const
{
    if (cfg_.penetration_avoidance)
    {
        for (int i=1; i<future.size(); ++i)
        {
            MyLineSegment future_line(future[i-1], future[i]);
            MyLineSegment curr_line(curr[i-1], curr[i]);
            __penetration_avoidance(future[i-1], future[i], cc.link9, past_cc.link9,
                            _check_penetration(future_line, curr_line, cc.link9, past_cc.link9), cfg_.link9_radius);
            const MyLineSegment ae(cc.ta, cc.te);
            const MyLineSegment past_ae(past_cc.ta, past_cc.te);
            __penetration_avoidance(future[i-1], future[i], ae, past_ae,
                                    _check_penetration(future_line, curr_line, ae, past_ae), cfg_.link8_radius);
            const MyLineSegment ef(cc.te, cc.tf);
            const MyLineSegment past_ef(past_cc.te, past_cc.tf);
            __penetration_avoidance(future[i-1], future[i], ef, past_ef,
                                    _check_penetration(future_line, curr_line, ef, past_ef), cfg_.link8_radius);
            const MyLineSegment fg(cc.tf, cc.tg);
            const MyLineSegment past_fg(past_cc.tf, past_cc.tg);
            __penetration_avoidance(future[i-1], future[i], fg, past_fg,
                                    _check_penetration(future_line, curr_line, fg, past_fg), cfg_.link8_radius);
            const MyLineSegment gb(cc.tg, cc.tb);
            const MyLineSegment past_gb(past_cc.tg, past_cc.tb);
            __penetration_avoidance(future[i-1], future[i], gb, past_gb,
                                    _check_penetration(future_line, curr_line, gb, past_gb), cfg_.link8_radius);
            __penetration_avoidance(future[i-1], future[i], cc.link7, past_cc.link7,
                            _check_penetration(future_line, curr_line, cc.link7, past_cc.link7), cfg_.link7_radius);                  
        }
        for (int i=future.size()-2; i==2; --i)
        {
            MyLineSegment future_line(future[i-1], future[i]);
            MyLineSegment curr_line(curr[i-1], curr[i]);
            __penetration_avoidance(future[i-1], future[i], cc.link9, past_cc.link9,
                            _check_penetration(future_line, curr_line, cc.link9, past_cc.link9), cfg_.link9_radius);
            const MyLineSegment ae(cc.ta, cc.te);
            const MyLineSegment past_ae(past_cc.ta, past_cc.te);
            __penetration_avoidance(future[i-1], future[i], ae, past_ae,
                                    _check_penetration(future_line, curr_line, ae, past_ae), cfg_.link8_radius);
            const MyLineSegment ef(cc.te, cc.tf);
            const MyLineSegment past_ef(past_cc.te, past_cc.tf);
            __penetration_avoidance(future[i-1], future[i], ef, past_ef,
                                    _check_penetration(future_line, curr_line, ef, past_ef), cfg_.link8_radius);
            const MyLineSegment fg(cc.tf, cc.tg);
            const MyLineSegment past_fg(past_cc.tf, past_cc.tg);
            __penetration_avoidance(future[i-1], future[i], fg, past_fg,
                                    _check_penetration(future_line, curr_line, fg, past_fg), cfg_.link8_radius);
            const MyLineSegment gb(cc.tg, cc.tb);
            const MyLineSegment past_gb(past_cc.tg, past_cc.tb);
            __penetration_avoidance(future[i-1], future[i], gb, past_gb,
                                    _check_penetration(future_line, curr_line, gb, past_gb), cfg_.link8_radius);
            __penetration_avoidance(future[i-1], future[i], cc.link7, past_cc.link7,
                            _check_penetration(future_line, curr_line, cc.link7, past_cc.link7), cfg_.link7_radius);                   
        }

    }
}

bool SutureSimulation::_check_collision(const DQ& position, const CollisionCheckMaterialBox& cc) const
{
    if (_check_line_segment_collision(position, cc.link9, cfg_.link9_radius) ||
        _check_point_collision(position, cc.ta, cfg_.link8_radius) ||
        _check_point_collision(position, cc.te, cfg_.link8_radius) ||
        _check_point_collision(position, cc.tf, cfg_.link8_radius) ||
        _check_point_collision(position, cc.tg, cfg_.link8_radius) ||
        _check_point_collision(position, cc.tb, cfg_.link8_radius) ||
        _check_line_segment_collision(position, cc.link7, cfg_.link7_radius) ||
        _check_plane_collision(position, cc.bed, cfg_.bed_safe_distance)
        )
        return true;
    return false;
}

void SutureSimulation::_calculate_future_positions(std::vector<DQ>& future, const std::vector<DQ>& curr, const std::vector<DQ>& past, const CollisionCheckMaterialBox& cc1, const CollisionCheckMaterialBox& cc2) const
{
    const int& size = future.size();
    const int& end_index = size -1;
    const double& lamda = 0.5 * pow(cfg_.dt, 2) * cfg_.elastic_modulus / cfg_.mass;
    std::vector<double> dl(size-1);
    dl[0] = 1 - cfg_.original_distance / vec4(curr[0] - curr[1]).norm();
    for (int i=1; i<end_index; ++i)
    {
        dl[i] = 1 - cfg_.original_distance / vec4(curr[i] - curr[i+1]).norm();
        if (_check_collision(curr[i], cc1) || _check_collision(curr[i], cc2)) 
            continue;
        future[i] = lamda * dl[i-1] * curr[i-1] + (1 - lamda * (dl[i-1] + dl[i])) * curr[i] + lamda * dl[i] * curr[i+1] + cfg_.activity_level * (curr[i] - past[i]) - pow(cfg_.dt, 2) * k_ * cfg_.gravity;
    }
    if (head_attachment_ == EndAttachment::released && !_check_collision(curr[0], cc1) && !_check_collision(curr[0], cc2))
    {
        future[0] = (1 - lamda * (dl[-1] + dl[0])) * curr[0] + lamda * dl[0] * curr[1] + cfg_.activity_level * (curr[0] - past[0]) - pow(cfg_.dt, 2) * k_ * cfg_.gravity;
    }
    if (end_attachment_ == EndAttachment::released && !_check_collision(curr[end_index], cc1) && !_check_collision(curr[end_index], cc2))
    {
        future[end_index] = lamda * dl[end_index-1] * curr[end_index-1] + (1 - lamda * (dl[end_index-1] + dl[end_index])) * curr[end_index] + cfg_.activity_level * (curr[end_index] - past[end_index]) - pow(cfg_.dt, 2) * k_ * cfg_.gravity;
    }

}

void SutureSimulation::_move_suture(const std::vector<DQ>& positions) const
{
    for (int i=0; i<positions.size(); ++i)
    {
        vrep_interface_.set_object_translation(particles_[i], DQ_VrepInterface::ABSOLUTE_FRAME, positions[i], DQ_VrepInterface::OP_ONESHOT);
    }
}

bool SutureSimulation::_should_shutdown() const
{
    return (*kill_this_node_);
}

CollisionCheckMaterialBox::CollisionCheckMaterialBox(DQ_VrepInterface& vi, const std::string suffix):
vi_(vi)
{
    t_handle_ = vi_.get_object_handle("x" + suffix);
    ta_handle_ = vi_.get_object_handle("a" + suffix);
    te_handle_ = vi_.get_object_handle("e" + suffix);
    tf_handle_ = vi_.get_object_handle("f" + suffix);
    tg_handle_ = vi_.get_object_handle("g" + suffix);
    tb_handle_ = vi_.get_object_handle("b" + suffix);
    tc_handle_ = vi_.get_object_handle("c" + suffix);
    bed_handle_ = vi_.get_object_handle("bed");
}

void CollisionCheckMaterialBox::update()
{
    t = vi_.get_object_translation(t_handle_, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_ONESHOT);
    ta = vi_.get_object_translation(ta_handle_, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_ONESHOT);
    te = vi_.get_object_translation(te_handle_, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_ONESHOT);
    tf = vi_.get_object_translation(tf_handle_, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_ONESHOT);
    tg = vi_.get_object_translation(tg_handle_, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_ONESHOT);
    tb = vi_.get_object_translation(tb_handle_, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_ONESHOT);
    tc = vi_.get_object_translation(tc_handle_, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_ONESHOT);
    link9.reset(t, ta);
    link7.reset(tb, tc);
    bed = k_ + E_ * dot(vi_.get_object_translation(bed_handle_, DQ_VrepInterface::ABSOLUTE_FRAME, DQ_VrepInterface::OP_ONESHOT), k_);
}

CollisionCheckMaterialBox& CollisionCheckMaterialBox::operator=(const CollisionCheckMaterialBox& cc)
{
    t = cc.t;
    ta = cc.ta;
    tb = cc.tb;
    tc = cc.tc;
    te = cc.te;
    tf = cc.tf;
    tg = cc.tg;
    link9 = cc.link9;
    link7 = cc.link7;
    bed = cc.bed;
    return *this;
}