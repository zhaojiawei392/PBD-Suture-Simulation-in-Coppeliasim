#include <string>
#include <vector>
#include <memory>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <eigen3/Eigen/Dense>
#include "my_line_segment.h"

using namespace DQ_robotics;
using namespace Eigen;
using MLS::MyLineSegment;

struct SutureSimulationConfiguration
{
    bool frequency_verbose;
    std::string vrep_ip;
    int vrep_port;
    int mass_amount;

    int realization_iteration_times;
    int continuity_iteration_times;
    int continuity_iteration_times_between_collision_avoidance;

    double forward_priority;
    double original_distance;
    double continuity_distance;
    double elastic_modulus;
    double activity_level;
    double mass;
    double dt;
    double gravity;
    double eta_d;
    double eta_d_ls;
    double eta_d_self;
    double link7_radius;
    double link8_radius;
    double link9_radius;
    std::string head_name;
    std::string end_name;
    std::string particle_name;
    double vfi_threshold;
    int thread_sampling_time_nsec;
    int inverse_sending_speed;
    int self_collision_check_threshold;
    double bed_safe_distance;
    bool collision_switch;
    bool self_collision_switch;
    bool penetration_avoidance;
    double penetration_check_threshold;
};

class CollisionCheckMaterialBox
{
private:
    DQ_VrepInterface& vi_;
    int t_handle_;
    int ta_handle_;
    int te_handle_;
    int tf_handle_;
    int tg_handle_;
    int tb_handle_;
    int tc_handle_;
    int bed_handle_;
public:
    DQ t;
    DQ ta;
    DQ te;
    DQ tf;
    DQ tg;
    DQ tb;
    DQ tc;
    MyLineSegment link9;
    MyLineSegment link7;
    DQ bed;
    CollisionCheckMaterialBox(DQ_VrepInterface& vi, const std::string suffix);
    CollisionCheckMaterialBox(const CollisionCheckMaterialBox&)=default;
    CollisionCheckMaterialBox& operator=(const CollisionCheckMaterialBox&);
    void update();
};

class SutureSimulation
{

private:
    enum class EndAttachment {attached, released};

    EndAttachment head_attachment_;
    EndAttachment end_attachment_;    
    volatile bool enabled_;
    SutureSimulationConfiguration cfg_;
    std::atomic_bool* kill_this_node_;
    DQ_VrepInterface vrep_interface_;
    std::vector<int> particles_;

    bool _should_shutdown() const;

    bool _check_point_collision(const DQ& position, const DQ& obstacle, const double& safe_distance) const;
    bool _check_line_segment_collision(const DQ& position, const MyLineSegment& obstacle, const double& safe_distance) const;
    bool _check_plane_collision(const DQ& position, const DQ& obstacle, const double& safe_distance) const;
    bool _check_collision(const DQ& position, const CollisionCheckMaterialBox& cc) const;

    void _continuity(std::vector<DQ>& positions, const int count) const;
    void _collision_avoidance(std::vector<DQ>& future, const std::vector<DQ>& curr, const CollisionCheckMaterialBox& cc, const CollisionCheckMaterialBox& past_cc) const;
    void _penetration_avoidance(std::vector<DQ>& future, const std::vector<DQ>& curr, const CollisionCheckMaterialBox& cc, const CollisionCheckMaterialBox& past_cc) const;
    void _self_collision_avoidance(std::vector<DQ>& future, const std::vector<DQ>& curr) const;

    void _calculate_future_positions(std::vector<DQ>& future, const std::vector<DQ>& curr, const std::vector<DQ>& past, const CollisionCheckMaterialBox& cc1, const CollisionCheckMaterialBox& cc2) const;
    void _move_suture(const std::vector<DQ>& positions) const;

    void _point_collision_avoidance(DQ& future_point, const DQ& curr_point, const DQ& ws_point, const DQ& past_ws_point, const double& safe_distance) const;
    void _line_segment_collision_avoidance(DQ& future_point, const DQ& curr_point, const MyLineSegment& ws_ls, const MyLineSegment& past_ws_ls, const double& safe_distance) const;
    void _plane_collision_avoidance(DQ& future_point, const DQ& curr_point, const DQ& plane, const DQ& past_plane, const double& safe_distance) const;
    bool _check_penetration(const MyLineSegment& future_line1, const MyLineSegment& curr_line1, const MyLineSegment& future_line2, const MyLineSegment& curr_line2) const;
    bool _self_penetration_avoidance(DQ& p1, DQ& p2, DQ& p3, DQ& p4, const MyLineSegment& future_line1, const MyLineSegment& curr_line1, const MyLineSegment& future_line2, const MyLineSegment& curr_line2, const double& safe_distance) const;

public:
    enum motion_mode{free_end, fixed_end};
    SutureSimulation()=delete;
    SutureSimulation(SutureSimulationConfiguration cfg, 
                    std::atomic_bool* kill_this_node);
    bool enabled() const;
    void initialize();
    void control_loop();
};