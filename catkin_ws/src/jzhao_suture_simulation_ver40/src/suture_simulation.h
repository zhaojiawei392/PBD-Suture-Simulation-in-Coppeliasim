#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include "line_segment.h"
#include "suture.h"
#include "eigen_vrep_interface.h"


namespace pbd
{
struct SutureSimulationConfiguration
{
    std::string suture_parameters_file;
    std::string vrep_ip;
    int vrep_port;
    int realization_iteration_times;
    int continuity_iteration_times;

    double dt;
    double gravity;
    double eta_d;
    double link7_radius;
    double link8_radius;
    double link9_radius;
    std::string head_name;
    std::string end_name;
    std::string element_name_prefix;
    double vfi_threshold;
    int self_collision_check_threshold;
    double bed_safe_distance;
    bool collision_switch;
    bool self_collision_switch;
    bool penetration_avoidance;
    double penetration_check_threshold;
};

class SutureSimulation
{

private:
    Eigen_VrepInterface eigen_vrep_;
    Suture curr_suture_;
    Suture future_suture_;
    enum class EndAttachment {attached, released};

    EndAttachment head_attachment_;
    EndAttachment end_attachment_;    
    volatile bool enabled_;
    SutureSimulationConfiguration cfg_;
    std::atomic_bool* kill_this_node_;
    std::vector<int> particle_handles_;

    class CollisionCheckMaterialBox
    {
    private:
        Eigen_VrepInterface& eigen_vrep_;
        int t_handle_;
        int ta_handle_;
        int te_handle_;
        int tf_handle_;
        int tg_handle_;
        int tb_handle_;
        int tc_handle_;
    public:
        LineSegment a;
        LineSegment a_e;
        LineSegment e_f;
        LineSegment f_g;
        LineSegment g_b;
        LineSegment b_c;

        CollisionCheckMaterialBox(Eigen_VrepInterface& ev, const std::string& suffix);
        CollisionCheckMaterialBox(const CollisionCheckMaterialBox&)=default;
        CollisionCheckMaterialBox& operator=(const CollisionCheckMaterialBox&);
        void update();
    };
    
    bool _should_shutdown() const;

    bool _check_point_collision(const Eigen::Vector3d& position, const Eigen::Vector3d& obstacle, const double& safe_distance) const;
    bool _check_line_segment_collision(const Eigen::Vector3d& position, const pbd::LineSegment& obstacle, const double& safe_distance) const;
    bool _check_plane_collision(const Eigen::Vector3d& position, const Eigen::Vector3d& obstacle, const double& safe_distance) const;
    bool _check_collision(const Eigen::Vector3d& position, const CollisionCheckMaterialBox& cc) const;

    void _continuity(std::vector<Eigen::Vector3d>& positions, const int count) const;
    void _collision_avoidance(std::vector<Eigen::Vector3d>& future, const std::vector<Eigen::Vector3d>& curr, const CollisionCheckMaterialBox& cc, const CollisionCheckMaterialBox& past_cc) const;
    void _penetration_avoidance(std::vector<Eigen::Vector3d>& future, const std::vector<Eigen::Vector3d>& curr, const CollisionCheckMaterialBox& cc, const CollisionCheckMaterialBox& past_cc) const;
    void _self_collision_avoidance(std::vector<Eigen::Vector3d>& future, const std::vector<Eigen::Vector3d>& curr) const;

    void _calculate_future_positions(std::vector<Eigen::Vector3d>& future, const std::vector<Eigen::Vector3d>& curr, const std::vector<Eigen::Vector3d>& past, const CollisionCheckMaterialBox& cc1, const CollisionCheckMaterialBox& cc2) const;
    void _move_suture(const std::vector<Eigen::Vector3d>& positions) const;

    void _point_collision_avoidance(Eigen::Vector3d& future_point, const Eigen::Vector3d& curr_point, const Eigen::Vector3d& ws_point, const Eigen::Vector3d& past_ws_point, const double& safe_distance) const;
    void _line_segment_collision_avoidance(Eigen::Vector3d& future_point, const Eigen::Vector3d& curr_point, const pbd::LineSegment& ws_ls, const pbd::LineSegment& past_ws_ls, const double& safe_distance) const;
    void _plane_collision_avoidance(Eigen::Vector3d& future_point, const Eigen::Vector3d& curr_point, const Eigen::Vector3d& plane, const Eigen::Vector3d& past_plane, const double& safe_distance) const;
    bool _check_penetration(const pbd::LineSegment& future_line1, const pbd::LineSegment& curr_line1, const pbd::LineSegment& future_line2, const pbd::LineSegment& curr_line2) const;
    bool _self_penetration_avoidance(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3, Eigen::Vector3d& p4, const pbd::LineSegment& future_line1, const pbd::LineSegment& curr_line1, const pbd::LineSegment& future_line2, const pbd::LineSegment& curr_line2, const double& safe_distance) const;

public:
    SutureSimulation()=delete;
    SutureSimulation(const SutureSimulationConfiguration& cfg,
                    std::atomic_bool* kill_this_node);
    bool enabled() const;
    void initialize();
    void control_loop();
};
}
