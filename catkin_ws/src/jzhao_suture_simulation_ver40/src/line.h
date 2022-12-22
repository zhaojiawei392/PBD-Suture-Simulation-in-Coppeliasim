#pragma once
#include <eigen3/Eigen/Dense>
#include "point"

namespace pbd
{
class Line
{
private:
    Eigen::Vector3d center_point_;
protected:
    Eigen::Vector3d direction_;
    Eigen::Vector3d dual_part_;
    double radius_;
public:
    Line(const Eigen::Vector3d& line_vector, const Eigen::Vector3d& point_in_line, const double& radius=0);

    void reset(const Eigen::Vector3d& line_vector, const Eigen::Vector3d& point_in_line, const double& radius=0);
    
    void set_radius(const double& radius);
    double radius() const;

    Eigen::Vector3d direction() const;
    Eigen::Vector3d dual_part() const;
    Eigen::Vector3d center_point() const;
    Eigen::Vector3d projection_point(const Eigen::Vector3d&) const;

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> closest_points_with(const Line& line2);

    virtual double distance_from_point(const Eigen::Vector3d& point) const;
    Eigen::Vector3d operator-(const Line& line) const;

    static Eigen::Vector3d projection_point_in_line(const Eigen::Vector3d& point, const Line& line);
    static std::tuple<Eigen::Vector3d, Eigen::Vector3d> closest_points_between(const Line& line1, const Line& line2);
};
    

class LineSegment:public Line
{
public:
    enum class LineSegmentState{line, first_point, second_point};
protected:
    Eigen::Vector3d first_point_;
    Eigen::Vector3d second_point_;
    double length_;
    
public:
    LineSegment(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point, const double& radius=0);

    void reset(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point, const double& radius=0);
    void reset(const LineSegment& line_segment);

    double length() const;

    Eigen::Vector3d first_point() const;
    Eigen::Vector3d second_point() const;

    Eigen::Vector3d closest_point_with(const Eigen::Vector3d& position);
    LineSegmentState check_line_state_with(const Eigen::Vector3d& position);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> closest_points_with(const LineSegment& line_segment2);
    std::tuple<LineSegmentState, LineSegmentState> check_line_states_with(const LineSegment& line_segment2);


    bool is_point_projected_in_segment(const Eigen::Vector3d& point) const;
    bool is_overlapping(const LineSegment& line_segment2) const;
    
    Eigen::Vector3d operator-(const LineSegment& line_segment) const;
    
    virtual double distance_from_point(const Eigen::Vector3d& point) const;

    static Eigen::Vector3d projection_point_in_line(const Eigen::Vector3d& point, const LineSegment& line_segment);
    static bool is_point_projected_in_segment(const Eigen::Vector3d& point, const LineSegment& line_segment);
    static std::tuple<Eigen::Vector3d, Eigen::Vector3d> closest_points_between(const LineSegment& line_segment1, const LineSegment& line_segment2);
    static std::tuple<LineSegmentState, LineSegmentState> check_line_states_between(const LineSegment& line_segment1, const LineSegment& line_segment2);
};


class VelocityLineSegment
{
private:
    Eigen::Vector3d first_point_velocity_;
    Eigen::Vector3d second_point_velocity_;
    LineSegment current_line_segment_;
    LineSegment past_line_segment_;
public:
    VelocityLineSegment(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point, const double& radius=0);
    VelocityLineSegment(const LineSegment& line_segment);

    void reset(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point, const double& radius=0);
    void reset(const LineSegment& line_segment);
    void reset(const VelocityLineSegment& vel_line_segment);

    void update(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point);
    void update(const LineSegment& line_segment);
    void update(const VelocityLineSegment& vel_line_segment);

    double radius() const;
    LineSegment& current_line_segment() const;
    LineSegment& past_line_segment() const;  
};


};