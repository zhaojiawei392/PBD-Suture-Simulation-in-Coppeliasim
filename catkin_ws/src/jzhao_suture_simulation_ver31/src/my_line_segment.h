#pragma once
#include <dqrobotics/DQ.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
using namespace DQ_robotics;
using namespace Eigen;


namespace MLS
{
    class MyLineSegment
    {
    private:
        DQ line_direction_;
        DQ dual_part_;
        DQ point1_;
        DQ point2_;
        double length_;
    public:
        MyLineSegment();
        MyLineSegment(const MyLineSegment& ls)=default;
        MyLineSegment(MyLineSegment&& ls)=default;
        MyLineSegment(const DQ& line_direction, const DQ& point1, const DQ& point2);
        MyLineSegment(const DQ& point1, const DQ& point2);
        void reset(const DQ& line_direction, const DQ& point1, const DQ& point2);
        void reset(const DQ& point1, const DQ& point2);
        DQ direction() const;
        DQ dual_part() const;
        DQ first_point() const;
        DQ second_point() const;
        DQ projection_point(const DQ&) const;
        bool is_projected_in_segment(const DQ& point) const;
        bool is_cross(const MyLineSegment& line2) const;
        double distance(const DQ& point) const;
        double length() const;
        DQ operator-(const MyLineSegment& ls) const;
        MyLineSegment& operator=(const MyLineSegment&)=default;
    };
    enum class LineState{line, point1, point2};
    DQ projection_point_on_line(const DQ& point, const MyLineSegment& line_segment);
    bool is_point_on_line_segment(const DQ& point, const MyLineSegment& line_segment);
    
    std::tuple<DQ, DQ> closest_points_between_lines(const DQ& line1, const DQ& line2);
    std::tuple<DQ, DQ> closest_points_between_lines(const MyLineSegment& line1, const MyLineSegment& line2);
    std::tuple<LineState, LineState> check_line_states(const DQ& line1, const DQ& line1_point1, const DQ& line1_point2, const DQ& line2, const DQ& line2_point1, const DQ& line2_point2);
    std::tuple<LineState, LineState> check_line_states(const MyLineSegment& line1, const MyLineSegment& line2);
};




