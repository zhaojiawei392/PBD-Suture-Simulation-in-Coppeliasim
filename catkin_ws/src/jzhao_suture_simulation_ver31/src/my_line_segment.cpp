#include "my_line_segment.h"
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/utils/DQ_Geometry.h>
namespace MLS
{
    MyLineSegment::MyLineSegment()
    {
    }
    MyLineSegment::MyLineSegment(const DQ& line_direction, const DQ& point1, const DQ& point2)
    {
        // if (int(cross(point1 - point2, line_direction)))
        //     throw std::runtime_error(ros::this_node::getName() + "::wrong MyLineSegment arguments");
        length_ = vec3(line_direction).norm();
        line_direction_ = line_direction * (1 / length_);
        point1_ = point1;
        point2_ = point2;
        dual_part_ = cross(point1_, line_direction_);
    }
    MyLineSegment::MyLineSegment(const DQ& point1, const DQ& point2)
    {
        length_ = vec3(point1 - point2).norm();
        line_direction_ = (point1 - point2) * (1 / length_);
        point1_ = point1;
        point2_ = point2;
        dual_part_ = cross(point1_, line_direction_);
    }
    void MyLineSegment::reset(const DQ& line_direction, const DQ& point1, const DQ& point2)
    {
        length_ = vec3(line_direction).norm();
        line_direction_ = line_direction * (1 / length_);
        point1_ = point1;
        point2_ = point2;
        dual_part_ = cross(point1_, line_direction_);
    }
    void MyLineSegment::reset(const DQ& point1, const DQ& point2)
    {
        length_ = vec3(point1 - point2).norm();
        line_direction_ = (point1 - point2) * (1 / length_);
        point1_ = point1;
        point2_ = point2;
        dual_part_ = cross(point1_, line_direction_);
    }
    DQ MyLineSegment::direction() const
    {
        return line_direction_;
    }
    DQ MyLineSegment::dual_part() const
    {
        return dual_part_;
    }
    DQ MyLineSegment::first_point() const
    {
        return point1_;
    }
    DQ MyLineSegment::second_point() const
    {
        return point2_;
    }
    DQ MyLineSegment::projection_point(const DQ& workspace_point) const
    {
        return point1_ + dot(workspace_point - point1_, line_direction_) * line_direction_;
    }

    bool MyLineSegment::is_projected_in_segment(const DQ& point) const
    {
        const DQ& pp = projection_point(point);

        const double& d_pp_p1 = vec3(pp - point1_).norm();
        const double& d_pp_p2 = vec3(pp - point2_).norm();
        bool is_point_in_line;
        if (d_pp_p1 <= length_ && d_pp_p2 <= length_)
        {
            return true;
        }
        return false;
    }

    double MyLineSegment::distance(const DQ& point) const
    {
        if (is_projected_in_segment(point))
        {
            return vec3(point - projection_point(point)).norm();
        }
        return std::min(vec3(point - point1_).norm(), vec3(point - point2_).norm());
    }

    bool MyLineSegment::is_cross(const MyLineSegment& line2) const
    {
        DQ cp1;
        DQ cp2;
        std::tie(cp1, cp2) = closest_points_between_lines(*this, line2);    
        const double& d_cp1_p1 = vec3(cp1 - point1_).norm();
        const double& d_cp1_p2 = vec3(cp1 - point2_).norm();
        const double& d_cp2_p1 = vec3(cp2 - line2.point1_).norm();
        const double& d_cp2_p2 = vec3(cp2 - line2.point2_).norm();
        if (d_cp1_p1 <= length_ && d_cp1_p2 <= length_ && d_cp2_p1 <= line2.length_ && d_cp2_p2 <= line2.length_)
        {
            return true;
        }
        return false;

    }

    double MyLineSegment::length() const
    {
        return length_;
    }
    DQ MyLineSegment::operator-(const MyLineSegment& ls) const
    {
        const DQ& m1 = dual_part_;
        const DQ& l1 = line_direction_;
        const DQ& m2 = ls.dual_part();
        const DQ& l2 = ls.direction();
        DQ cp1 = (cross(-1.0*m1,cross(l2,cross(l1,l2))) + dot(m2,cross(l1,l2) )*l1 )*std::pow(1./vec4(cross(l1,l2)).norm(),2);
        DQ cp2 = (cross( m2,cross(l1,cross(l1,l2))) - dot(m1,cross(l1,l2) )*l2 )*std::pow(1./vec4(cross(l1,l2)).norm(),2);
        return cp1-cp2;
    }

    DQ projection_point_on_line(const DQ& point, const MyLineSegment& line_segment)
    {
        return line_segment.projection_point(point);
    }
    bool is_point_on_line_segment(const DQ& point, const MyLineSegment& line_segment)
    {
        double line_length = line_segment.length();
        DQ projection_point = line_segment.projection_point(point);
        double d_1 = vec3(projection_point - line_segment.first_point()).norm();
        double d_2 = vec3(projection_point - line_segment.second_point()).norm();
        if (d_1 < line_length && d_2 < line_length)
            return true;
        else
            return false;
    }
    std::tuple<DQ, DQ> closest_points_between_lines(const DQ& line1, const DQ& line2)
    {
        const DQ& l1 = line1.P();
        const DQ& m1 = line1.D();
        const DQ& l2 = line2.P();
        const DQ& m2 = line2.D();

        const DQ& cp1 = (cross(-1.0*m1,cross(l2,cross(l1,l2))) + dot(m2,cross(l1,l2) )*l1 )*std::pow(1./vec4(cross(l1,l2)).norm(),2);
        const DQ& cp2 = (cross( m2,cross(l1,cross(l1,l2))) - dot(m1,cross(l1,l2) )*l2 )*std::pow(1./vec4(cross(l1,l2)).norm(),2);        
        return {cp1, cp2};
    }
    std::tuple<DQ, DQ> closest_points_between_lines(const MyLineSegment& line1, const MyLineSegment& line2)
    {
        const DQ& l1 = line1.direction();
        const DQ& m1 = line1.dual_part();
        const DQ& l2 = line2.direction();
        const DQ& m2 = line2.dual_part();

        const DQ& cp1 = (cross(-1.0*m1,cross(l2,cross(l1,l2))) + dot(m2,cross(l1,l2) )*l1 )*std::pow(1./vec4(cross(l1,l2)).norm(),2);
        const DQ& cp2 = (cross( m2,cross(l1,cross(l1,l2))) - dot(m1,cross(l1,l2) )*l2 )*std::pow(1./vec4(cross(l1,l2)).norm(),2);        
        return {cp1, cp2};
    }

    std::tuple<LineState, LineState> check_line_states(const DQ& line1, const DQ& line1_point1, const DQ& line1_point2, const DQ& line2, const DQ& line2_point1, const DQ& line2_point2)
    {
        const double& line1_length = vec3(line1_point1 - line2_point2).norm();
        const double& line2_length = vec3(line2_point1 - line2_point2).norm();
        DQ cp1;
        DQ cp2;
        std::tie(cp1, cp2) = closest_points_between_lines(line1, line2);

        const double& d_cp1_p1 = vec3(cp1 - line1_point1).norm();
        const double& d_cp1_p2 = vec3(cp1 - line1_point2).norm();
        const double& d_cp2_p1 = vec3(cp2 - line2_point1).norm();
        const double& d_cp2_p2 = vec3(cp2 - line2_point2).norm();

        LineState line1_state;
        LineState line2_state;
        if (d_cp1_p1 <= line1_length && d_cp1_p2 <= line1_length)
        {
            line1_state = LineState::line;
        }
        else if (d_cp1_p1 < d_cp1_p2)
        {
            line1_state = LineState::point1;
        }
        else 
        {
            line1_state = LineState::point2;
        }

        if (d_cp2_p1 <= line2_length && d_cp2_p2 <= line2_length)
        {
            line2_state = LineState::line;
        }
        else if (d_cp2_p1 <= d_cp2_p2)
        {
            line2_state = LineState::point1;
        }
        else
        {
            line2_state = LineState::point2;
        }
        return {line1_state, line2_state};
    }

    std::tuple<LineState, LineState> check_line_states(const MyLineSegment& line1, const MyLineSegment& line2)
    {
        DQ cp1;
        DQ cp2;
        std::tie(cp1, cp2) = closest_points_between_lines(line1, line2);    
        const double& d_cp1_p1 = vec3(cp1 - line1.first_point()).norm();
        const double& d_cp1_p2 = vec3(cp1 - line1.second_point()).norm();
        const double& d_cp2_p1 = vec3(cp2 - line2.first_point()).norm();
        const double& d_cp2_p2 = vec3(cp2 - line2.second_point()).norm();
        LineState line1_state;
        LineState line2_state;
        if (d_cp1_p1 <= line1.length() && d_cp1_p2 <= line1.length())
        {
            line1_state = LineState::line;
        }
        else if (d_cp1_p1 < d_cp1_p2)
        {
            line1_state = LineState::point1;
        }
        else 
        {
            line1_state = LineState::point2;
        }

        if (d_cp2_p1 <= line2.length() && d_cp2_p2 <= line2.length())
        {
            line2_state = LineState::line;
        }
        else if (d_cp2_p1 <= d_cp2_p2)
        {
            line2_state = LineState::point1;
        }
        else
        {
            line2_state = LineState::point2;
        }
        return {line1_state, line2_state};
    }
}

