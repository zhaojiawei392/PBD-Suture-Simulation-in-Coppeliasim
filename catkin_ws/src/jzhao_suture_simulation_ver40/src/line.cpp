#include "line.h" 

namespace pbd
{
/*************************************************************************************************************************************************
 * @brief Construct a new Line:: Line object
 * 
 * @param line_vector 
 * @param point_in_line 
 */
Line::Line(const Eigen::Vector3d& line_vector, const Eigen::Vector3d& point_in_line, const double& radius):
    radius_(radius)
{
    direction_ = line_vector.normalized();
    dual_part_ = point_in_line.cross(direction_);
    center_point_ = direction_.cross(dual_part_);
}

void Line::reset(const Eigen::Vector3d& line_vector, const Eigen::Vector3d& point_in_line, const double& radius)
{
    radius_ = radius;
    direction_ = line_vector.normalized();
    dual_part_ = point_in_line.cross(direction_);
    center_point_ = direction_.cross(dual_part_);
}

void Line::set_radius(const double& radius)
{
    radius_ = radius;
}

double Line::radius() const
{
    return radius_;
}

Eigen::Vector3d Line::direction() const
{
    return direction_;
}

Eigen::Vector3d Line::dual_part() const
{
    return dual_part_;
}

Eigen::Vector3d Line::center_point() const
{
    return center_point_;
}

Eigen::Vector3d Line::projection_point(const Eigen::Vector3d& point) const
{
    return center_point_ + (point - center_point_).dot(direction_) * direction_;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> Line::closest_points_with(const Line& line2)
{
    const Eigen::Vector3d& l1 = direction_;
    const Eigen::Vector3d& m1 = dual_part_;
    const Eigen::Vector3d& l2 = line2.direction_;
    const Eigen::Vector3d& m2 = line2.dual_part_;

    const Eigen::Vector3d& cp1 = (m2.dot(l1.cross(l2)) * l1 - m1.cross(l2.cross(l1.cross(l2)))) * std::pow(1./l1.cross(l2).norm(), 2);
    const Eigen::Vector3d& cp2 = (m2.cross(l1.cross(l1.cross(l2))) - m1.dot(l1.cross(l2)) * l2) * std::pow(1./l1.cross(l2).norm(), 2);  
    return {cp1, cp2};
}

virtual double Line::distance_from_point(const Eigen::Vector3d& point) const
{
    return (point - projection_point(point)).norm();
}

virtual Eigen::Vector3d Line::operator-(const Line& line2) const
{
    Eigen::Vector3d cp1;
    Eigen::Vector3d cp2;
    std::tie(cp1, cp2) = closest_points_with(line2);
    return cp1 - cp2;
}

Eigen::Vector3d Line::projection_point_in_line(const Eigen::Vector3d& point, const Line& line)
{
    return line.projection_point(point);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> Line::closest_points_between(const Line& line1, const Line& line2)
{
    return line1.closest_points_with(line2);
}



/*************************************************************************************************************************************************
 * @brief Construct a new Line Segment:: Line Segment object
 * 
 * @param first_point 
 * @param second_point 
 */
LineSegment::LineSegment(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point, const double& radius):
    Line(first_point - second_point, first_point, radius)
{
    length_ = (first_point - second_point).norm();
    first_point_ = first_point;
    second_point_ = second_point;
}

void LineSegment::reset(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point, const double& radius)
{
    Line::reset(first_point - second_point, first_point, radius);
    length_ = (first_point - second_point).norm();
    first_point_ = first_point;
    second_point_ = second_point;
}

void LineSegment::reset(const LineSegment& line_segment)
{
    Line::reset(line_segment.direction_, line_segment.center_point_, line_segment.radius_);
    length_ = line_segment.length_;
    first_point_ = line_segment.first_point_;
    second_point_ = line_segment.second_point_;
}

double LineSegment::length() const
{
    return length_;
}

Eigen::Vector3d LineSegment::first_point() const
{
    return first_point_;
}

Eigen::Vector3d LineSegment::second_point() const
{
    return second_point_;
}

Eigen::Vector3d LineSegment::closest_point_with(const Eigen::Vector3d& position)
{
    const Eigen::Vector3d& pp = projection_point(position);

    const double& d_pp_p1 = (pp - first_point_).norm();
    const double& d_pp_p2 = (pp - second_point_).norm();

    if (d_pp_p1 <= length_ && d_pp_p2 <= length_)
        return pp;
    else if (d_pp_p1 < d_pp_p2)
        return first_point_;
    else
        return second_point_;
}

LineSegmentState LineSegment::check_line_state_with(const Eigen::Vector3d& position)
{
    const Eigen::Vector3d& pp = projection_point(position);

    const double& d_pp_p1 = (pp - first_point_).norm();
    const double& d_pp_p2 = (pp - second_point_).norm();

    if (d_pp_p1 <= length_ && d_pp_p2 <= length_)
        return LineSegmentState::line;
    else if (d_pp_p1 < d_pp_p2)
        return LineSegmentState::first_point;
    else
        return LineSegmentState::second_point;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> LineSegment::closest_points_with(const LineSegment& line_segment2)
{
    const Eigen::Vector3d& l1 = direction_;
    const Eigen::Vector3d& m1 = dual_part_;
    const Eigen::Vector3d& l2 = line_segment2.direction_;
    const Eigen::Vector3d& m2 = line_segment2.dual_part_;

    Eigen::Vector3d cp1 = (m2.dot(l1.cross(l2)) * l1 - m1.cross(l2.cross(l1.cross(l2)))) * std::pow(1./l1.cross(l2).norm(), 2);
    Eigen::Vector3d cp2 = (m2.cross(l1.cross(l1.cross(l2))) - m1.dot(l1.cross(l2)) * l2) * std::pow(1./l1.cross(l2).norm(), 2);

    const double& d_cp1_p1 = (cp1 - first_point_).norm();
    const double& d_cp1_p2 = (cp1 - second_point_).norm();
    const double& d_cp2_p1 = (cp2 - line_segment2.first_point_).norm();
    const double& d_cp2_p2 = (cp2 - line_segment2.second_point_).norm();
    if (d_cp1_p1 <= length_ && d_cp1_p2 <= length_)
    {
    }
    else if (d_cp1_p1 < d_cp1_p2)
    {
        cp1 = first_point_;
    }
    else 
    {
        cp1 = second_point_;
    }

    if (d_cp2_p1 <= line_segment2.length_ && d_cp2_p2 <= line_segment2.length_)
    {
    }
    else if (d_cp2_p1 <= d_cp2_p2)
    {
        cp2 = line_segment2.first_point_;
    }
    else
    {
        cp2 = line_segment2.second_point_;
    }     
    return {cp1, cp2};
}

std::tuple<LineSegmentState, LineSegmentState> LineSegment::check_line_states_with(const LineSegment& line_segment2)
{
    const Eigen::Vector3d& l1 = direction_;
    const Eigen::Vector3d& m1 = dual_part_;
    const Eigen::Vector3d& l2 = line_segment2.direction_;
    const Eigen::Vector3d& m2 = line_segment2.dual_part_;

    const Eigen::Vector3d& cp1 = (m2.dot(l1.cross(l2)) * l1 - m1.cross(l2.cross(l1.cross(l2)))) * std::pow(1./l1.cross(l2).norm(), 2);
    const Eigen::Vector3d& cp2 = (m2.cross(l1.cross(l1.cross(l2))) - m1.dot(l1.cross(l2)) * l2) * std::pow(1./l1.cross(l2).norm(), 2);

    const double& d_cp1_p1 = (cp1 - line_segment1.first_point_).norm();
    const double& d_cp1_p2 = (cp1 - line_segment1.second_point_).norm();
    const double& d_cp2_p1 = (cp2 - line_segment2.first_point_).norm();
    const double& d_cp2_p2 = (cp2 - line_segment2.second_point_).norm();
    LineSegmentState line_segment1_state;
    LineSegmentState line_segment2_state;

    if (d_cp1_p1 <= line_segment1.length_ && d_cp1_p2 <= line_segment1.length_)
    {
        line_segment1_state = LineSegmentState::line;
    }
    else if (d_cp1_p1 < d_cp1_p2)
    {
        line_segment1_state = LineSegmentState::first_point;
    }
    else 
    {
        line_segment1_state = LineSegmentState::second_point;
    }

    if (d_cp2_p1 <= line_segment2.length_ && d_cp2_p2 <= line_segment2.length_)
    {
        line_segment2_state = LineSegmentState::line;
    }
    else if (d_cp2_p1 <= d_cp2_p2)
    {
        line_segment2_state = LineSegmentState::first_point;
    }
    else
    {
        line_segment2_state = LineSegmentState::second_point;
    }
    return {line_segment1_state, line_segment2_state};
}

bool LineSegment::is_point_projected_in_segment(const Eigen::Vector3d& point) const
{
    const Eigen::Vector3d& pp = projection_point(point);

    const double& d_pp_p1 = (pp - first_point_).norm();
    const double& d_pp_p2 = (pp - second_point_).norm();

    return d_pp_p1 <= length_ && d_pp_p2 <= length_;
}

double LineSegment::distance_from_point(const Eigen::Vector3d& point) const
{
    if (is_point_projected_in_segment(point))
    {
        return (point - projection_point(point)).norm();
    }
    return std::min((point - first_point_).norm(), (point - second_point_).norm());
}

bool LineSegment::is_overlapping(const LineSegment& line_segment2) const
{
    LineSegmentState line_segment1_state;
    LineSegmentState line_segment2_state;  
    std::tie(line_segment1_state, line_segment2_state) = check_line_states_with(line_segment2); 

    return line_segment1_state == LineSegmentState::line && line_segment2_state == LineSegmentState::line;
}

Eigen::Vector3d LineSegment::operator-(const LineSegment& line_segment2) const
{
    Eigen::Vector3d cp1;
    Eigen::Vector3d cp2;
    std:tie(cp1, cp2) = closest_points_with(line_segment2); 
    return cp1 - cp2;
}

Eigen::Vector3d LineSegment::projection_point_in_line(const Eigen::Vector3d& point, const LineSegment& line_segment)
{
    return line_segment.projection_point(point);
}

bool LineSegment::is_point_projected_in_segment(const Eigen::Vector3d& point, const LineSegment& line_segment)
{
    const Eigen::Vector3d& projection_point = line_segment.projection_point(point);
    const double& d_1 = (projection_point - line_segment.first_point_).norm();
    const double& d_2 = (projection_point - line_segment.second_point_).norm();

    return d_1 <= line_segment.length_ && d_2 <= line_segment.length_;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> LineSegment::closest_points_between(const LineSegment& line_segment1, const LineSegment& line_segment2)
{
    return line_segment1.closest_points_with(line_segment2);
}

std::tuple<LineSegmentState, LineSegmentState> LineSegment::check_line_states_between(const LineSegment& line_segment1, const LineSegment& line_segment2)
{
    return line_segment1.check_line_states_with(line_segment2);
}



/*************************************************************************************************************************************************
 * @brief Construct a new Velocity Line Segment:: Velocity Line Segment object
 * 
 * @param first_point 
 */
VelocityLineSegment::VelocityLineSegment(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point, const double& radius):
    current_line_segment_(first_point, second_point, radius),
    past_line_segment_(first_point, second_point, radius)
{
    first_point_velocity_ = Eigen::Vector3d(0,0,0);
    second_point_velocity_ = Eigen::Vector3d(0,0,0);
}

VelocityLineSegment::VelocityLineSegment(const LineSegment& line_segment):
    current_line_segment_(line_segment),
    past_line_segment_(line_segment)
{
    first_point_velocity_ = Eigen::Vector3d(0,0,0);
    second_point_velocity_ = Eigen::Vector3d(0,0,0);
}

void VelocityLineSegment::reset(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point, const double& radius)
{
    current_line_segment_.reset(first_point, second_point, radius);
    past_line_segment_.reset(first_point, second_point, radius);
    first_point_velocity_ = Eigen::Vector3d(0,0,0);
    second_point_velocity_ = Eigen::Vector3d(0,0,0);
}

void VelocityLineSegment::reset(const LineSegment& line_segment)
{
    current_line_segment_.reset(line_segment);
    past_line_segment_.reset(line_segment);
    first_point_velocity_ = Eigen::Vector3d(0,0,0);
    second_point_velocity_ = Eigen::Vector3d(0,0,0);
}

void VelocityLineSegment::reset(const VelocityLineSegment& vel_line_segment)
{
    current_line_segment_.reset(vel_line_segment.current_line_segment_);
    past_line_segment_.reset(vel_line_segment.past_line_segment_);
    first_point_velocity_ = Eigen::Vector3d(0,0,0);
    second_point_velocity_ = Eigen::Vector3d(0,0,0);
}

void VelocityLineSegment::update(const Eigen::Vector3d& first_point, const Eigen::Vector3d& second_point)
{
    past_first_point_ = first_point_;
    past_second_point_ = second_point_;
    first_point_ = first_point;
    second_point_ = second_point;
    first_point_velocity_ = first_point - past_line_segment.first_point();
    second_point_velocity_ = second_point - past_line_segment.second_point();
}

void VelocityLineSegment::update(const LineSegment& line_segment)
{
    past_first_point_ = first_point_;
    past_second_point_ = second_point_;
    first_point_ = line_segment.first_point_;
    second_point_ = line_segment.second_point_;
    first_point_velocity_ = first_point_ - past_first_point_;
    second_point_velocity_ = second_point_ - past_second_point_;
}

void VelocityLineSegment::update(const VelocityLineSegment& vel_line_segment)
{
    past_first_point_ = first_point_;
    past_second_point_ = second_point_;
    first_point_ = vel_line_segment.first_point_;
    second_point_ = vel_line_segment.second_point_;
    first_point_velocity_ = first_point_ - past_first_point_;
    second_point_velocity_ = second_point_ - past_second_point_;
}

double VelocityLineSegment::radius() const
{
    return current_line_segment_.radius();
}

LineSegment& VelocityLineSegment::current_line_segment() const
{
    return current_line_segment_;
}

LineSegment& VelocityLineSegment::past_line_segment() const
{
    return past_line_segment_;
}


}
