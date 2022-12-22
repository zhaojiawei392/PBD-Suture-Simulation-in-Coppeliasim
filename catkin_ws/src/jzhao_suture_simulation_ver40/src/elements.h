#pragma once
#include <eigen3/Eigen/Dense>


namespace pbd
{
class Attachment;
class Point;
class Sphere;
class Particle;
class VelocityParticle;
class Line;
class Cylinder;
class Capsule;
class MassCylinder;
class MassCapsule;
class VelocityMassCapsule;


class Attachment
{
protected:
    enum {attached, released}attachment_; 
public:
    Attachment();

    void reset();
    
    void set_attachment();
    void release();
    bool is_attached() const;
    bool is_released() const;
    int attachment() const;
}

class Point
{
protected:
    Eigen::Vector3d current_position_;
public:
    Point();
    Point(const Eigen::Vector3d& position, const bool& is_attached=false);

    void reset();
    void reset(const Eigen::Vector3d& position, const bool& is_attached=false);

    void set_position(const Eigen::Vector3d& position);
    void set_position(const Point& point);

    Eigen::Vector3d position() const;

    void dodge(const Eigen::Vector3d& position, const double& position_radius);
    void dodge(const Point& point, const double& point_radius);
    void dodge(const Line& line, const double& line_radius);
    void dodge(const Cylinder& cylinder);
    void dodge(const Capsule& capsule);

    Eigen::Vector3d operator+ (const Eigen::Vector3d& position) const;
    Eigen::Vector3d operator+ (const Point& point) const;
    Eigen::Vector3d operator- (const Eigen::Vector3d& position) const;
    Eigen::Vector3d operator- (const Point& point) const;
    void operator+= (const Eigen::Vector3d& position);
    void operator+= (const Point& point);
    void operator-= (const Eigen::Vector3d& position);
    void operator-= (const Point& point);

    Eigen::Vector3d point_projected_in_line(const Line& line);
};

class Sphere: public Point
{
protected:
    double radius_;
public:
    Sphere(const double& radius=0);
    Sphere(const Eigen::Vector3d& position, const double& radius=0, const bool& is_attached=false);

    void reset(const double& radius=0);
    void reset(const Eigen::Vector3d& position, const double& radius=0, const bool& is_attached=false);

    void set_radius(const double& radius);
    void set_radius(double&& radius);
    double radius() const;

    virtual void dodge(const Eigen::Vector3d& position, const double& position_radius);
    virtual void dodge(const Point& point, const double& point_radius);
    virtual void dodge(const Line& line, const double& line_radius);
    virtual void dodge(const Cylinder& cylinder);
    virtual void dodge(const Capsule& capsule);
    void dodge(const Sphere& sphere);

}

class Particle:public Sphere 
{
protected:
    double mass_;
public:
    Particle(const double& mass=0, const double& radius=0);
    Particle(const Eigen::Vector3d& position, const double& mass=0, const double& radius=0, const bool& is_attached=false);

    void reset(const double& radius=0, const double& mass=0);
    void reset(const Eigen::Vector3d& position, const double& mass=0, const double& radius=0, const bool& is_attached=false);

    void set_mass(const double& mass);
    void set_mass(double&& mass);
    double mass() const;
};

class VelocityParticle: public Attachment
{
private: 
    Eigen::Vector3d velocity_;
    Particle current_particle_;
    Particle past_particle_;
public:
    VelocityParticle(const double& mass=0, const double& radius=0);
    VelocityParticle(const Eigen::Vector3d& position, const double& mass=0, const double& radius=0, const bool& is_attached=false);

    void reset(const double& radius=0, const double& mass=0);
    void reset(const Eigen::Vector3d& position, const double& mass=0, const double& radius=0, const bool& is_attached=false);

    void initialize(const Eigen::Vector3d& position);
    void initialize(const Point& point);

    void update_position_and_velocity(const Eigen::Vector3d& position);
    void update_position_and_velocity(const Point& point);
    void update_position_and_velocity(const VelocityParticle& vel_particle);

    Particle& current() const;
    Particle& past() const;
    Eigen::Vector3d velocity() const;

    void dodge(const Eigen::Vector3d& position, const double& position_radius);
    void dodge(const Point& point, const double& point_radius);
    void dodge(const Line& line, const double& line_radius);
    void dodge(const Cylinder& cylinder);
    void dodge(const Capsule& capsule);
    void dodge(const Sphere& sphere);

    void vector_dodge(const VelocityParticle& vel_particle, const double& threshold=1., const double& gain=1.);

    Eigen::Vector3d operator+ (const Eigen::Vector3d& position) const;
    Eigen::Vector3d operator+ (const Point& point) const;
    Eigen::Vector3d operator+ (const VelocityParticle& vel_particle) const;
    Eigen::Vector3d operator- (const Eigen::Vector3d& position) const;
    Eigen::Vector3d operator- (const Point& point) const;
    Eigen::Vector3d operator- (const VelocityParticle& vel_particle) const;
    void operator+= (const Eigen::Vector3d& position);
    void operator+= (const Point& point);
    void operator+= (const VelocityParticle& vel_particle);
    void operator-= (const Eigen::Vector3d& position);
    void operator-= (const Point& point);
    void operator-= (const VelocityParticle& vel_particle);

    Eigen::Vector3d point_projected_in_line(const Line& line);
};

class Line
{
protected:
    Eigen::Vector3d direction_;
    Eigen::Vector3d dual_part_;
public:
    Line(const Eigen::Vector3d& line_vector, const Eigen::Vector3d& point_in_line);

    void reset(const Eigen::Vector3d& line_vector, const Eigen::Vector3d& point_in_line);

    Eigen::Vector3d direction() const;
    Eigen::Vector3d dual_part() const;
    Eigen::Vector3d center_point() const;
    Eigen::Vector3d projection_point_of(const Eigen::Vector3d& position) const;
    Eigen::Vector3d projection_point_of(const Point& point) const;
    Eigen::Vector3d projection_point_of(const VelocityParticle& vel_particle) const;
    static Eigen::Vector3d point_projected_in_line(const Eigen::Vector3d position, const Line& line);
    friend Eigen::Vector3d point_projected_in_line(const Point& point, const Line& line);
    friend Eigen::Vector3d point_projected_in_line(const VelocityParticle& vel_particle, const Line& line);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> closest_points_with(const Line& line2);

    virtual double distance_from_point(const Eigen::Vector3d& point) const;
    Eigen::Vector3d operator-(const Line& line) const;

    static Eigen::Vector3d projection_point_in_line(const Eigen::Vector3d& point, const Line& line);
    static std::tuple<Eigen::Vector3d, Eigen::Vector3d> closest_points_between(const Line& line1, const Line& line2);
};

class Cylinder: public Line
{
protected:
    double radius_;
public:
    Cylinder(const Eigen::Vector3d& line_vector, const Eigen::Vector3d& point_in_line, const double& radius);

    void reset(const Eigen::Vector3d& line_vector, const Eigen::Vector3d& point_in_line, const double& radius);

    void set_radius(const double& radius);
    void set_radius(double&& radius);
    double radius() const;
}
    

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