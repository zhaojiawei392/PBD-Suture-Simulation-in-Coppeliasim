#pragma once
#include "line.h"
#include <eigen3/Eigen/Dense>


namespace pbd
{

class Point
{
protected:
    enum {attached, released}attachment_;
    Eigen::Vector3d current_position_;
public:
    Point();
    Point(const Eigen::Vector3d& position, const bool& is_attached=false);

    void reset();
    void reset(const Eigen::Vector3d& position, const bool& is_attached=false);

    void set_position(const Eigen::Vector3d& position);
    void set_position(const Point& point);
    void attach_to(const Eigen::Vector3d& position);
    void attach_to(const Point& point);
    void release();

    Eigen::Vector3d position() const;
    std::string is_attached() const;
    int attachment() const;

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

};

class Sphere: public Point
{
protected:
    double radius_;
public:
    Sphere(const double& radius=0);
    Sphere(const Eigen::Vector3d& position, const bool& is_attached=false, const double& radius=0);

    void reset(const double& radius=0);
    void reset(const Eigen::Vector3d& position, const bool& is_attached=false, const double& radius=0);

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
    Particle(const double& radius=0, const double& mass=0);
    Particle(const Eigen::Vector3d& position, const bool& is_attached=false, const double& radius=0, const double& mass=0);

    void reset(const double& radius=0, const double& mass=0);
    void reset(const Eigen::Vector3d& position, const bool& is_attached=false, const double& radius=0, const double& mass=0);

    void set_mass(const double& mass);
    void set_mass(double&& mass);
    double mass() const;
};



class VelocityParticle
{
private: 
    Eigen::Vector3d velocity_;
    Partcile current_particle_;
    Particle past_particle_;
public:
    VelocityParticle(const double& mass=0, const double& radius=0);

    virtual void initialize(const Eigen::Vector3d& position);

    void update_position_and_velocity(const Eigen::Vector3d& position);
    void update_position_and_velocity(const VelocityParticle& vel_particle);

    Eigen::Vector3d past_position() const;
    Eigen::Vector3d velocity() const;

    void dodge(const VelocityParticle& vel_particle);
    void dodge(const VelocityLineSegment& vel_line_segment);
    void vfi_dodge(const VelocityParticle& vel_particle, const double& vfi_threshold, const double& vfi_gain);
    void vfi_dodge(const VelocityLineSegment& vel_line_segment, const double& vfi_threshold, const double& vfi_gain);

    Eigen::Vector3d operator+ (const VelocityParticle& vel_particle) const;

    void operator+= (const VelocityParticle& vel_particle);

    Eigen::Vector3d operator- (const VelocityParticle& vel_particle) const;
    Eigen::Vector3d operator- (const LineSegment& line_segment) const;

    void operator-= (const VelocityParticle& vel_particle);
   
};
};