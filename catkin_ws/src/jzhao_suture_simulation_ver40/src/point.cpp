#include "point.h"

namespace pbd
{
/**************************************************************************************************************************************************
 * @brief Construct a new Point:: Point object
 * 
 */
Point::Point():
    attachment_(released)
{}

void Point::initialize(const Eigen::Vector3d& position)
{
    current_position_ = position;
}

void Point::set_position(const Eigen::Vector3d& position)
{
    current_position_ = position;
}

Eigen::Vector3d Point::position() const
{
    return current_position_;
}

int Point::attachment() const
{
    return attachment_;
}

std::string Point::is_attached() const
{
    return attachment_ == attached;
}

void Point::attach_to(const Eigen::Vector3d& position)
{
    attachment_ = attached;
    current_position_ = position;
}

void Point::release()
{
    attachment_ = released;
}

Eigen::Vector3d Point::operator+ (const Eigen::Vector3d& position)
{
    return current_position_ + position;
}

void Point::operator+= (const Eigen::Vector3d& position)
{
    current_position_ += position;
}

Eigen::Vector3d Point::operator- (const Eigen::Vector3d& position)
{
    return current_position_ - position;
}

void Point::operator-= (const Eigen::Vector3d& position)
{
    current_position_ -= position;
}



/**************************************************************************************************************************************************
 * @brief Construct a new Particle:: Particle object
 * 
 * @param mass 
 * @param radius 
 */
Particle::Particle(const double& mass, const double& radius):
    Point(),
    mass_(mass),
    radius_(radius)
{}

void Particle::set_mass(const double mass)
{
    mass_ = mass;
}

void Particle::set_mass(double&& mass)
{
    mass_ = std::move(mass);
}

void Particle::set_radius(const double& radius)
{
    radius_ = radius;
}

void Particle::set_radius(double&& radius)
{
    radius_ = std::move(radius);
}

void Particle::dodge(const Eigen::Vector3d& position, const double& position_radius)
{
    const double& distance = (current_position_ - position).norm();
    const double& safe_distance = radius_ + position_radius;
    if (distance < safe_distance)
    {
        current_position_ += (safe_distance / distance - 1) * (current_position_ - position);
    }
}

/**************************************************************************************************************************************************
 * @brief Construct a new Velocity Particle:: Velocity Particle object
 * 
 * @param mass 
 * @param radius 
 */
VelocityParticle::VelocityParticle(const double& mass, const double& radius):
    Particle(mass, radius)
{}

void VelocityParticle::initialize(const Eigen::Vector3d& position)
{
    past_position_ = current_position_ = position;
    velocity_ = Eigen::Vector3d(0,0,0);
}

void VelocityParticle::set_position(const VelocityParticle& vel_particle)
{
    current_position_ = vel_particle.current_position_;
}

void VelocityParticle::update_position_and_velocity(const Eigen::Vector3d& position)
{
    past_position_ = current_position_;
    current_position_ = position;
    velocity_ = current_position_ - past_position_;
}

void VelocityParticle::update_position_and_velocity(const VelocityParticle& vel_particle)
{
    past_position_ = current_position_;
    current_position_ = vel_particle.current_position_;
    velocity_ = current_position_ - past_position_;
}

Eigen::Vector3d VelocityParticle::past_position() const
{
    return past_position_;
}

Eigen::Vector3d VelocityParticle::velocity() const
{
    return velocity_;
}

void VelocityParticle::dodge(const VelocityParticle& vel_particle)
{
    const double& distance = (current_position_ - vel_particle.current_position_).norm();
    const double& safe_distance = radius_ + vel_particle.radius_;
    if (distance < safe_distance)
    {
        current_position_ += (current_position_ - vel_particle.current_position_) * (safe_distance / distance - 1);
    }   
}

void VelocityParticle::vfi_dodge(const VelocityParticle& vel_particle, const double& vfi_threshold, const double& vfi_gain)
{
    const double& distance = (past_position_ - vel_particle.past_position_).norm();
    const double& safe_distance = radius_ + vel_particle.radius_;
    if (distance < vfi_threshold * safe_distance)
    {
        const Eigen::Vector3d& normal = (past_position_ - vel_particle.past_position_) / distance;
        const double& speed = (velocity_ - vel_particle.velocity_).dot(normal); 
        const double& limit_speed = vfi_gain * (safe_distance - distance);
        if (speed < limit_speed)
        {
            current_position_ += normal * (limit_speed - speed);
        }
    }
}

void VelocityParticle::vfi_dodge(const VelocityLineSegment& vel_line_segment, const double& vfi_threshold, const double& vfi_gain)
{
    const double& past_past_distance = vel_line_segment.past_line_segment().distance_from_point(past_position_);
    const double& past_curr_distance = vel_line_segment.current_line_segment().distance_from_point(past_position_);
    const double& safe_distance = radius_ + vel_line_segment.radius();
    if (past_past_distance < vfi_threshold * safe_distance)
    {
        const Eigen::Vector3d& normal = (past_position_ - vel_line_segment.past_line_segment()) / past_past_distance;

    }
}




















Eigen::Vector3d VelocityParticle::operator+ (const VelocityParticle& vel_particle)
{
    return current_position_ + vel_particle.current_position_;
}

void VelocityParticle::operator+= (const VelocityParticle& vel_particle)
{
    current_position_ += vel_particle.current_position_;
}

Eigen::Vector3d VelocityParticle::operator- (const VelocityParticle& vel_particle)
{
    return current_position_ - vel_particle.current_position_;
}

Eigen::Vector3d VelocityParticle::operator- (const LineSegment& line_segment) const
{
    const Eigen::Vector3d cp = line_segment.cl
}

void VelocityParticle::operator-= (const VelocityParticle& vel_particle)
{
    current_position_ -= vel_particle.current_position_;
}

}