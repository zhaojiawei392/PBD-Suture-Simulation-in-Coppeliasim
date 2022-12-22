#pragma once
#include <ros/ros.h>
#include <vector>
#include <json11.hpp>
#include "line.h"
#include "point.h"

namespace kai
{

struct SutureConfiguration
{
    double element_number;
    double element_mass;
    double forward_priority;
    double element_spacing;
    double elastic_modulus;
    double activity_level;
}

class Suture
{
private:
    std::vector<Particle> body_;
    SutureConfiguration cfg_;
public:
    Suture(const std::string& suture_parameters_path);
    void initialize(const std::vector<Eigen::Vector3d>& positions);
    void initialize(const std::vector<Eigen::Vector3f>& positions);

    int size() const;

    void continuity(int count);
    void constraints();

    Suture operator= (const Suture& suture);
    Particle operator[] (const int& i) const;

}
}