#include "suture.h"

namespace kai
{

json11::Json __parse_json(const std::string& file)
{
    std::string error;
    std::ifstream infile(file);
    std::string str_in_file;
    if (infile)
    {
        std::ostringstream outstring;
        outstring << infile.rdbuf();
        str_in_file = outstring.str();
    }
    json11::Json parsed_json = json11::Json::parse(str_in_file, error);
    if (error != "")
        throw std::runtime_error(ros::this_node::getName() + "::Json parse error: " + error);
    return parsed_json;
}

void __initialize_configuration(SutureConfiguration& cfg_, const json11::Json& parsed_json)
{
    cfg_.element_number = parsed_json["element_number"].int_value();
    cfg_.activity_level = parsed_json["activity_level"].number_value();
    cfg_.elastic_modulus = parsed_json["elastic_modulus"].number_value();
    cfg_.element_mass = parsed_json["element_mass"].number_value();
    cfg_.forward_priority = parsed_json["forward_priority"].number_value();
    cfg_.element_spacing = parsed_json["element_spacing"].number_value();
}

Suture::Suture(const std::string& suture_parameters_path)
{
    json11::Json parsed_json = __parse_json(suture_parameters_path);
    __initialize_configuration(cfg_, parsed_json);

    for (int i=0; i<cfg_.element_number; ++i)
    {
        body_.push_back(Particle(cfg_.element_mass));
    }    
}

void Suture::initialize(const std::vector<Eigen::Vector3d>& positions)
{
    for (int i=0; i<body_.size(); ++i)
    {
        body_[i].initialize(positions[i]);
    }
}

void Suture::initialize(const std::vector<Eigen::Vector3f>& positions)
{
    for (int i=0; i<body_.size(); ++i)
    {
        body_[i].initialize(positions[i]);
    }
}

int Suture::size() const
{
    return body_.size();
}

void Suture::continuity(int count)
{
    const int& size = body_.size();
    auto __jakobsen_method = [&](const int& index, const int& toward_index, const double& ratio)
    {
        // let element of {index} move towards the element of {toward_index} with distance of {ratio * (current distance - element spacing)}
        const Eigen::Vector3d& diff = body_[toward_index] - body_[index];
        const double& diff_norm = vec3(diff).norm();
        body_[index] += ratio * (1 - cfg_.element_spacing  / diff_norm) * diff;
    };
    while(count--)
    {

        if (!body_[0].is_attached())
        {
            if (!body_[1].is_attached())
                __jakobsen_method(0, 1, 1 - cfg_.forward_priority);
            else
                __jakobsen_method(0, 1, 1);
        }
        for (int i=1; i<size-1; ++i)
        {
            if (!body_[i].is_attached())
            {
                if (!body_[i-1].is_attached() && !body_[i+1].is_attached())
                {
                    __jakobsen_method(i, i - 1, cfg_.forward_priority);
                    __jakobsen_method(i, i + 1, 1 - cfg_.forward_priority);
                }
                else if (body_[i-1].is_attached() && !body_[i+1].is_attached())
                {
                    __jakobsen_method(i, i - 1, 1);
                    __jakobsen_method(i, i + 1, 1 - cfg_.forward_priority);
                }
                else if (!body_[i-1].is_attached() && body_[i+1].is_attached())
                {
                    __jakobsen_method(i, i - 1, cfg_.forward_priority);
                    __jakobsen_method(i, i + 1, 1);
                }
            }
        }
        if (!body_[size-1].is_attached())
        {
            if (!body_[size-2].is_attached())
                __jakobsen_method(size-1, size-2, cfg_.forward_priority);
            else
                __jakobsen_method(size-1, size-2, 1);
        }
    }
}

Suture Suture::operator= (const Suture& suture)
{
    cfg_ = suture.cfg_;
    body_ = suture.body_;
}

Particle Suture::operator[] (const int& i) const
{
    return body_[i];
}
}