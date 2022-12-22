
#pragma once
#define NON_MATLAB_PARSING
#define MAX_EXT_API_CONNECTIONS=255
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <exception>
#include <thread>
#include "remoteApi/extApi.h"
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>

namespace kai
{
class Eigen_VrepInterface
{
public:
    Eigen_VrepInterface(std::atomic_bool* no_blocking_loops=nullptr);
    ~Eigen_VrepInterface();

    bool connect(const int &port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);
    bool connect(const std::string& ip, const int& port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);

    void disconnect();

    void disconnect_all();

    void start_simulation() const;

    void stop_simulation()  const;

    bool is_simulation_running() const;

    int              get_object_handle(const std::string& objectname);

    std::vector<int> get_object_handles(const std::vector<std::string>& objectnames);

    Eigen::Vector3d  get_object_translation(const int& handle);

    void set_object_translation(const int& handle, const Eigen::Vector3d& t) const;

private:
    int MAX_TRY_COUNT_;
    int TIMEOUT_IN_MILISECONDS_;
    int clientID_;
    long int global_retry_count_;
    std::atomic_bool* no_blocking_loops_;
};

}


