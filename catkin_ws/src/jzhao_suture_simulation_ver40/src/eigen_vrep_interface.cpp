#include "eigen_vrep_interface.h"

namespace kai
{

Eigen_VrepInterface::Eigen_VrepInterface(std::atomic_bool* no_blocking_loops=nullptr)
{
    no_blocking_loops_ = no_blocking_loops;
    global_retry_count_ = 0;
    clientID_ = -1;
}

Eigen_VrepInterface::~Eigen_VrepInterface()
{
    disconnect();
}

bool Eigen_VrepInterface::connect(const int &port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT)
{
    TIMEOUT_IN_MILISECONDS_ = TIMEOUT_IN_MILISECONDS;
    MAX_TRY_COUNT_          = MAX_TRY_COUNT;

    //The timeout for simxStart makes more sense as a negative number
    //http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm#simxStart
    simxStart()
    clientID_ = simxStart("127.0.0.1", port, true, true, -TIMEOUT_IN_MILISECONDS_, );
    if(clientID_!=-1)
    {
        //We'll start at least one streaming service at connection time so that we can grab the simulation
        //state correctly. http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm#simxGetInMessageInfo
        int dummyValue;
        simxGetIntegerParameter(clientID_, sim_intparam_program_version, &dummyValue, simx_opmode_streaming);
        return true;
    }
    else
        return false;
}

bool Eigen_VrepInterface::connect(const std::string& ip, const int& port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT)
{
    TIMEOUT_IN_MILISECONDS_ = TIMEOUT_IN_MILISECONDS;
    MAX_TRY_COUNT_          = MAX_TRY_COUNT;

    //The timeout for simxStart makes more sense as a negative number
    //http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm#simxStart
    clientID_ = simxStart(ip.c_str(),port,1,1,-TIMEOUT_IN_MILISECONDS_,1);
    if(clientID_!=-1)
    {
        //We'll start at least one streaming service at connection time so that we can grab the simulation
        //state correctly. http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm#simxGetInMessageInfo
        int dummyValue;
        simxGetIntegerParameter(clientID_,sim_intparam_program_version,&dummyValue,simx_opmode_streaming);
        return true;
    }
    else
        return false;
}

void Eigen_VrepInterface::disconnect()
{
    if(clientID_>-1)
        simxFinish(clientID_);
}

void Eigen_VrepInterface::disconnect_all()
{
    simxFinish(-1);
}

void Eigen_VrepInterface::start_simulation() const
{
    simxStartSimulation(clientID_, simx_opmode_blocking);
}

void Eigen_VrepInterface::stop_simulation()  const
{
    simxStopSimulation(clientID_, simx_opmode_blocking);
}

bool Eigen_VrepInterface::is_simulation_running() const
{
    simxInt simulation_state;
    simxGetInMessageInfo(clientID_, simx_headeroffset_server_state, &simulation_state);
    return simulation_state & 0x1;
}

int Eigen_VrepInterface::get_object_handle(const std::string& objectname)
{
    int hp;
    simxGetObjectHandle(clientID_, objectname.c_str(), &hp, simx_opmode_blocking);
    return hp;
}

std::vector<int> Eigen_VrepInterface::get_object_handles(const std::vector<std::string>& objectnames)
{
    const int& n = objectnames.size();
    std::vector<int> handles(n);
    for(int i=0; i<n; ++i)
    {
        handles[i] = get_object_handle(objectnames[i]);
    }
    return handles;
}


Eigen::Vector3d Eigen_VrepInterface::get_object_translation(const int& handle)
{
    simxFloat tp[3];
    simxGetObjectPosition(clientID_, handle, sim_handle_world, tp, simx_opmode_blocking);
    return Eigen::Vector3d(double(tp[0]), double(tp[1]), double(tp[2]));
}

void Eigen_VrepInterface::set_object_translation(const int& handle, const Eigen::Vector3d& t) const
{
    simxFloat tp[3];
    tp[0]=float(t[0]);
    tp[1]=float(t[1]);
    tp[2]=float(t[2]);

    simxSetObjectPosition(clientID_, handle, sim_handle_world, tp, simx_opmode_oneshot);
}

}
