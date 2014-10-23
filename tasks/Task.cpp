/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <ptu_kongsberg_oe10/Driver.hpp>

using namespace ptu_kongsberg_oe10;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    init();
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    init();
}

Task::~Task()
{
}

void Task::init()
{
    m_sample.resize(2);
    m_sample.names[0] = "pan";
    m_sample.names[1] = "tilt";
    m_cmd.resize(2);

    m_limits.resize(2);
    m_limits.names[0] = "pan";
    m_limits[0].min.position = 0;
    m_limits[0].max.position = 270 * M_PI / 180;
    m_limits[0].max.speed = 0;
    m_limits.names[1] = "tilt";
    m_limits[1].min.position = 0;
    m_limits[1].max.position = 270 * M_PI / 180;
    m_limits[1].max.speed = 0;
    _limits.set(m_limits);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    std::auto_ptr<Driver> driver(new Driver);
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());

    m_driver = driver.release();
    setDriver(m_driver);

    if (! TaskBase::configureHook())
        return false;

    // This is not strictly needed, but makes sure the device is indeed there available
    Status status = m_driver->getStatus(_device_id.get());
    writeJoints(base::Time::now(), status.pan, status.tilt);
    return true;
}

void Task::writeJoints(base::Time const& time, float pan, float tilt)
{
    m_sample[0] = base::JointState::Position(pan);
    m_sample[1] = base::JointState::Position(tilt);
    m_sample.time = base::Time::now();
    _joints_samples.write(m_sample);
    
    base::samples::RigidBodyState ptu_sample;
    ptu_sample.orientation =
       Eigen::AngleAxisd(pan, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY());
    ptu_sample.sourceFrame = _moving_frame.get();
    ptu_sample.targetFrame = _base_frame.get();
    ptu_sample.time = time;
    _orientation_samples.write(ptu_sample);
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    // Initialize the current command to the current value
    PanTiltStatus status = m_driver->getPanTiltStatus(_device_id.get());
    writeJoints(base::Time::now(), status.pan, status.tilt);

    // Start pumping
    m_driver->requestPanTiltStatus(_device_id.get());
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::processIO()
{
    PanTiltStatus status = m_driver->readPanTiltStatus(_device_id.get());
    writeJoints(base::Time::now(), status.pan, status.tilt);

    if (_joints_cmd.readNewest(m_cmd) == RTT::NewData)
    {
        if (m_cmd[0].hasPosition())
            m_driver->setPanPosition(_device_id.get(), m_cmd[0].position);
        if (m_cmd[1].hasPosition())
            m_driver->setTiltPosition(_device_id.get(), m_cmd[1].position);
    }
    m_driver->requestPanTiltStatus(_device_id.get());
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete m_driver;
}

