/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <ptu_kongsberg_oe10/Driver.hpp>

using namespace ptu_kongsberg_oe10;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    std::auto_ptr<Driver> driver(new Driver);
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());

    // This is not strictly needed, but makes sure the device is indeed there available
    Status status = driver->getStatus(_device_id.get());
    Angles initial;
    initial.pan = status.pan;
    initial.tilt = status.tilt;
    _angles.write(initial);

    setDriver(driver.release());
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::processIO()
{
    throw std::runtime_error("NOT IMPLEMENTED YET");
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
}
