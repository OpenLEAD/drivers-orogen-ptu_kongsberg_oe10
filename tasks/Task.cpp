/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <ptu_kongsberg_oe10/Driver.hpp>

using namespace ptu_kongsberg_oe10;

Task::Task(std::string const& name)
    : TaskBase(name),m_driver(NULL)
{
    init();
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),m_driver(NULL)
{
    init();
}

Task::~Task()
{
}

::ptu_kongsberg_oe10::Status Task::getStatus()
{
    if(!m_driver)
        throw std::runtime_error("setEndStop: device not configured");
    return m_driver->getStatus(_device_id.get());
}

void Task::setEndStop(::ptu_kongsberg_oe10::END_STOPS const & mode)
{
    if(!m_driver)
        throw std::runtime_error("setEndStop: device not configured");

    switch(mode)
    {
    case PAN_POSITIVE:
        m_driver->setPanPositiveEndStop(_device_id.get());
 	break;
    case PAN_NEGATIVE:
        m_driver->setPanNegativeEndStop(_device_id.get());
 	break;
    case TILT_POSITIVE:
        m_driver->setTiltPositiveEndStop(_device_id.get());
 	break;
    case TILT_NEGATIVE:
        m_driver->setTiltNegativeEndStop(_device_id.get());
 	break;
    }
}

void Task::panStop()
{
    if(!m_driver)
        throw std::runtime_error("panStop: device not configured");
    m_driver->panStop(_device_id.get());
}

void Task::tiltStop()
{
    if(!m_driver)
        throw std::runtime_error("tiltStop: device not configured");
    m_driver->tiltStop(_device_id.get());
}

void Task::useEndStops(bool enable)
{
    if(!m_driver)
        throw std::runtime_error("useEndStops: device not configured");
    m_driver->useEndStops(_device_id.get(), enable);
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
    m_limits[0].max.position = 360 * M_PI / 180;
    m_limits[0].max.speed = 0;
    m_limits.names[1] = "tilt";
    m_limits[1].min.position = 0;
    m_limits[1].max.position = 360 * M_PI / 180;
    m_limits[1].max.speed = 0;
    _limits.set(m_limits);

    // Start frame is the pan plate. X is within the plate's plane, Z going out
    // of the plate towards the body. The center of the body lies at the
    // intersection of both tilt and pan rotation axis.

    // Transformation from the center of the pan plate to the center of the body.
    Eigen::Vector3d pan_plate2center(0, 0, -0.1148);
    // Transformation from the center of the body to the center of the tilt plate
    Eigen::Vector3d center2tilt_plate(0, 0.0858, 0);
    // Rotation to align the tilt frame properly. X is along the zero
    // measurement, Z going out of the plate directed outside the PTU body
    Eigen::AngleAxisd q_center2tilt_plate =
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX());

    Eigen::Isometry3d pan_plate2tilt_plate = Eigen::Isometry3d::Identity();
    pan_plate2tilt_plate.translate(pan_plate2center);
    pan_plate2tilt_plate.translate(center2tilt_plate);
    pan_plate2tilt_plate.rotate(q_center2tilt_plate);
    this->pan_plate2tilt_plate = pan_plate2tilt_plate;
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
    m_driver->useEndStops(_device_id.get(), _use_end_stops.get());
    m_driver->setTiltSpeed(_device_id.get(),_tilt_speed.get());
    m_driver->setPanSpeed(_device_id.get(),_pan_speed.get());
    return true;
}

void Task::writeJoints(base::Time const& time, float pan, float tilt)
{
    m_sample[0] = base::JointState::Position(pan);
    m_sample[1] = base::JointState::Position(tilt);
    m_sample.time = base::Time::now();
    _joints_samples.write(m_sample);
    
    base::samples::RigidBodyState ptu_sample;
    Eigen::Isometry3d transform(pan_plate2tilt_plate);
    transform.prerotate( Eigen::AngleAxisd(pan, Eigen::Vector3d::UnitZ()) );
    transform.rotate( Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitZ()) );
    ptu_sample.sourceFrame = _tilt_plate_frame.get();
    ptu_sample.targetFrame = _pan_plate_frame.get();
    ptu_sample.time = time;
    ptu_sample.orientation = transform.rotation();
    ptu_sample.position = transform.translation();
    _transformation_samples.write(ptu_sample);
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    m_junk_angle_filter = _junk_angle_filter.get();

    // clear old messages
    // if nothing is in the buffer after the stopHook was called
    // rtt is complaining about a timeout
    if(m_driver->isPanTiltStatusRequested(_device_id.get()))
        m_driver->readPanTiltStatus(_device_id.get());

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

bool Task::filterJunkAngles(PanTiltStatus const& status) const
{
    double time_diff = (status.time - m_sample.time).toSeconds();
    double max_pan  = status.pan_speed  * time_diff * 30 * M_PI / 180;
    double max_tilt = status.tilt_speed * time_diff * 30 * M_PI / 180;
    double pan_diff = fabs(status.pan - m_sample[0].position);
        double tilt_diff = fabs(status.tilt - m_sample[1].position);
    return (pan_diff <= max_pan) && (tilt_diff < max_tilt);
}

void Task::processIO()
{
    PanTiltStatus status = m_driver->readPanTiltStatus(_device_id.get());
    if (m_junk_angle_filter)
    {
        if (filterJunkAngles(status))
            writeJoints(status.time, status.pan, status.tilt);
    }

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
    // stop the device
    m_driver->tiltStop(_device_id.get());
    m_driver->panStop(_device_id.get());

    // do not clear the requestPanTiltStatus here
    // because this leads to a rtt timeout

    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete m_driver;
    m_driver = NULL;
}

