#ifndef ptu_kongsberg_oe10_TYPES_HPP
#define ptu_kongsberg_oe10_TYPES_HPP

namespace ptu_kongsberg_oe10 {
    /** This is not the main output for the PTU, but a much nicer way to get the
     * PTU state than RigidBodyState
     */
    struct Angles
    {
        base::Time time;
        base::Angle tilt;
        base::Angle roll;
    };
}

#endif

