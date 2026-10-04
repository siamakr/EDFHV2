#include "ControlAllocator.h"

// ─────────────────────────────────────────────────────────────────────────────
// Pure routing. Every channel is already in the unit its actuator expects --
// the controller publishes degrees, and the RCS takes torque -- so there is no
// conversion left to do here.
//
// The only decision encoded below is which control channel drives which servo.
// ─────────────────────────────────────────────────────────────────────────────
AllocatorOutput ControlAllocator::allocate(const float u[4]) const
{
    AllocatorOutput out;
    out.gimbalX_deg  = u[0];   // roll channel  -> X servo
    out.gimbalY_deg  = u[1];   // pitch channel -> Y servo
    out.yawTorque_Nm = u[2];
    out.thrust_N     = u[3];
    return out;
}
