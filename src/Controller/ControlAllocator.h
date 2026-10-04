#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// ControlAllocator
//
// Maps the controller's 4-element output vector onto the four physical
// actuator channels.
//
// The LQR outputs TVC DEFLECTION ANGLES directly, in degrees. It does not
// output body torques: the thrust-vector geometry that relates a deflection
// to a moment is already baked into the gain K, which was solved against a
// plant whose input is gimbal angle. So there is no torque inversion to do
// here, and no dependence on live thrust or on the COM-to-TVC moment arm.
//
// An earlier draft of this class carried a second TORQUE mode that computed
//
//     delta = asin( tau / (Fz * L) )
//
// for a future controller re-derived against a torque-input plant. That mode
// is gone. It was dead code guarding against a redesign that is not happening,
// and keeping it meant every reader had to work out which of two unit
// conventions the input vector was in before they could follow the data flow.
// If a torque-output controller is ever written, the inversion belongs with
// it, not behind a runtime flag here.
//
// WHAT IS LEFT, AND WHY THE CLASS STILL EXISTS
// ────────────────────────────────────────────
// Routing. The controller thinks in roll and pitch CHANNELS; the airframe has
// two gimbal servos on specific pins. Deciding that the roll channel drives
// the X servo and the pitch channel drives the Y servo is a real allocation
// decision, and this is the one place it is written down. Get it backwards and
// the vehicle tumbles -- it is worth a named home rather than being implicit
// in the argument order of a setGimbal() call.
//
// Input, from AttitudeController::update():
//     u = [ gimbalX_deg, gimbalY_deg, yawTorque_Nm, thrust_N ]
//
// Yaw stays a torque. RCSActuator owns torque -> per-motor force -> PWM, so
// the allocator passes it through untouched.
// ─────────────────────────────────────────────────────────────────────────────

// Result of allocation, in the units each actuator channel expects.
struct AllocatorOutput {
    float gimbalX_deg;    // GIMBAL_X actuator input
    float gimbalY_deg;    // GIMBAL_Y actuator input
    float yawTorque_Nm;   // RCS actuator input (vehicle yaw torque)
    float thrust_N;       // EDF actuator input
};

class ControlAllocator {
public:
    ControlAllocator() = default;

    // u : 4-element controller output. Passed as a plain array so this class
    //     has NO dependency on the linear-algebra library or the controller
    //     type -- keeps it unit-testable in isolation.
    AllocatorOutput allocate(const float u[4]) const;
};
