#pragma once
#include <BasicLinearAlgebra.h>
#include "consts.h"

// ─────────────────────────────────────────────────────────────────────────────
// AttitudeController
//
// Inner-loop LQR regulating vehicle attitude and altitude. Pure state->u map:
// it takes measurements, subtracts the held setpoint, multiplies by the gain,
// adds hover-thrust feedforward, filters and clamps, and returns the control
// vector. It does NOT allocate to actuators and does NOT know the position
// loop exists -- feed setReference() directly for a standalone inner-loop
// test, or let PositionController drive the roll/pitch setpoints in the full
// cascade.
//
// State  (8): [ roll, pitch, yaw, gyroX, gyroY, gyroZ, z, velZ ]
// Output (4): [ gimbalX_deg, gimbalY_deg, yawTorque_Nm, thrust_N ]
//
// THE TVC CHANNELS ARE DEFLECTION ANGLES, NOT TORQUES
// ───────────────────────────────────────────────────
// K was solved against a plant whose input is gimbal angle, so -K*x already
// IS a deflection. The thrust-vector geometry relating that deflection to a
// body moment is inside the gain; nothing downstream has to invert it. This
// is why ControlAllocator is pure routing and never sees the moment arm.
//
// The gain is expressed in radians, because the state it multiplies is in
// radians. The conversion to degrees happens HERE rather than in the
// allocator, so that every consumer of this class -- allocator, actuator,
// telemetry, logs -- sees one unit and the output name can carry a _deg
// suffix honestly. Scaling before the filter and clamp is exactly equivalent
// to scaling after, since both operations are linear.
//
// Sign convention: yaw and gyroZ are negated when loaded into the state
// vector, matching the MATLAB model the gain K was derived against. This is a
// frame convention, not a workaround -- do not remove without re-deriving K.
// See also the mounting-handedness note in Imu.h, which interacts with this.
//
// Gravity feedforward: hover thrust (WEIGHT_N) is added to the thrust channel
// so the LQR only regulates deviation from hover, not the whole weight.
//
// Yaw torque clamps to [0, MAX_YAW_TORQUE_NM], not a symmetric range. Both
// RCS motors spin the same way and can only push the airframe one direction.
//
// IIR filter: the two gimbal channels are low-pass filtered across calls to
// smooth servo motion. State is held internally, so the controller is
// stateful -- construct one, reuse it every loop.
// ─────────────────────────────────────────────────────────────────────────────

class AttitudeController {
public:
    AttitudeController();

    // Set all four setpoints at once.
    //   rollRef, pitchRef, yawRef : radians
    //   zRef                      : metres
    // In the cascade, PositionController supplies rollRef/pitchRef each loop.
    void setReference(float rollRef_rad, float pitchRef_rad,
                      float yawRef_rad, float zRef_m);

    // Set one setpoint without disturbing the others. In the cascade the
    // position loop only updates roll/pitch each step, so these let it write
    // just those two and leave yaw/z as last set.
    void setRollRef(float rollRef_rad);
    void setPitchRef(float pitchRef_rad);
    void setYawRef(float yawRef_rad);
    void setZRef(float zRef_m);

    // One control step. Arguments are the measured state:
    //   roll_rad, pitch_rad, yaw_rad : radians
    //   gyroX/Y/Z_rps                : rad/s (body rates)
    //   z_m                          : metres (altitude)
    //   velZ_mps                     : m/s (vertical velocity)
    // Returns [ gimbalX_deg, gimbalY_deg, yawTorque_Nm, thrust_N ].
    BLA::Matrix<4, 1> update(float roll_rad, float pitch_rad, float yaw_rad,
                             float gyroX_rps, float gyroY_rps, float gyroZ_rps,
                             float z_m, float velZ_mps);

    // Reset the IIR filter memory (e.g. on re-arm) so stale servo commands
    // don't bleed into the next flight.
    void resetFilter();

    // Last output, for telemetry without re-running the control law.
    const BLA::Matrix<4, 1>& getLastOutput() const { return _u; }

private:
    // out = (1-alpha)*in + alpha*prev   -- first-order IIR low-pass.
    static float iir(float in, float prev, float alpha);

    BLA::Matrix<cfg::ctrl::NU, cfg::ctrl::NX> _K;     // gain, 4x8
    BLA::Matrix<cfg::ctrl::NX, 1>             _ref;   // setpoint, 8x1
    BLA::Matrix<4, 1>                         _u;     // last output

    // IIR memory, in DEGREES -- the same unit the channels are published in.
    float _prevGimbalX_deg;
    float _prevGimbalY_deg;
};
