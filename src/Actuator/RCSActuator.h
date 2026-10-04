#pragma once
#include "Actuator.h"

// ─────────────────────────────────────────────────────────────────────────────
// RCSActuator
//
// Two counter-clockwise BLDC motors that counteract the EDF's angular
// momentum. Both motors are wired to a SINGLE PWM line (Y-harness), so they
// always receive an identical command and produce an identical force.
//
// Why the torque is divided by two
// ────────────────────────────────
// Both motors sit at the same distance r from the vehicle COM, and each
// produces the same force F. Their torques add:
//
//     T = F*r + F*r  =  2*F*r
//
// Solving for the force ONE motor must produce:
//
//     F = T / (2*r)
//
// The regression (_p0.._p2) is calibrated for a SINGLE motor -- force in
// Newtons -> PWM microseconds. So F, not the total force and not the torque,
// is what gets fed through it. The resulting PWM goes out one wire and both
// motors follow it.
//
// Units:
//   drive()           takes vehicle torque in N*m
//   _min / _max       are PER-MOTOR force limits in Newtons
//   _neutral          is 0 N*m -- neutral is always in drive()'s INPUT
//                     units, which for this class differ from min/max
//   getCurrentValue() returns PER-MOTOR force in Newtons (not torque)
// ─────────────────────────────────────────────────────────────────────────────

class RCSActuator : public Actuator {
public:
    // pin        : shared PWM pin driving both ESCs
    // p0, p1, p2 : regression, single-motor force (N) -> PWM (us)
    // minForce   : per-motor force lower limit (N)
    // maxForce   : per-motor force upper limit (N)
    // momentArm  : COM-to-RCS distance in metres
    // cutoffNm / offUs : a torque at or below cutoffNm writes offUs raw, so
    //              neutral() holds the ESCs below their arming threshold.
    RCSActuator(uint8_t pin,
                float p0, float p1, float p2,
                float minForce, float maxForce,
                float momentArm,    // neutral is fixed at 0 N*m
                float cutoffNm = -FLT_MAX,
                uint16_t offUs = 0);

    // value is VEHICLE torque in N*m
    void drive(float torque_Nm) override;

    float getMomentArm() const { return _momentArm; }

    // Torque currently being produced, reconstructed from per-motor force.
    float getCurrentTorque() const { return _currentValue * 2.0f * _momentArm; }

private:
    float _momentArm;
};
