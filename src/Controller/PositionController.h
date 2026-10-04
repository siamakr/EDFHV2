#pragma once
#include <BasicLinearAlgebra.h>
#include "consts.h"

// Outer-loop LQR from V0.1. Turns horizontal position/velocity into
// roll and pitch setpoints for AttitudeController.
//
// State  (6): [ x, y, vx, vy, xint, yint ]
// Output (2): [ rollRef_rad, pitchRef_rad ]
//
// X error -> roll (X servo), Y error -> pitch (Y servo). V0.1's K rows
// were the other way around; that moved the wrong gimbal on a Y waypoint.

struct AttitudeSetpoint {
    float roll_rad  = 0.0f;
    float pitch_rad = 0.0f;
};

class PositionController {
public:
    PositionController();

    void setReference(float x_m, float y_m);
    void reset();

    AttitudeSetpoint update(float x_m, float y_m, float vx_mps, float vy_mps);

    const AttitudeSetpoint& getLastOutput() const { return _u; }

private:
    BLA::Matrix<2, 6> _K;
    float _spX_m = 0.0f;
    float _spY_m = 0.0f;
    float _intX_m = 0.0f;
    float _intY_m = 0.0f;
    AttitudeSetpoint _u;
};
