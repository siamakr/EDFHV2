#include "PositionController.h"
#include <algorithm>
#include <math.h>

using namespace cfg::ctrl::pos;

PositionController::PositionController()
{
    // Rows: roll, pitch. Columns: x, y, vx, vy, xint, yint.
    // Swapped vs V0.1 paper labels: on this airframe X error must command
    // roll (X servo) and Y error pitch (Y servo). Hover allocation is unchanged.
    _K = { K_X,  0.0f,   K_VX, 0.0f,   K_XINT,  0.0f,
           0.0f, K_Y,    0.0f, K_VY,   0.0f,    K_YINT };
}

void PositionController::setReference(float x_m, float y_m)
{
    _spX_m = x_m;
    _spY_m = y_m;
}

void PositionController::reset()
{
    _intX_m = 0.0f;
    _intY_m = 0.0f;
    _u = {};
}

AttitudeSetpoint PositionController::update(float x_m, float y_m,
                                            float vx_mps, float vy_mps)
{
    const float ex = _spX_m - x_m;
    const float ey = _spY_m - y_m;

    _intX_m += ex * cfg::loop::DT_S;
    _intY_m += ey * cfg::loop::DT_S;
    _intX_m = std::clamp(_intX_m, -INT_LIM_M, INT_LIM_M);
    _intY_m = std::clamp(_intY_m, -INT_LIM_M, INT_LIM_M);

    const BLA::Matrix<6, 1> error{ ex, ey, -vx_mps, -vy_mps, _intX_m, _intY_m };
    const BLA::Matrix<2, 1> out = _K * error;

    _u.roll_rad  = std::clamp(out(0), -MAX_ATT_RAD, MAX_ATT_RAD);
    _u.pitch_rad = std::clamp(out(1), -MAX_ATT_RAD, MAX_ATT_RAD);
    if (!isfinite(_u.roll_rad))  _u.roll_rad  = 0.0f;
    if (!isfinite(_u.pitch_rad)) _u.pitch_rad = 0.0f;
    return _u;
}
