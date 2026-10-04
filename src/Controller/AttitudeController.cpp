#include "AttitudeController.h"
#include <algorithm>
#include <math.h>

using namespace cfg::ctrl;
using cfg::vehicle::WEIGHT_N;
using cfg::unit::R2D;

// ─────────────────────────────────────────────────────────────────────────────
AttitudeController::AttitudeController()
    : _prevGimbalX_deg(0.0f)
    , _prevGimbalY_deg(0.0f)
{
    // Gain matrix, row per output, column per state.
    // Rows:    [ gimbalX, gimbalY, yawTorque, thrust ]
    // Columns: [ roll, pitch, yaw, gyroX, gyroY, gyroZ, z, velZ ]
    // Zeros are the couplings the LQR solve left negligible.
    _K = { K_ROLL,   0.0f,    0.0f,   K_GX,   0.0f,   0.0f,   0.0f,   0.0f,
           0.0f,     K_PITCH, 0.0f,   0.0f,   K_GY,   0.0f,   0.0f,   0.0f,
           0.0f,     0.0f,    K_YAW,  0.0f,   0.0f,   K_GZ,   0.0f,   0.0f,
           0.0f,     0.0f,    0.0f,   0.0f,   0.0f,   0.0f,   K_Z,    K_VZ };

    _ref.Fill(0.0f);
    _u.Fill(0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
void AttitudeController::setReference(float rollRef_rad, float pitchRef_rad,
                                      float yawRef_rad, float zRef_m)
{
    _ref(0) = rollRef_rad;
    _ref(1) = pitchRef_rad;
    _ref(2) = yawRef_rad;
    _ref(3) = 0.0f;    // gyroX target
    _ref(4) = 0.0f;    // gyroY target
    _ref(5) = 0.0f;    // gyroZ target
    _ref(6) = zRef_m;
    _ref(7) = 0.0f;    // velZ target
}

void AttitudeController::setRollRef(float rollRef_rad)   { _ref(0) = rollRef_rad; }
void AttitudeController::setPitchRef(float pitchRef_rad) { _ref(1) = pitchRef_rad; }
void AttitudeController::setYawRef(float yawRef_rad)     { _ref(2) = yawRef_rad; }
void AttitudeController::setZRef(float zRef_m)           { _ref(6) = zRef_m; }

// ─────────────────────────────────────────────────────────────────────────────
BLA::Matrix<4, 1> AttitudeController::update(
        float roll_rad, float pitch_rad, float yaw_rad,
        float gyroX_rps, float gyroY_rps, float gyroZ_rps,
        float z_m, float velZ_mps)
{
    // Load state. Yaw and gyroZ are negated to match the frame convention K
    // was derived against.
    BLA::Matrix<NX, 1> x = { roll_rad, pitch_rad, -yaw_rad,
                             gyroX_rps, gyroY_rps, -gyroZ_rps,
                             z_m, velZ_mps };

    BLA::Matrix<NX, 1> error = x - _ref;

    // LQR control law. Rows 0 and 1 come out as TVC deflection angles in
    // radians, because the state driving them is in radians -- not torques.
    BLA::Matrix<4, 1> u = -_K * error;

    // Publish the deflections in degrees, the unit every consumer wants.
    // Done before the filter and clamp, which is equivalent to doing it after
    // since both are linear -- but it means the IIR memory and the limit are
    // in the same unit as the output, so nothing downstream has to convert.
    u(0) *= R2D;
    u(1) *= R2D;
    if (!isfinite(u(0))) u(0) = 0.0f;
    if (!isfinite(u(1))) u(1) = 0.0f;
    if (!isfinite(_prevGimbalX_deg)) _prevGimbalX_deg = 0.0f;
    if (!isfinite(_prevGimbalY_deg)) _prevGimbalY_deg = 0.0f;

    // Hover-thrust feedforward: regulate deviation from hover, not full weight.
    u(3) += WEIGHT_N;

    // Low-pass the two gimbal channels; keep memory for next call.
    u(0) = iir(u(0), _prevGimbalX_deg, SERVO_ALPHA);
    u(1) = iir(u(1), _prevGimbalY_deg, SERVO_ALPHA);
    _prevGimbalX_deg = u(0);
    _prevGimbalY_deg = u(1);

    // Clamp to physical limits. MAX_TVC_DEG is an alias of the gimbal's own
    // mechanical travel, so the controller and the actuator cannot disagree.
    u(0) = std::clamp(u(0), -MAX_TVC_DEG, MAX_TVC_DEG);
    u(1) = std::clamp(u(1), -MAX_TVC_DEG, MAX_TVC_DEG);

    // Yaw is one-directional: both RCS motors spin the same way.
    u(2) = std::clamp(u(2), 0.0f, MAX_YAW_TORQUE_NM);
    u(3) = std::clamp(u(3), MIN_THRUST_N, MAX_THRUST_N);

    _u = u;
    return u;
}

// ─────────────────────────────────────────────────────────────────────────────
void AttitudeController::resetFilter()
{
    _prevGimbalX_deg = 0.0f;
    _prevGimbalY_deg = 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
float AttitudeController::iir(float in, float prev, float alpha)
{
    return (1.0f - alpha) * in + alpha * prev;
}
