#pragma once
#include "SensorManager.h"
#include <BasicLinearAlgebra.h>

// ─────────────────────────────────────────────────────────────────────────────
// Estimator
//
// Fuses the three sensors into a 6-state world-frame estimate:
//
//     [ x, y, z, vx, vy, vz ]
//
// Steady-state Kalman form: the gain Kf is fixed (computed offline), so there
// is no covariance propagation on the Teensy. A and B are built ONCE from
// cfg::loop::DT_S and held const -- the control loop is scheduled on a fixed
// 5 ms tick, so rebuilding them per cycle would be wasted work.
//
// Split out of V0.1's Sensors class, which owned both the hardware and this
// maths. As a separate class it can be stepped with synthetic measurements on
// a host, with no BNO080, LIDAR or SPI in the picture.
//
// WHAT IT DOES WITH EACH MEASUREMENT
// ──────────────────────────────────
//   accelerometer  body -> world, then used as the control input u (not a
//                  measurement) -- it drives the prediction step through B
//   lidar          tilt-compensated to a true vertical height, then a
//                  position measurement on z
//   optical flow   gyro-compensated, scaled by height, rotated to world, then
//                  a velocity measurement on vx/vy
//
// Only the rows of H whose sensors reported fresh data this cycle are enabled,
// so a dropped LIDAR reading leaves the altitude to coast on the prediction
// rather than being corrected toward a stale value.
//
// ORDERING -- read this before wiring main
// ────────────────────────────────────────
// update() must run BEFORE the controller, every cycle, as it did in V0.1.
// Run after, the controller consumes a z and vz one full cycle (5 ms) old.
//
// TELEMETRY
// ─────────
// No printing. getState() is the estimate; getDebug() carries the intermediate
// quantities V0.1 kept in debug_t -- world-frame accelerations, the tilt
// compensated height, the pre-correction prediction, and the compensated flow
// velocities. They are real members with getters rather than locals, precisely
// so a logger can read them without this class changing.
// ─────────────────────────────────────────────────────────────────────────────

struct StateEstimate {
    float x_m   = 0.0f, y_m    = 0.0f, z_m    = 0.0f;
    float vx_mps = 0.0f, vy_mps = 0.0f, vz_mps = 0.0f;
};

struct EstimatorDebug {
    // Acceleration after rotation into the world frame (the u vector)
    float axw_mps2 = 0.0f, ayw_mps2 = 0.0f, azw_mps2 = 0.0f;

    // Lidar height after tilt compensation, and its raw derivative. vzLidar
    // is diagnostic only -- vz is corrected from the z residual, not from this.
    float zWorld_m    = 0.0f;
    float vzLidar_mps = 0.0f;

    // Flow after gyro compensation, range scaling and rotation: the actual
    // vx/vy fed to the correction step
    float vxMeas_mps = 0.0f, vyMeas_mps = 0.0f;
    float flowRange_m = 0.0f;

    // Prediction before the correction step
    float xpreX_m = 0.0f, xpreY_m = 0.0f, xpreZ_m = 0.0f;
    float xpreVx_mps = 0.0f, xpreVy_mps = 0.0f, xpreVz_mps = 0.0f;

    // Rows of H that were active this cycle
    bool usedLidar = false;
    bool usedFlow  = false;
    bool usedPos   = false;

    // Diagnostics
    uint32_t nanResets   = 0;   // times the state went non-finite and was zeroed
    uint32_t overruns    = 0;   // cycles rejected because the loop fell behind
    uint32_t updateCount = 0;
};

class Estimator {
public:
    Estimator();

    // Zeroes the state. Call before arming so a bench session's accumulated
    // drift does not carry into the flight. The first valid lidar reading
    // after this seeds z directly rather than being converged toward, so
    // there is no startup transient in vz.
    void reset();

    // True once an altitude has been seeded since the last reset().
    bool isSeeded() const { return _seeded; }

    // One correction/prediction cycle. Runs before the controller.
    void update(const SensorFrame& frame);

    // Tells the estimator the last interval was far enough off nominal that
    // integrating it would corrupt the state. The next update() holds the
    // state instead of stepping it. main sets this when the measured interval
    // exceeds cfg::loop::DT_MAX_US.
    void flagOverrun() { _overrunPending = true; }

    // Optional external horizontal position fix (motion capture, camera).
    // Enables the x/y rows of H for one cycle. Unused in V0.1, where the
    // equivalent block was commented out, but the gains for it exist in Kf.
    void setExternalPosition(float x_m, float y_m);

    const StateEstimate&  getState() const { return _state; }
    const EstimatorDebug& getDebug() const { return _dbg; }

    // The raw state vector, for anyone who wants the matrix form.
    const BLA::Matrix<6, 1>& getStateVector()      const { return _Xe; }
    const BLA::Matrix<6, 1>& getPredictionVector() const { return _Xpre; }

private:
    // Body -> world rotation for the current attitude. Built once per update
    // and reused for all three vectors; V0.1's rotate_to_world() rebuilt it
    // from scratch on each of the three calls, recomputing nine trig
    // functions three times per cycle.
    static BLA::Matrix<3, 3> bodyToWorld(float roll_rad,
                                         float pitch_rad,
                                         float yaw_rad);

    static bool isFinite(float v) { return v == v && v < 1e30f && v > -1e30f; }

    // ── Fixed model ───────────────────────────────────────────────────────
    // Not _B and _U: newlib's <ctype.h> defines _B, _U, _L, _N, _S, _P, _C
    // and _X as character-class macros, and Arduino.h pulls it in. A member
    // named _B expands to 0200 before the compiler ever sees it.
    const BLA::Matrix<6, 6> _A;      // state transition, from DT_S
    const BLA::Matrix<6, 3> _Bmat;   // acceleration input, from DT_S
    BLA::Matrix<6, 6>       _Kf;     // steady-state gain

    // ── Working state ─────────────────────────────────────────────────────
    BLA::Matrix<6, 6> _H;      // measurement selector, rebuilt each cycle
    BLA::Matrix<6, 1> _Z;      // measurement vector, rebuilt each cycle
    BLA::Matrix<6, 1> _Xe;     // estimate
    BLA::Matrix<6, 1> _Xpre;   // prediction
    BLA::Matrix<3, 1> _Uaccel; // world-frame acceleration input

    StateEstimate  _state;
    EstimatorDebug _dbg;

    // Previous tilt-compensated height, for the diagnostic vz derivative
    float _prevZWorld_m = 0.0f;

    // Gyro integrated at the estimator rate, snapshotted when flow is fused
    // so compensation uses the average rate over the flow interval (PX4).
    float _gyroIntX_rad = 0.0f;
    float _gyroIntY_rad = 0.0f;
    float _gyroIntXAtFlow_rad = 0.0f;
    float _gyroIntYAtFlow_rad = 0.0f;

    // Cleared by reset(). The first valid altitude afterwards is written
    // straight into the state instead of being treated as a measurement to
    // converge toward -- see the note in update().
    bool  _seeded = false;

    bool  _overrunPending = false;

    // External position fix, consumed on the next update
    bool  _extPosFresh = false;
    float _extPosX_m   = 0.0f;
    float _extPosY_m   = 0.0f;
};
