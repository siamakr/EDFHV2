#include "Estimator.h"
#include "consts.h"
#include <math.h>

namespace {
    constexpr float DT   = cfg::loop::DT_S;
    constexpr float HDT2 = 0.5f * cfg::loop::DT_S * cfg::loop::DT_S;
}

Estimator::Estimator()
    // Constant-acceleration kinematics over one fixed tick.
    //        x  y  z  vx  vy  vz
    : _A{ 1, 0, 0, DT,  0,  0,      // x  += vx*dt
          0, 1, 0,  0, DT,  0,      // y  += vy*dt
          0, 0, 1,  0,  0, DT,      // z  += vz*dt
          0, 0, 0,  1,  0,  0,      // vx
          0, 0, 0,  0,  1,  0,      // vy
          0, 0, 0,  0,  0,  1 }     // vz
    //       ax    ay    az
    , _Bmat{ HDT2,    0,    0,         // x  += 0.5*ax*dt^2
             0, HDT2,    0,         // y
             0,    0, HDT2,         // z
            DT,    0,    0,         // vx += ax*dt
             0,   DT,    0,         // vy
             0,    0,   DT }        // vz
{
    using namespace cfg::est;

    // Steady-state gain. Rows are states, columns are measurement channels in
    // the same order: [x, y, z, vx, vy, vz].
    _Kf = { K_POS,     0,     0, K_POS_VEL,         0,      0,
                0, K_POS,     0,         0, K_POS_VEL,      0,
                0,     0,   K_Z,         0,         0, K_Z_VZ,
        K_POS_VEL,     0,     0,     K_VEL,         0,      0,
                0, K_POS_VEL, 0,         0,     K_VEL,      0,
                0,     0, K_VZ_Z,       0,         0,   K_VZ };

    reset();
}

void Estimator::reset()
{
    _Xe.Fill(0.0f);
    _Xpre.Fill(0.0f);
    _H.Fill(0.0f);
    _Z.Fill(0.0f);
    _Uaccel.Fill(0.0f);
    _state         = StateEstimate{};
    _prevZWorld_m        = 0.0f;
    _gyroIntX_rad        = 0.0f;
    _gyroIntY_rad        = 0.0f;
    _gyroIntXAtFlow_rad  = 0.0f;
    _gyroIntYAtFlow_rad  = 0.0f;
    _extPosFresh         = false;
    _seeded              = false;
}

void Estimator::setExternalPosition(float x_m, float y_m)
{
    _extPosX_m   = x_m;
    _extPosY_m   = y_m;
    _extPosFresh = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// ZYX body-to-world rotation. Carried over unchanged from V0.1's
// rotate_to_world(), just hoisted so the trig is evaluated once per cycle.
// ─────────────────────────────────────────────────────────────────────────────
BLA::Matrix<3, 3> Estimator::bodyToWorld(float roll_rad,
                                         float pitch_rad,
                                         float yaw_rad)
{
    const float p = roll_rad;
    const float q = pitch_rad;
    const float u = yaw_rad;

    const float sp = sinf(p), cp = cosf(p);
    const float sq = sinf(q), cq = cosf(q);
    const float su = sinf(u), cu = cosf(u);

    return BLA::Matrix<3, 3>{
        cq * cu,  sp * sq * cu - cp * su,  cp * sq * cu + sp * su,
        cq * su,  sp * sq * su + cp * cu,  cp * sq * su - sp * cu,
            -sq,                 sp * cq,                 cp * cq
    };
}

void Estimator::update(const SensorFrame& f)
{
    // A cycle that arrived far off schedule would be integrated with the wrong
    // dt. Hold the state rather than corrupt it, and skip this measurement set.
    if (_overrunPending) {
        _overrunPending = false;
        ++_dbg.overruns;
        return;
    }

    const BLA::Matrix<3, 3> R =
        bodyToWorld(f.imu.roll_rad, f.imu.pitch_rad, f.imu.yaw_rad);

    // ── Lidar: tilt-compensate to a true vertical height ──────────────────
    // The beam points along body -Z, so a tilted vehicle over flat ground
    // measures a longer slant range than its actual altitude.
    const BLA::Matrix<3, 1> pBody{ 0.0f, 0.0f, f.lidar.z_m };
    const BLA::Matrix<3, 1> pWorld = R * pBody;
    const float zWorld = pWorld(2);

    _dbg.zWorld_m = zWorld;

    // First valid altitude after a reset SEEDS the state rather than being
    // corrected toward. Starting from z = 0 while the vehicle actually sits
    // 0.92 m up leaves a residual that the Kf(5,2) = 1.0 gain converts almost
    // one-for-one into vertical velocity: measured here as a phantom 3.9 m/s
    // descent rate within five cycles of arming, fed straight to the altitude
    // loop. Seeding removes the transient entirely.
    if (!_seeded && f.lidarFresh && f.lidar.valid) {
        _Xe(2)        = zWorld;
        _prevZWorld_m = zWorld;
        _seeded       = true;
    }

    _dbg.vzLidar_mps = (zWorld - _prevZWorld_m) / DT;
    _prevZWorld_m    = zWorld;

    // ── Optical flow: gyro-compensate, scale by range, rotate ─────────────
    // Pixhawk / PX4:
    //   - convert counts to radians with FOCAL (385)
    //   - subtract the gyro *integral* over the same interval
    //   - scale by beam range (not tilt-compensated world Z)
    //   - v = (flow_rate - gyro_rate) * range
    //
    // Range is the lidar's raw beam length: the PMW3901 looks along the same
    // body -Z as the lidar. Using zWorld (feet AGL, tilt-projected, often
    // negative on the pad) inverted and under-scaled the velocity.
    _gyroIntX_rad += f.imu.gx_rps * DT;
    _gyroIntY_rad += f.imu.gy_rps * DT;

    float range = f.lidar.raw_cm * 0.01f;
    if (range < cfg::flow::MIN_RANGE_M) range = cfg::flow::MIN_RANGE_M;
    if (range > cfg::flow::MAX_RANGE_M) range = cfg::flow::MAX_RANGE_M;
    _dbg.flowRange_m = range;

    float gxAvg = f.imu.gx_rps;
    float gyAvg = f.imu.gy_rps;
    if (f.flowFresh && f.flow.dt_s > 0.0f) {
        gxAvg = (_gyroIntX_rad - _gyroIntXAtFlow_rad) / f.flow.dt_s;
        gyAvg = (_gyroIntY_rad - _gyroIntYAtFlow_rad) / f.flow.dt_s;
        _gyroIntXAtFlow_rad = _gyroIntX_rad;
        _gyroIntYAtFlow_rad = _gyroIntY_rad;
    }

    // Axis pairing matches a downward pinhole aligned to the body: pitch
    // (gy) produces X flow, roll (gx) produces Y flow. Same as V0.1.
    const BLA::Matrix<3, 1> vBody{
        (f.flow.vx - gyAvg) * range,
        (f.flow.vy - gxAvg) * range,
        0.0f
    };
    const BLA::Matrix<3, 1> vWorld = R * vBody;

    _dbg.vxMeas_mps = vWorld(0);
    _dbg.vyMeas_mps = vWorld(1);

    // ── Accelerometer: rotate to world, becomes the prediction input ──────
    const BLA::Matrix<3, 1> aBody{ f.imu.ax_mps2, f.imu.ay_mps2, f.imu.az_mps2 };
    const BLA::Matrix<3, 1> aWorld = R * aBody;

    _dbg.axw_mps2 = aWorld(0);
    _dbg.ayw_mps2 = aWorld(1);
    _dbg.azw_mps2 = aWorld(2);

    _Uaccel = aWorld;

    // ── Assemble the measurement, enabling only fresh channels ────────────
    _H.Fill(0.0f);
    _Z.Fill(0.0f);

    _dbg.usedPos   = false;
    _dbg.usedLidar = false;
    _dbg.usedFlow  = false;

    if (_extPosFresh) {
        _H(0, 0) = 1.0f; _Z(0) = _extPosX_m;
        _H(1, 1) = 1.0f; _Z(1) = _extPosY_m;
        _extPosFresh   = false;
        _dbg.usedPos   = true;
    }

    if (f.lidarFresh && f.lidar.valid) {
        _H(2, 2) = 1.0f; _Z(2) = zWorld;
        _dbg.usedLidar = true;
    }

    if (f.flowFresh) {
        _H(3, 3) = 1.0f; _Z(3) = vWorld(0);
        _H(4, 4) = 1.0f; _Z(4) = vWorld(1);
        _dbg.usedFlow  = true;
    }

    // ── Predict, then correct ─────────────────────────────────────────────
    _Xpre = (_A * _Xe) + (_Bmat * _Uaccel);

    _dbg.xpreX_m     = _Xpre(0);
    _dbg.xpreY_m     = _Xpre(1);
    _dbg.xpreZ_m     = _Xpre(2);
    _dbg.xpreVx_mps  = _Xpre(3);
    _dbg.xpreVy_mps  = _Xpre(4);
    _dbg.xpreVz_mps  = _Xpre(5);

    _Xe = _Xpre + _Kf * (_Z - _H * _Xpre);

    // A single non-finite element poisons every subsequent cycle through the
    // feedback in A, so the whole state is dropped rather than patched.
    // V0.1 only tested element 0.
    for (int i = 0; i < 6; ++i) {
        if (!isFinite(_Xe(i))) {
            _Xe.Fill(0.0f);
            ++_dbg.nanResets;
            break;
        }
    }

    _state.x_m    = _Xe(0);
    _state.y_m    = _Xe(1);
    _state.z_m    = _Xe(2);
    _state.vx_mps = _Xe(3);
    _state.vy_mps = _Xe(4);
    _state.vz_mps = _Xe(5);

    ++_dbg.updateCount;
}
