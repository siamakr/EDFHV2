#include "Wayfinder.h"
#include "consts.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>

namespace {
    const MissionEntry kDefaultMission[] = {
        { "HOVER 1", 0.30f, 0.00f, 0.00f, 2.5f },
        { "LAND",    0.00f, 0.00f, 0.00f, 0.0f },
    };
    constexpr uint8_t kDefaultMissionLen =
        sizeof(kDefaultMission) / sizeof(kDefaultMission[0]);
}

Wayfinder::Wayfinder(PositionController& position,
                     AttitudeController& attitude,
                     ActuatorManager&    actuators,
                     Actuator&           edf,
                     Imu&                imu)
    : _position(position)
    , _attitude(attitude)
    , _actuators(actuators)
    , _edf(edf)
    , _imu(imu)
{
    _missionLen = kDefaultMissionLen;
    if (_missionLen > MAX_MISSION) _missionLen = MAX_MISSION;
    for (uint8_t i = 0; i < _missionLen; ++i) {
        _mission[i] = kDefaultMission[i];
    }
}

void Wayfinder::start(float lidar_z_m)
{
    _imu.setYawOrigin();
    _attitude.setYawRef(0.0f);
    _index             = 0;
    _landed            = false;
    _landingConfirmed  = false;
    _complete          = false;
    _elapsedUs         = 0;
    _startUs           = micros();
    _position.reset();
    _actuators.arm();

    if (_mode == Mode::AttitudeTest) {
        _zHold_m = lidar_z_m;
        loadAttitude(0);
    } else {
        loadFlight(1, _mission[0]);
    }
}

void Wayfinder::abort()
{
    _active           = false;
    _complete         = true;
    _landingConfirmed = false;
    _actuators.disarm();
    _actuators.safeAll();
    _edf.writeRaw(cfg::edf::OFF_US);
}

void Wayfinder::cut()
{
    _landingConfirmed = true;
    _landed           = true;
    _complete         = true;
    _active           = true;
    _actuators.disarm();
    _actuators.safeAll();
    _edf.writeRaw(cfg::edf::OFF_US);
}

void Wayfinder::update(const StateEstimate& est)
{
    if (_mode == Mode::AttitudeTest) {
        serviceAttitude();
        if (!_active || _landingConfirmed) return;
        _attitude.setRollRef(_rollRef_deg * cfg::unit::D2R);
        _attitude.setPitchRef(_pitchRef_deg * cfg::unit::D2R);
        _attitude.setZRef(_zHold_m);
        return;
    }

    serviceFlight(est);
    if (!_active || _landingConfirmed) return;

    _position.setReference(_wp.x_m, _wp.y_m);
    const AttitudeSetpoint attSp = _position.update(
        est.x_m, est.y_m, est.vx_mps, est.vy_mps);
    _attitude.setRollRef(attSp.roll_rad);
    _attitude.setPitchRef(attSp.pitch_rad);
    _attitude.setZRef(_wp.alt_m);
}

void Wayfinder::loadFlight(uint8_t number, const MissionEntry& entry)
{
    _wp = Waypoint{};
    _wp.number  = number;
    strncpy(_wp.name, entry.name ? entry.name : "", sizeof(_wp.name) - 1);
    _wp.isLand  = (strcmp(_wp.name, "LAND") == 0);
    _wp.alt_m   = _wp.isLand ? 0.0f : entry.alt_m;
    _wp.x_m     = _wp.isLand ? 0.0f : entry.x_m;
    _wp.y_m     = _wp.isLand ? 0.0f : entry.y_m;
    _wp.dwell_s = entry.dwell_s;

    _active            = true;
    _complete          = false;
    _landed            = false;
    _landingConfirmed  = false;
    _elapsedUs         = 0;
    if (!_wp.isLand) _prevAlt_m = _wp.alt_m;

    _position.setReference(_wp.x_m, _wp.y_m);
    _position.reset();
    _attitude.setZRef(_wp.alt_m);
}

void Wayfinder::loadAttitude(uint8_t index)
{
    _index        = index;
    _rollRef_deg  = cfg::attTest::SETPOINTS[index][0];
    _pitchRef_deg = cfg::attTest::SETPOINTS[index][1];

    _wp = Waypoint{};
    _wp.number  = index + 1;
    snprintf(_wp.name, sizeof(_wp.name), "ATT %u", (unsigned)(index + 1));
    _wp.alt_m   = _zHold_m;
    _wp.dwell_s = cfg::attTest::DWELL_S;
    _wp.isLand  = false;

    _active            = true;
    _complete          = false;
    _landed            = false;
    _landingConfirmed  = false;
    _elapsedUs         = 0;

    _attitude.setRollRef(_rollRef_deg * cfg::unit::D2R);
    _attitude.setPitchRef(_pitchRef_deg * cfg::unit::D2R);
    _attitude.setYawRef(0.0f);
    _attitude.setZRef(_zHold_m);
}

void Wayfinder::serviceFlight(const StateEstimate& est)
{
    if (!_active || _complete) return;

    const float scale = _wp.isLand ? _prevAlt_m : fabsf(_wp.alt_m);
    const float xyF   = _wp.isLand ? cfg::wp::LAND_TOL_XY : cfg::wp::WP_TOL_XY;
    const float zF    = _wp.isLand ? cfg::wp::LAND_TOL_Z  : cfg::wp::WP_TOL_Z;
    const float zTol  = zF * ((scale > 1e-4f) ? scale : 1.0f);
    const bool xyIn   = inTolPct(est.x_m, _wp.x_m, xyF, scale)
                     && inTolPct(est.y_m, _wp.y_m, xyF, scale);
    const bool zIn    = _wp.isLand
                        ? (est.z_m <= _wp.alt_m + zTol)
                        : inTolPct(est.z_m, _wp.alt_m, zF, scale);
    const bool inBand = xyIn && zIn;

    _attitude.setZRef(_wp.alt_m);

    if (_wp.isLand && inBand) {
        cut();
        return;
    }

    if (!inBand) {
        _elapsedUs = 0;
        return;
    }

    _elapsedUs += cfg::loop::DT_US;

    if ((_elapsedUs / 1000000.0f) >= _wp.dwell_s) {
        _complete = true;
        if (_index + 1 < _missionLen) {
            ++_index;
            loadFlight(_index + 1, _mission[_index]);
        }
    }
}

void Wayfinder::serviceAttitude()
{
    if (!_active || _complete) return;

    _elapsedUs += cfg::loop::DT_US;

    if ((_elapsedUs / 1000000.0f) >= cfg::attTest::DWELL_S) {
        if (_index + 1 < cfg::attTest::N) {
            loadAttitude(_index + 1);
        } else {
            cut();
        }
    }
}

bool Wayfinder::inTolPct(float meas, float target, float frac, float zeroScale)
{
    const float scale = (fabsf(target) > 1e-4f) ? fabsf(target) : zeroScale;
    return fabsf(meas - target) <= frac * scale;
}

float Wayfinder::dwellTarget_s() const
{
    return (_mode == Mode::AttitudeTest) ? cfg::attTest::DWELL_S : _wp.dwell_s;
}

const char* Wayfinder::csvHeader()
{
    return "t_s,mode,step,tgt_r_deg,tgt_p_deg,tgt_x_m,tgt_y_m,tgt_z_m,"
           "est_x_m,est_y_m,est_z_m,r_deg,p_deg,y_deg,gx,gy,gz,"
           "gimb_x_deg,gimb_y_deg,gimb_x_us,gimb_y_us,F_N,edf_us,yawT_Nm\n";
}

void Wayfinder::formatLine(char* buf, size_t n,
                           const StateEstimate& est,
                           const ImuData& att,
                           float gimbalX_deg, float gimbalY_deg,
                           uint16_t edfUs) const
{
    using cfg::unit::R2D;
    const auto& u = _attitude.getLastOutput();
    const float F = _landingConfirmed ? 0.0f : (float)u(3);
    const char* cut = _landingConfirmed ? "  CUT" : "";

    if (_mode == Mode::AttitudeTest) {
        snprintf(buf, n,
            "att %u  tgt r=%+5.2f p=%+5.2f  meas r=%6.2f p=%6.2f y=%6.2f  "
            "t=%4.1f/%4.1f  z=%6.3f/%6.3f  gimb x=%+5.2f y=%+5.2f  "
            "F=%5.1f N  edf=%4u us%s\n",
            (unsigned)_wp.number,
            _rollRef_deg, _pitchRef_deg,
            att.roll_rad * R2D, att.pitch_rad * R2D, att.yawRel_rad * R2D,
            dwellElapsed_s(), dwellTarget_s(),
            est.z_m, _zHold_m,
            gimbalX_deg, gimbalY_deg,
            F, edfUs, cut);
        return;
    }

    const char* tag = _landingConfirmed ? "  LANDING_CONFIRMED"
                    : (_landed         ? "  LANDED"
                    : (_complete       ? "  done" : ""));

    snprintf(buf, n,
        "wp %u %s  tgt x=%5.2f y=%5.2f z=%5.2f m  "
        "est x=%6.3f y=%6.3f z=%6.3f m  t=%4.1f/%4.1f%s  "
        "r=%6.2f p=%6.2f y=%6.2f  gimb x=%+5.2f y=%+5.2f  "
        "yawT=%5.3f Nm  F=%5.1f N  edf=%4u us%s\n",
        (unsigned)_wp.number, _wp.name,
        _wp.x_m, _wp.y_m, _wp.alt_m,
        est.x_m, est.y_m, est.z_m,
        dwellElapsed_s(), _wp.dwell_s, tag,
        att.roll_rad * R2D, att.pitch_rad * R2D, att.yawRel_rad * R2D,
        gimbalX_deg, gimbalY_deg, (float)u(2),
        F, edfUs, cut);
}

void Wayfinder::formatCsv(char* buf, size_t n,
                          const StateEstimate& est,
                          const ImuData& att,
                          float gimbalX_deg, float gimbalY_deg,
                          uint16_t gimbalX_us, uint16_t gimbalY_us,
                          uint16_t edfUs) const
{
    using cfg::unit::R2D;
    const auto& u = _attitude.getLastOutput();
    const float t_s = (_startUs == 0) ? 0.0f
                    : (micros() - _startUs) / 1000000.0f;
    const float F = _landingConfirmed ? 0.0f : (float)u(3);
    const char* mode = (_mode == Mode::AttitudeTest) ? "att" : "flt";

    snprintf(buf, n,
        "%.3f,%s,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%u,%u,%.3f,%u,%.4f\n",
        t_s, mode, (unsigned)_wp.number,
        _rollRef_deg, _pitchRef_deg,
        _wp.x_m, _wp.y_m,
        (_mode == Mode::AttitudeTest) ? _zHold_m : _wp.alt_m,
        est.x_m, est.y_m, est.z_m,
        att.roll_rad * R2D, att.pitch_rad * R2D, att.yawRel_rad * R2D,
        att.gx_rps, att.gy_rps, att.gz_rps,
        gimbalX_deg, gimbalY_deg,
        (unsigned)gimbalX_us, (unsigned)gimbalY_us,
        F, (unsigned)edfUs, (float)u(2));
}
