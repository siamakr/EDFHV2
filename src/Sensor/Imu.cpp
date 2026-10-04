#include "Imu.h"
#include "consts.h"
#include <Arduino.h>

Imu::Imu(uint8_t csPin, uint8_t wakePin, uint8_t intPin, uint8_t rstPin)
    : Sensor("imu")
    , _csPin(csPin), _wakePin(wakePin), _intPin(intPin), _rstPin(rstPin)
{}

bool Imu::initialize()
{
    if (!_bno.beginSPI(_csPin, _wakePin, _intPin, _rstPin, cfg::imu::SPI_HZ)) {
        _initialized = false;
        return false;
    }

    _bno.calibrateAll();
    _bno.enableLinearAccelerometer(cfg::imu::REPORT_INTERVAL_MS);
    _bno.enableRotationVector(cfg::imu::REPORT_INTERVAL_MS);
    _bno.enableGyro(cfg::imu::REPORT_INTERVAL_MS);

    _initialized = true;
    return true;
}

bool Imu::sample()
{
    if (!_initialized || !_bno.dataAvailable()) {
        return false;
    }

    applyMounting();

    _d.yawRel_rad = wrapPi(_d.yaw_rad - _yawOrigin_rad);

    markSampled(micros());
    serviceCalibration(millis());
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// The mounting transform. X and Y are exchanged on every triple, and roll and
// pitch are exchanged on the Euler solution, because the part is mounted
// rotated relative to the body.
//
// See the handedness warning in Imu.h before touching any of this: the
// exchange is a reflection rather than a rotation, and the controller gains
// were derived against it as-is.
// ─────────────────────────────────────────────────────────────────────────────
void Imu::applyMounting()
{
    using namespace cfg::imu;

    // ── Linear acceleration (m/s^2, gravity already removed) ──────────────
    _d.ayRaw_mps2 = _bno.getLinAccelX();
    _d.axRaw_mps2 = _bno.getLinAccelY();
    _d.azRaw_mps2 = _bno.getLinAccelZ();

    _d.ax_mps2 = iir(_d.axRaw_mps2, _d.ax_mps2, ALPHA_ACCEL);
    _d.ay_mps2 = iir(_d.ayRaw_mps2, _d.ay_mps2, ALPHA_ACCEL);
    _d.az_mps2 = iir(_d.azRaw_mps2, _d.az_mps2, ALPHA_ACCEL);
    _d.linAccuracy = _bno.getLinAccelAccuracy();

    // ── Angular rate (rad/s) ──────────────────────────────────────────────
    _d.gx_rps = iir(_bno.getGyroY(), _d.gx_rps, ALPHA_GYRO);
    _d.gy_rps = iir(_bno.getGyroX(), _d.gy_rps, ALPHA_GYRO);
    _d.gz_rps = iir(_bno.getGyroZ(), _d.gz_rps, ALPHA_GYRO);
    _d.gyroAccuracy = _bno.getGyroAccuracy();

    // ── Orientation quaternion, unmodified ────────────────────────────────
    _bno.getQuat(_d.qi, _d.qj, _d.qk, _d.qw,
                 _d.quatRadianAccuracy, _d.quatAccuracy);

    // ── Euler attitude, plus fixed mount offsets ──────────────────────────
    _d.roll_rad  = _bno.getPitch() + ROLL_OFFSET_RAD;
    _d.pitch_rad = _bno.getRoll()  + PITCH_OFFSET_RAD;
    _d.yaw_rad   = _bno.getYaw()   + YAW_OFFSET_RAD;
}

// ─────────────────────────────────────────────────────────────────────────────
// Calibration
// ─────────────────────────────────────────────────────────────────────────────

bool Imu::isFullyCalibrated() const
{
    return _d.linAccuracy  >= cfg::imu::ACCURACY_TARGET
        && _d.gyroAccuracy >= cfg::imu::ACCURACY_TARGET
        && _d.quatAccuracy >= cfg::imu::ACCURACY_TARGET;
}

uint32_t Imu::getCalibratedDwellMs() const
{
    if (_dwellStartMs == 0) return 0;
    return millis() - _dwellStartMs;
}

void Imu::requestCalibrationSave()
{
    // Only a flag. The work happens in serviceCalibration(), inside sample().
    _savePending = true;
}

void Imu::serviceCalibration(uint32_t nowMs)
{
    // ── Track how long we have been fully calibrated ──────────────────────
    if (isFullyCalibrated()) {
        if (_dwellStartMs == 0) {
            _dwellStartMs = nowMs;
        }
        // Unattended save once the dwell is satisfied. Fires once.
        if (_autoSaveEnabled && !_autoSaved &&
            (nowMs - _dwellStartMs) >= cfg::imu::CAL_DWELL_MS) {
            _savePending = true;
            _autoSaved   = true;
        }
    } else {
        // Accuracy dropped -- the dwell has to start over.
        _dwellStartMs = 0;
    }

    // ── Kick off a pending save ───────────────────────────────────────────
    if (_savePending && _calState != CalState::Saving) {
        _bno.saveCalibration();
        _bno.requestCalibrationStatus();
        _saveDeadlineMs = nowMs + cfg::imu::CAL_SAVE_TIMEOUT_MS;
        _calState       = CalState::Saving;
        _savePending    = false;
        return;
    }

    // ── Await the acknowledgement, without blocking ───────────────────────
    if (_calState == CalState::Saving) {
        if (_bno.calibrationComplete()) {
            _calState = CalState::Saved;
        } else if ((int32_t)(nowMs - _saveDeadlineMs) >= 0) {
            _calState = CalState::Failed;
            markError();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────

void Imu::setYawOrigin()
{
    _yawOrigin_rad = _d.yaw_rad;
    _d.yawRel_rad  = 0.0f;
}

float Imu::wrapPi(float angle_rad)
{
    while (angle_rad >   cfg::unit::PI_F) angle_rad -= 2.0f * cfg::unit::PI_F;
    while (angle_rad <= -cfg::unit::PI_F) angle_rad += 2.0f * cfg::unit::PI_F;
    return angle_rad;
}
