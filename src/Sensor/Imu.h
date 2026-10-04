#pragma once
#include "Sensor.h"
#include <BNO080.h>

// ─────────────────────────────────────────────────────────────────────────────
// Imu -- BNO080 (FSM305 breakout) over SPI
//
// Owns everything about turning the chip's output into BODY-frame attitude,
// rates and accelerations. Nothing downstream should ever have to remember
// that the part is mounted rotated, or that roll and pitch come out swapped.
// In V0.1 that knowledge was smeared across sample_fsm() in Sensors.cpp and
// two loose consts in Sensors.h; a reader of the estimator had no way to know
// the axes had already been exchanged.
//
// MOUNTING TRANSFORM (see applyMounting() in the .cpp for the exact code)
// ──────────────────────────────────────────────────────────────────────
// The part is rotated relative to the body, so X and Y are exchanged:
//
//     body ax    <- sensor linAccel Y        body roll  <- sensor pitch
//     body ay    <- sensor linAccel X        body pitch <- sensor roll
//     body az    <- sensor linAccel Z        body yaw   <- sensor yaw
//     body gx    <- sensor gyro Y
//     body gy    <- sensor gyro X
//     body gz    <- sensor gyro Z
//
// !! Carried over from V0.1 EXACTLY as it flew, including a wrinkle: a plain
// X<->Y exchange with Z left alone has determinant -1, so it is a REFLECTION,
// not a rotation. A true 90 deg yaw rotation would be (x,y) <- (y,-x) -- note
// the sign. As written this mirrors the frame, which inverts the sense of one
// rotational axis.
//
// This is NOT changed here, deliberately. The vehicle flew with it, and the
// LQR gain K was derived against whatever convention the flight data actually
// had -- AttitudeController already negates yaw and gyroZ for the same reason.
// A sign error here and a compensating sign in K cancel in closed loop. Fixing
// one without the other would fly worse, not better.
//
// Resolve it on the bench, not in the air: rotate the airframe nose-up and
// confirm pitch and gyroY are positive, then roll right and confirm roll and
// gyroX are positive. If a sign is inverted, fix it HERE (one line) and
// re-derive K, rather than patching signs at the controller.
//
// CALIBRATION
// ───────────
// The BNO reports a 0-3 accuracy per subsystem and self-calibrates as it sees
// different orientations, which is why the procedure is to rotate the vehicle
// through a box pattern. Two ways to commit the result:
//
//   - automatic: once all three accuracies have held at ACCURACY_TARGET
//     continuously for CAL_DWELL_MS, the save fires by itself. This is exactly
//     the condition you were watching for by eye, so no keypress is needed.
//   - manual: requestCalibrationSave() from a serial command handler in main.
//
// Both are non-blocking. V0.1 did the save inside sample_fsm() with a
// `while(1)` polling loop plus `delay(1)` and a `delay(1000)` on success --
// up to 1.1 s of stall in the 200 Hz sample path. Here the request only sets a
// flag, and sample() advances a small state machine against a deadline.
//
// Also note V0.1 read Serial *inside* the sensor to look for 's', which
// swallowed whatever byte the flight state machine was waiting on. This class
// does not touch Serial at all; main owns the port and dispatches commands.
// ─────────────────────────────────────────────────────────────────────────────

struct ImuData {
    // Body-frame attitude, mounting transform and offsets already applied
    float roll_rad  = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad   = 0.0f;     // raw sensor yaw
    float yawRel_rad = 0.0f;    // yaw relative to the origin set at init

    // Body-frame angular rate, IIR filtered
    float gx_rps = 0.0f;
    float gy_rps = 0.0f;
    float gz_rps = 0.0f;

    // Body-frame linear acceleration (gravity already removed by the BNO),
    // IIR filtered
    float ax_mps2 = 0.0f;
    float ay_mps2 = 0.0f;
    float az_mps2 = 0.0f;

    // Orientation quaternion, straight from the sensor (NOT remapped -- it is
    // carried for logging and for anyone who wants to redo the maths)
    float qi = 0.0f, qj = 0.0f, qk = 0.0f, qw = 1.0f;

    // Unfiltered accelerations, kept for filter tuning from logs
    float axRaw_mps2 = 0.0f;
    float ayRaw_mps2 = 0.0f;
    float azRaw_mps2 = 0.0f;

    uint8_t linAccuracy  = 0;   // 0-3
    uint8_t gyroAccuracy = 0;   // 0-3
    uint8_t quatAccuracy = 0;   // 0-3
    float   quatRadianAccuracy = 0.0f;
};

class Imu : public Sensor {
public:
    enum class CalState : uint8_t { Idle, Saving, Saved, Failed };

    Imu(uint8_t csPin, uint8_t wakePin, uint8_t intPin, uint8_t rstPin);

    bool initialize() override;

    // One packet, if the chip has one. Call on EVERY loop() pass -- if this
    // stops, the BNO queue backs up and the values freeze.
    bool sample() override;

    // ── Data ──────────────────────────────────────────────────────────────
    const ImuData& getData() const { return _d; }

    // ── Yaw origin ────────────────────────────────────────────────────────
    // Captures current heading as zero. Called once at init and again on
    // arming, so "yaw = 0" means "pointing where it was when armed".
    void  setYawOrigin();
    float getYawOrigin() const { return _yawOrigin_rad; }

    // ── Calibration ───────────────────────────────────────────────────────
    void     requestCalibrationSave();
    CalState getCalibrationState()  const { return _calState; }
    bool     isFullyCalibrated()    const;
    uint32_t getCalibratedDwellMs() const;
    bool     didAutoSave()          const { return _autoSaved; }

    // Enable/disable the unattended save. On by default.
    void setAutoSave(bool on) { _autoSaveEnabled = on; }

private:
    void applyMounting();
    void serviceCalibration(uint32_t nowMs);

    static float iir(float newSample, float prevOutput, float alpha)
    {
        return (1.0f - alpha) * newSample + alpha * prevOutput;
    }

    // Wrap to (-PI, PI]
    static float wrapPi(float angle_rad);

    BNO080  _bno;
    uint8_t _csPin, _wakePin, _intPin, _rstPin;

    ImuData _d;
    float   _yawOrigin_rad = 0.0f;

    // Calibration state
    CalState _calState        = CalState::Idle;
    bool     _savePending     = false;
    bool     _autoSaved       = false;
    bool     _autoSaveEnabled = true;
    uint32_t _saveDeadlineMs  = 0;
    uint32_t _dwellStartMs    = 0;   // 0 = not currently at full accuracy
};
