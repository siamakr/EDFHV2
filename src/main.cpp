#include <Arduino.h>

#include "consts.h"

#include "Actuator.h"
#include "RCSActuator.h"
#include "ActuatorManager.h"

#include "Imu.h"
#include "Lidar.h"
#include "OpticalFlow.h"
#include "SensorManager.h"
#include "Estimator.h"

#include "AttitudeController.h"
#include "ControlAllocator.h"

// ─────────────────────────────────────────────────────────────────────────────
// BRING-UP SKELETON -- NOT A FLIGHT PROGRAM
//
// This wires the rebuilt subsystems together and runs the sensing half of the
// loop at 200 Hz. It does NOT fly: there is no state machine, no arming
// sequence, no launch/hover/land logic, and no actuator output. The EDF is
// held at OFF_US and the gimbals at neutral for the entire run.
//
// Its purpose is to be the thing you can actually flash: it proves the whole
// tree compiles and links for the Teensy, that every sensor comes up on its
// bus, and that the estimator runs inside the 5 ms budget on real hardware.
//
// The flight state machine from V0.1 is preserved verbatim at
// old-files/main_v0.1.cpp and is the reference for the rewrite. Two things
// from docs/REBUILD_NOTES.md must change when porting it:
//
//   - the estimator has to run BEFORE the controller. V0.1 called
//     run_estimator() after the LQR, so z and vz were a full cycle stale.
//   - the yaw torque -> grams hop is gone. RCSActuator is calibrated
//     directly in force, so do not reintroduce it.
//
// Serial is owned HERE and nowhere else. No sensor or actuator class reads
// it. That is deliberate: in V0.1 the IMU calibration routine called
// Serial.read() from inside the sampling path and swallowed the byte the
// flight state machine was waiting on, which looked like a freeze.
// ─────────────────────────────────────────────────────────────────────────────

// ── Actuators ───────────────────────────────────────────────────────────────
// The EDF is the only channel that passes a cutoff and an off pulse: its
// thrust regression is only valid from 25.2 N up, so a zero command cannot go
// through the regression. See the note in Actuator.h.
static Actuator edf(cfg::pin::EDF,
                    cfg::edf::P0, cfg::edf::P1, cfg::edf::P2,
                    cfg::edf::MIN_N, cfg::edf::MAX_N, cfg::edf::NEUTRAL_N,
                    cfg::edf::CUTOFF_N, cfg::edf::OFF_US);

static Actuator gimbalX(cfg::pin::GIMBAL_X,
                        cfg::gimbal::x::P0, cfg::gimbal::x::P1, cfg::gimbal::x::P2,
                        cfg::gimbal::x::MIN_DEG, cfg::gimbal::x::MAX_DEG,
                        cfg::gimbal::x::NEUTRAL_DEG);

static Actuator gimbalY(cfg::pin::GIMBAL_Y,
                        cfg::gimbal::y::P0, cfg::gimbal::y::P1, cfg::gimbal::y::P2,
                        cfg::gimbal::y::MIN_DEG, cfg::gimbal::y::MAX_DEG,
                        cfg::gimbal::y::NEUTRAL_DEG);

static RCSActuator rcs(cfg::pin::RCS,
                       cfg::rcs::P0, cfg::rcs::P1, cfg::rcs::P2,
                       cfg::rcs::MIN_N, cfg::rcs::MAX_N,
                       cfg::vehicle::COM_TO_RCS_M);

// ActuatorManager names its gimbal channels pitch/roll; ControlAllocator names
// them Y/X. They are the same two servos: the roll channel drives X and the
// pitch channel drives Y. Spelled out here because getting it backwards is a
// tumble, and this is the only place the two namings meet.
static ActuatorManager actuators(edf,
                                 /* gimbalPitch = */ gimbalY,
                                 /* gimbalRoll  = */ gimbalX,
                                 rcs);

// ── Sensing ─────────────────────────────────────────────────────────────────
static Imu           imu(cfg::pin::IMU_CS, cfg::pin::IMU_WAK,
                         cfg::pin::IMU_INT, cfg::pin::IMU_RST);
static Lidar         lidar;
static OpticalFlow   flow;
static SensorManager sensors(imu, lidar, flow);
static Estimator     estimator;

// ── Control ─────────────────────────────────────────────────────────────────
static AttitudeController attitude;
static ControlAllocator   allocator;

// ── Loop scheduling ─────────────────────────────────────────────────────────
static uint32_t lastTickUs = 0;
static uint32_t overruns   = 0;

// ─────────────────────────────────────────────────────────────────────────────
// The single reader of the serial port. Add flight commands here as the state
// machine goes in; nothing below main owns stdin.
// ─────────────────────────────────────────────────────────────────────────────
static void handleSerialCommands()
{
    if (!Serial.available()) return;

    switch (Serial.read()) {
        case 's':
            // Manual override. Not normally needed -- Imu saves by itself once
            // all three accuracies have held at 3 for CAL_DWELL_MS, which is
            // the condition you are watching for while rotating the box.
            imu.requestCalibrationSave();
            break;

        case 'o':
            imu.setYawOrigin();
            break;

        case 'r':
            estimator.reset();
            attitude.resetFilter();
            break;

        default:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    // Actuators first, and before anything can command them. initialize()
    // attaches and immediately drives each channel to its rest position --
    // the Servo library latches 1500 us on attach(), which on this EDF is
    // about 21 N of thrust.
    actuators.initialize();

    const bool sensorsOk = sensors.initialize();

    Serial.println(sensorsOk ? "sensors: all up"
                             : "sensors: one or more FAILED");
    Serial.print("  imu   "); Serial.println(imu.isInitialized());
    Serial.print("  lidar "); Serial.println(lidar.isInitialized());
    Serial.print("  flow  "); Serial.println(flow.isInitialized());

    estimator.reset();
    lastTickUs = micros();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    const uint32_t now     = micros();
    const uint32_t elapsed = now - lastTickUs;

    if (elapsed < cfg::loop::DT_US) return;
    lastTickUs = now;

    // The estimator integrates against a fixed DT_S. If the loop fell far
    // enough behind that the real interval no longer resembles it, tell the
    // estimator to hold rather than step on a bad dt.
    if (elapsed > cfg::loop::DT_MAX_US) {
        estimator.flagOverrun();
        ++overruns;
    }

    handleSerialCommands();

    // Estimate BEFORE control -- see the note at the top of this file.
    sensors.sampleAll();
    estimator.update(sensors.getFrame());

    const StateEstimate& state = estimator.getState();
    const ImuData&       att   = imu.getData();

    // The control law runs so its timing is representative, but the result is
    // deliberately discarded: there is no arming logic yet, and setGimbal() on
    // ActuatorManager is NOT gated by the armed flag. Wiring the allocator
    // output through to the actuators is the job of the state machine.
    BLA::Matrix<4, 1> u = attitude.update(att.roll_rad, att.pitch_rad, att.yaw_rad,
                                          att.gx_rps, att.gy_rps, att.gz_rps,
                                          state.z_m, state.vz_mps);

    const float uArr[4] = { u(0), u(1), u(2), u(3) };
    const AllocatorOutput cmd = allocator.allocate(uArr);
    (void)cmd;
}
