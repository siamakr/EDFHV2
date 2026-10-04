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
// old-files/main_v0.1.cpp and is the reference for the rewrite. When porting:
//
//   - keep the estimator BEFORE the controller, as V0.1 had it.
//   - the yaw torque -> grams hop is gone. RCSActuator is calibrated
//     directly in Newtons, so do not reintroduce it.
//   - prime with actuators.primeEdf(cfg::edf::PRIME_US) for PRIME_MS, not
//     V0.1's 1250 us for 6 s.
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
                       cfg::vehicle::COM_TO_RCS_M,
                       cfg::rcs::CUTOFF_NM, cfg::rcs::OFF_US);

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
static uint32_t lastFlowUs = 0;

// ── Serial modes ────────────────────────────────────────────────────────────
// main   : quiet bring-up (default).  m returns here.
// inspect: IMU + lidar + estimator stream.  k enters here.
enum class SerialMode : uint8_t { Main, Inspect };
static SerialMode serialMode   = SerialMode::Main;
static uint32_t   lastPrintUs  = 0;

static void printInspectLine(uint32_t nowUs)
{
    const ImuData&        att = imu.getData();
    const LidarData&      rng = lidar.getData();
    const FlowData&       fl  = flow.getData();
    const StateEstimate&  est = estimator.getState();
    using cfg::unit::R2D;

    char line[320];
    snprintf(line, sizeof(line),
        "imu  r=%6.2f p=%6.2f y=%6.2f  gx=%6.3f gy=%6.3f gz=%6.3f  "
        "ax=%6.3f ay=%6.3f az=%6.3f  acc=%u/%u/%u\n"
        "lid  z=%6.3f raw=%4u\n"
        "flo  dx=%4d dy=%4d  vx=%6.3f vy=%6.3f\n"
        "est  x=%6.3f y=%6.3f z=%6.3f  vx=%6.3f vy=%6.3f vz=%6.3f\n",
        att.roll_rad  * R2D, att.pitch_rad * R2D, att.yaw_rad * R2D,
        att.gx_rps, att.gy_rps, att.gz_rps,
        att.ax_mps2, att.ay_mps2, att.az_mps2,
        att.linAccuracy, att.gyroAccuracy, att.quatAccuracy,
        rng.z_m, rng.raw_cm,
        (int)fl.dx_px, (int)fl.dy_px, fl.vx, fl.vy,
        est.x_m, est.y_m, est.z_m,
        est.vx_mps, est.vy_mps, est.vz_mps);
    Serial.print(line);
}

// ─────────────────────────────────────────────────────────────────────────────
// The single reader of the serial port. Add flight commands here as the state
// machine goes in; nothing below main owns stdin.
// ─────────────────────────────────────────────────────────────────────────────
static void handleSerialCommands()
{
    if (!Serial.available()) return;

    switch (Serial.read()) {
        case 'k':
            serialMode = SerialMode::Inspect;
            Serial.println("mode: inspect  (m = main)");
            break;

        case 'm':
            serialMode = SerialMode::Main;
            Serial.println("mode: main  (k = inspect)");
            break;

        case 's':
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
    // about 21 N of thrust. EDF and RCS both land on their 900 us off pulse.
    edf.setPwmLimits(cfg::edf::FLIGHT_MIN_US, cfg::edf::MAX_US);
    actuators.initialize();

    const bool sensorsOk = sensors.initialize();

    Serial.println(sensorsOk ? "sensors: all up"
                             : "sensors: one or more FAILED");
    Serial.print("  imu   "); Serial.println(imu.isInitialized());
    Serial.print("  lidar "); Serial.println(lidar.isInitialized());
    Serial.print("  flow  "); Serial.println(flow.isInitialized());
    Serial.println("k = inspect   m = main");

    estimator.reset();
    lastTickUs = lastFlowUs = micros();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    // V0.1: sample the BNO on every pass, before anything else. A Serial
    // print here will stall this and freeze the attitude.
    imu.sample();

    const uint32_t now = micros();
    if (now - lastTickUs < cfg::loop::DT_US) return;
    lastTickUs = now;

    handleSerialCommands();

    sensors.sampleAll();
    if (now - lastFlowUs >= cfg::loop::DT_US * 2) {
        lastFlowUs = now;
        sensors.sampleFlow();
    }
    estimator.update(sensors.getFrame());

    const ImuData& att = imu.getData();
    const StateEstimate& state = estimator.getState();
    BLA::Matrix<4, 1> u = attitude.update(att.roll_rad, att.pitch_rad, att.yaw_rad,
                                          att.gx_rps, att.gy_rps, att.gz_rps,
                                          state.z_m, state.vz_mps);
    const float uArr[4] = { u(0), u(1), u(2), u(3) };
    (void)allocator.allocate(uArr);

    if (serialMode == SerialMode::Inspect && (now - lastPrintUs) >= 100000) {
        lastPrintUs = now;
        printInspectLine(now);
    }
}
