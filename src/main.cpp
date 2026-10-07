#include <Arduino.h>
#include <math.h>
#include <string.h>

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
#include "PositionController.h"
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
static PositionController position;
static ControlAllocator   allocator;

// ── Loop scheduling ─────────────────────────────────────────────────────────
static uint32_t lastTickUs = 0;
static uint32_t lastFlowUs = 0;

// ── Serial modes ────────────────────────────────────────────────────────────
// main   : quiet bring-up (default).  m returns here.
// inspect: IMU + lidar + estimator stream.  k enters here.
// servo  : manual gimbal PWM.  c enters here.  z/x roll, t/y pitch, 5 us.
// dance  : one circle at 7, 4, then 2 deg.  d starts it.
// edf    : manual EDF thrust.  e enters here.  0 = off, 1-6 = 10-60 N.
// waypoint: runs the mission table.  w enters here.
enum class SerialMode : uint8_t { Main, Inspect, Servo, Dance, Edf, Waypoint };
static SerialMode serialMode   = SerialMode::Main;
static uint32_t   lastPrintUs  = 0;
static uint16_t   servoRollUs  = 0;
static uint16_t   servoPitchUs = 0;
static uint8_t    danceAmpIdx  = 0;
static float      danceTheta   = 0.0f;
static float      edfCmd_N     = 0.0f;

// ── Waypoints ───────────────────────────────────────────────────────────────
struct Waypoint {
    uint8_t number  = 0;
    char    name[16] = {};
    float   alt_m   = 0.0f;
    float   x_m     = 0.0f;
    float   y_m     = 0.0f;
    float   dwell_s = 0.0f;
    bool    isLand  = false;
};

static const struct {
    const char* name;
    float alt_m, x_m, y_m, dwell_s;
} mission[] = {
    // name, alt_m, x_m, y_m, dwell_s
    { "HOVER 1", 0.30f, 0.00f, 0.00f, 2.5f },
    // { "HOVER 2", 0.50f, 1.00f, 0.00f, 5.0f },
    // { "HOVER 2", 0.50f, 0.00f, 0.00f, 5.0f },
    // { "HOVER 3", 0.50f, 1.50f, 0.00f, 5.0f },
    // { "HOVER 4", 0.50f, 0.00f, 0.00f, 5.0f },
    { "LAND",  0.00f, 0.00f, 0.00f, 0.0f },
};
static constexpr uint8_t missionLen = sizeof(mission) / sizeof(mission[0]);

static Waypoint wp;
static bool     wpActive   = false;
static bool     wpComplete = false;
static bool     wpLanded            = false;
static bool     landingConfirmed    = false;
static uint32_t wpInBandUs  = 0;
static uint8_t  wpIndex     = 0;
static float    wpPrevAlt_m = 0.0f;

static bool inTolPct(float meas, float target, float frac, float zeroScale)
{
    const float scale = (fabsf(target) > 1e-4f) ? fabsf(target) : zeroScale;
    return fabsf(meas - target) <= frac * scale;
}

static void waypoint(uint8_t number, const char* name,
                     float alt_m, float x_m, float y_m, float dwell_s)
{
    wp = Waypoint{};
    wp.number  = number;
    strncpy(wp.name, name ? name : "", sizeof(wp.name) - 1);
    wp.isLand  = (strcmp(wp.name, "LAND") == 0);
    wp.alt_m   = wp.isLand ? 0.0f : alt_m;
    wp.x_m     = wp.isLand ? 0.0f : x_m;
    wp.y_m     = wp.isLand ? 0.0f : y_m;
    wp.dwell_s = dwell_s;

    wpActive   = true;
    wpComplete = false;
    wpLanded         = false;
    landingConfirmed = false;
    wpInBandUs = 0;
    if (!wp.isLand) wpPrevAlt_m = wp.alt_m;

    position.setReference(wp.x_m, wp.y_m);
    position.reset();
    attitude.setZRef(wp.alt_m);
}

static void serviceWaypoint(const StateEstimate& est)
{
    if (!wpActive || wpComplete) return;

    const float scale = wp.isLand ? wpPrevAlt_m : fabsf(wp.alt_m);
    const float xyF   = wp.isLand ? cfg::wp::LAND_TOL_XY : cfg::wp::WP_TOL_XY;
    const float zF    = wp.isLand ? cfg::wp::LAND_TOL_Z  : cfg::wp::WP_TOL_Z;
    const float zTol  = zF * ((scale > 1e-4f) ? scale : 1.0f);
    const bool xyIn   = inTolPct(est.x_m, wp.x_m, xyF, scale)
                     && inTolPct(est.y_m, wp.y_m, xyF, scale);
    // On the pad z is often slightly negative. Treat "at or below" the
    // land height as down -- do not require a ± band that misses -1 cm.
    const bool zIn    = wp.isLand
                        ? (est.z_m <= wp.alt_m + zTol)
                        : inTolPct(est.z_m, wp.alt_m, zF, scale);
    const bool inBand = xyIn && zIn;

    attitude.setZRef(wp.alt_m);

    if (wp.isLand && inBand) {
        landingConfirmed = true;
        wpLanded         = true;
        wpComplete       = true;
        actuators.disarm();
        actuators.safeAll();
        edf.writeRaw(cfg::edf::OFF_US);
        return;
    }

    if (!inBand) {
        wpInBandUs = 0;
        return;
    }

    wpInBandUs += cfg::loop::DT_US;

    if ((wpInBandUs / 1000000.0f) >= wp.dwell_s) {
        wpComplete = true;
        if (wpIndex + 1 < missionLen) {
            ++wpIndex;
            waypoint(wpIndex + 1, mission[wpIndex].name,
                     mission[wpIndex].alt_m, mission[wpIndex].x_m,
                     mission[wpIndex].y_m, mission[wpIndex].dwell_s);
        }
    }
}

static void startWaypointMission()
{
    imu.setYawOrigin();
    attitude.setYawRef(0.0f);
    wpIndex            = 0;
    wpLanded           = false;
    landingConfirmed   = false;
    position.reset();
    actuators.arm();
    waypoint(1, mission[0].name, mission[0].alt_m,
             mission[0].x_m, mission[0].y_m, mission[0].dwell_s);
}

static void abortWaypointMission()
{
    wpActive   = false;
    wpComplete = true;
    actuators.disarm();
    actuators.safeAll();
    edf.writeRaw(cfg::edf::OFF_US);
}

static void parkGimbals()
{
    gimbalX.neutral();
    gimbalY.neutral();
}

static void stepDance()
{
    const float amp = cfg::gimbal::DANCE_AMP_DEG[danceAmpIdx];
    const float w   = 2.0f * cfg::unit::PI_F / cfg::gimbal::DANCE_PERIOD_S;
    danceTheta += w * cfg::loop::DT_S;

    if (danceTheta >= 2.0f * cfg::unit::PI_F) {
        danceTheta = 0.0f;
        ++danceAmpIdx;
        if (danceAmpIdx >= cfg::gimbal::DANCE_N_AMPS) {
            parkGimbals();
            serialMode = SerialMode::Main;
            Serial.println("dance done");
            return;
        }
    }

    gimbalX.drive(amp * cosf(danceTheta));
    gimbalY.drive(amp * sinf(danceTheta));
}

static void nudgeServo(uint16_t& us, Actuator& servo, int delta)
{
    int next = (int)us + delta;
    if (next < (int)cfg::gimbal::MIN_US) next = (int)cfg::gimbal::MIN_US;
    if (next > (int)cfg::gimbal::MAX_US) next = (int)cfg::gimbal::MAX_US;
    us = (uint16_t)next;
    servo.writeRaw(us);
}

static void printServoLine()
{
    char line[96];
    snprintf(line, sizeof(line),
        "roll  %+6.2f deg  %4u us    pitch  %+6.2f deg  %4u us\n",
        gimbalX.getCurrentValue(), gimbalX.getLastPwmUs(),
        gimbalY.getCurrentValue(), gimbalY.getLastPwmUs());
    Serial.print(line);
}

static uint16_t edfThrustToUs(float n)
{
    if (n <= cfg::edf::CUTOFF_N) return cfg::edf::OFF_US;
    long us = lroundf(cfg::edf::P0 + cfg::edf::P1 * n + cfg::edf::P2 * n * n);
    if (us < (long)cfg::edf::OFF_US) us = cfg::edf::OFF_US;
    if (us > (long)cfg::edf::MAX_US) us = cfg::edf::MAX_US;
    return (uint16_t)us;
}

static void setEdfTestThrust(float n)
{
    edfCmd_N = n;
    edf.writeRaw(edfThrustToUs(n));
}

static void printEdfLine()
{
    char line[80];
    snprintf(line, sizeof(line),
        "edf  cmd=%4.0f N  %4u us\n",
        edfCmd_N, edf.getLastPwmUs());
    Serial.print(line);
}

static void printWaypointLine()
{
    const StateEstimate& est = estimator.getState();
    const ImuData&       att = imu.getData();
    const auto&          u   = attitude.getLastOutput();
    using cfg::unit::R2D;

    const float gimbX = isfinite(u(0)) ? (float)u(0) : gimbalX.getCurrentValue();
    const float gimbY = isfinite(u(1)) ? (float)u(1) : gimbalY.getCurrentValue();

    char line[240];
    snprintf(line, sizeof(line),
        "wp %u %s  tgt x=%5.2f y=%5.2f z=%5.2f m  "
        "est x=%6.3f y=%6.3f z=%6.3f m  t=%4.1f/%4.1f%s  "
        "r=%6.2f p=%6.2f y=%6.2f  gimb x=%+5.2f y=%+5.2f  "
        "yawT=%5.3f Nm  F=%5.1f N  edf=%4u us%s\n",
        (unsigned)wp.number, wp.name,
        wp.x_m, wp.y_m, wp.alt_m,
        est.x_m, est.y_m, est.z_m,
        wpInBandUs / 1000000.0f, wp.dwell_s,
        landingConfirmed ? "  LANDING_CONFIRMED" : (wpLanded ? "  LANDED" : (wpComplete ? "  done" : "")),
        att.roll_rad * R2D, att.pitch_rad * R2D, att.yawRel_rad * R2D,
        gimbX, gimbY, (float)u(2),
        landingConfirmed ? 0.0f : (float)u(3),
        edf.getLastPwmUs(),
        landingConfirmed ? "  CUT" : "");
    Serial.print(line);
}

static void printInspectLine(uint32_t nowUs)
{
    const ImuData&        att = imu.getData();
    const LidarData&      rng = lidar.getData();
    const FlowData&       fl  = flow.getData();
    const StateEstimate&  est = estimator.getState();
    const EstimatorDebug& dbg = estimator.getDebug();
    using cfg::unit::R2D;

    char line[360];
    snprintf(line, sizeof(line),
        "imu  r=%6.2f p=%6.2f y=%6.2f  gx=%6.3f gy=%6.3f gz=%6.3f  "
        "ax=%6.3f ay=%6.3f az=%6.3f  acc=%u/%u/%u\n"
        "lid  z=%6.3f raw=%4u\n"
        "flo  dx=%4d dy=%4d  q=%3u  vx=%6.3f vy=%6.3f  v=%6.3f %6.3f  rng=%5.3f\n"
        "est  x=%6.3f y=%6.3f z=%6.3f  vx=%6.3f vy=%6.3f vz=%6.3f\n",
        att.roll_rad  * R2D, att.pitch_rad * R2D, att.yaw_rad * R2D,
        att.gx_rps, att.gy_rps, att.gz_rps,
        att.ax_mps2, att.ay_mps2, att.az_mps2,
        att.linAccuracy, att.gyroAccuracy, att.quatAccuracy,
        rng.z_m, rng.raw_cm,
        (int)fl.dx_px, (int)fl.dy_px, (unsigned)fl.squal, fl.vx, fl.vy,
        dbg.vxMeas_mps, dbg.vyMeas_mps, dbg.flowRange_m,
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

    const int ch = Serial.read();

    switch (ch) {
        case 'k':
            if (serialMode == SerialMode::Edf) setEdfTestThrust(0.0f);
            serialMode = SerialMode::Inspect;
            Serial.println("mode: inspect  (m = main)");
            break;

        case 'c':
            if (serialMode == SerialMode::Edf) setEdfTestThrust(0.0f);
            serialMode   = SerialMode::Servo;
            servoRollUs  = gimbalX.getLastPwmUs();
            servoPitchUs = gimbalY.getLastPwmUs();
            Serial.println("mode: servo  z/x roll  t/y pitch  (+/- 5 us)  m = main");
            break;

        case 'd':
            if (serialMode == SerialMode::Edf) setEdfTestThrust(0.0f);
            serialMode  = SerialMode::Dance;
            danceAmpIdx = 0;
            danceTheta  = 0.0f;
            Serial.println("mode: dance  7 / 4 / 2 deg  (m = abort)");
            break;

        case 'e':
            if (serialMode == SerialMode::Waypoint) abortWaypointMission();
            if (serialMode == SerialMode::Servo || serialMode == SerialMode::Dance) {
                parkGimbals();
            }
            serialMode = SerialMode::Edf;
            setEdfTestThrust(0.0f);
            Serial.println("mode: edf  0 = off  1-6 = 10-60 N  m = main");
            break;

        case 'w':
            if (serialMode == SerialMode::Edf) setEdfTestThrust(0.0f);
            serialMode = SerialMode::Waypoint;
            startWaypointMission();
            Serial.println("mode: waypoint  (m = abort)");
            break;

        case 'm':
            if (serialMode == SerialMode::Servo || serialMode == SerialMode::Dance) {
                parkGimbals();
            }
            if (serialMode == SerialMode::Waypoint) {
                abortWaypointMission();
            }
            actuators.disarm();
            rcs.neutral();
            edf.writeRaw(cfg::edf::OFF_US);
            serialMode = SerialMode::Main;
            Serial.println("mode: main  (k = inspect  c = servo  d = dance  e = edf  w = waypoint)");
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
            position.reset();
            break;

        default:
            if (serialMode == SerialMode::Servo) {
                switch (ch) {
                    case 'z': nudgeServo(servoRollUs,  gimbalX, -5); break;
                    case 'x': nudgeServo(servoRollUs,  gimbalX, +5); break;
                    case 't': nudgeServo(servoPitchUs, gimbalY, -5); break;
                    case 'y': nudgeServo(servoPitchUs, gimbalY, +5); break;
                    default: break;
                }
            } else if (serialMode == SerialMode::Edf && ch >= '0' && ch <= '6') {
                setEdfTestThrust((float)(ch - '0') * 10.0f);
            }
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
    Serial.println("k = inspect   c = servo   d = dance   e = edf   w = waypoint   m = main");

    estimator.reset();
    actuators.safeAll();
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

    const StateEstimate& state = estimator.getState();
    if (serialMode == SerialMode::Waypoint) serviceWaypoint(state);

    if (serialMode == SerialMode::Waypoint && !landingConfirmed) {
        position.setReference(wp.x_m, wp.y_m);
        const AttitudeSetpoint attSp = position.update(
            state.x_m, state.y_m, state.vx_mps, state.vy_mps);
        attitude.setRollRef(attSp.roll_rad);
        attitude.setPitchRef(attSp.pitch_rad);
        attitude.setZRef(wp.alt_m);
    }

    const ImuData& att = imu.getData();
    const float yawForCtrl = (serialMode == SerialMode::Waypoint)
                             ? att.yawRel_rad : att.yaw_rad;
    BLA::Matrix<4, 1> u = attitude.update(att.roll_rad, att.pitch_rad, yawForCtrl,
                                          att.gx_rps, att.gy_rps, att.gz_rps,
                                          state.z_m, state.vz_mps);
    const float uArr[4] = { u(0), u(1), u(2), u(3) };
    const AllocatorOutput cmd = allocator.allocate(uArr);

    // Servo/dance own the gimbals. After LAND, keep the EDF on the off pulse
    // every tick so the LQR hover feedforward cannot spin it back up.
    if (landingConfirmed) {
        edf.writeRaw(cfg::edf::OFF_US);
        rcs.neutral();
        gimbalX.neutral();
        gimbalY.neutral();
    } else if (serialMode == SerialMode::Waypoint && actuators.isArmed()) {
        actuators.setGimbal(cmd.gimbalY_deg, cmd.gimbalX_deg);
        actuators.setYawTorque(cmd.yawTorque_Nm);
        actuators.setThrust(cmd.thrust_N);
    } else if (serialMode == SerialMode::Edf) {
        rcs.neutral();
        gimbalX.neutral();
        gimbalY.neutral();
        edf.writeRaw(edfThrustToUs(edfCmd_N));
    } else if (serialMode != SerialMode::Servo
               && serialMode != SerialMode::Dance) {
        // Main / inspect: propulsion stays off. Gimbals still follow attitude.
        rcs.neutral();
        edf.writeRaw(cfg::edf::OFF_US);
        actuators.setGimbal(cmd.gimbalY_deg, cmd.gimbalX_deg);
    }

    if (serialMode == SerialMode::Dance) stepDance();

    if ((now - lastPrintUs) >= 100000) {
        lastPrintUs = now;
        if (serialMode == SerialMode::Inspect) printInspectLine(now);
        else if (serialMode == SerialMode::Servo || serialMode == SerialMode::Dance) printServoLine();
        else if (serialMode == SerialMode::Edf) printEdfLine();
        else if (serialMode == SerialMode::Waypoint) printWaypointLine();
    }
}
