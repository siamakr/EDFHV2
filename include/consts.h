#pragma once
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// consts.h -- every tunable and physical constant for the vehicle.
//
// constexpr, not #define. Reasons:
//   - typed, so a unit mixup can actually fail to compile
//   - scoped, so cfg::edf::P0 and cfg::gimbal::P0 can coexist
//   - visible to the debugger; #define is gone by the time you're stepping
//   - no macro capture of names like G, MASS, or PI
//
// Namespaced so the call site reads as documentation: cfg::vehicle::MASS_KG
// tells you the unit without a comment.
//
// RULE: a physical quantity is declared exactly ONCE in this file. In V0.1 the
// max TVC deflection existed twice with two different values (10 deg in
// Controller.h, 8 deg in Actuator.h) and d2r/r2d existed three times. Where a
// second namespace needs the same number it ALIASES the original rather than
// restating the literal, so the two can never drift apart.
//
// UNITS: all forces are NEWTONS. The EDF and RCS regressions are fit directly
// against the newtons column of the load-cell data in ./Mappings, not
// converted from V0.1's gram-force fits.
// ─────────────────────────────────────────────────────────────────────────────

namespace cfg {

// ── Units ───────────────────────────────────────────────────────────────────
namespace unit {
    constexpr float PI_F  = 3.14159265358979f;
    constexpr float D2R   = PI_F / 180.0f;
    constexpr float R2D   = 180.0f / PI_F;

}

// ── Timing ──────────────────────────────────────────────────────────────────
namespace loop {
    constexpr float    CONTROL_HZ = 200.0f;
    constexpr float    DT_S       = 1.0f / CONTROL_HZ;      // 0.005
    constexpr float    DT_MS      = 1000.0f / CONTROL_HZ;   // 5.0
    constexpr uint32_t DT_US      = (uint32_t)(1e6f / CONTROL_HZ);  // 5000

    // main() schedules on DT_US. If the loop falls behind badly enough that
    // the measured interval exceeds this, the step is rejected rather than
    // integrated -- V0.1 assumed DT was always exactly right and never checked.
    // The estimator's A/B stay fixed at DT_S; this is a guard, not a variable
    // timestep.
    constexpr float    DT_MAX_S  = DT_S * 3.0f;
    constexpr uint32_t DT_MAX_US = DT_US * 3;
}

// ── Pins (Teensy 4.1) ───────────────────────────────────────────────────────
namespace pin {
    constexpr uint8_t GIMBAL_X = 2;
    constexpr uint8_t GIMBAL_Y = 33;
    constexpr uint8_t EDF      = 36;
    constexpr uint8_t RCS      = 37;   // shared line, both BLDCs (V0.1's RW pin)

    // Sensors (SPI). Matches the flight-proven wiring in EDFH_V0.1 and the
    // most recent Teststand_TEENSY build -- main's Config.h had WAK and RST
    // swapped.
    constexpr uint8_t IMU_CS   = 10;
    constexpr uint8_t IMU_WAK  = 7;
    constexpr uint8_t IMU_INT  = 8;
    constexpr uint8_t IMU_RST  = 9;

    // PMW3901 on SPI1
    constexpr uint8_t FLOW_CS   = 29;
    constexpr uint8_t FLOW_MOSI = 26;
    constexpr uint8_t FLOW_MISO = 1;
    constexpr uint8_t FLOW_SCK  = 27;
}

// ── Vehicle ─────────────────────────────────────────────────────────────────
namespace vehicle {
    constexpr float MASS_KG      = 4.1237f;       // Config.h stored 4123.7 GRAMS
    constexpr float G_MS2        = 9.807f;
    constexpr float WEIGHT_N     = MASS_KG * G_MS2;   // 40.441 N hover thrust

    constexpr float COM_TO_TVC_M = 0.3302f;
    constexpr float COM_TO_RCS_M = 0.12f;         // V0.1 LENGTH_RW_M; confirm
                                                  // it still holds for the
                                                  // BLDC pair
    constexpr float COM_TO_EDF_M = 0.050f;

    // Mass moment of inertia
    constexpr float JXX = 0.0058595f;       // From V0.1, not yet measured
    constexpr float JYY = 0.0058595f;       // From V0.1, not yet measured  
    constexpr float JZZ = 0.01202768f;      // From V0.1, not yet measured

    // EDF rotor spin inertia, for the yaw reaction torque it dumps into the
    // airframe. RW_JZZ is gone with the reaction wheel.
    constexpr float EDF_JZZ = 0.0001744f;
}

// ── EDF ─────────────────────────────────────────────────────────────────────
// Regression: PWM_us = P0 + P1*F + P2*F^2, F in NEWTONS.
//
// Fit against Mappings/Hacker-EDF-PWM-to-Thrust-Mapping/hacker_5100AH_reg_003
// over PWM 1540-1950 -- from just above the notch to the onset of saturation,
// which is the widest range the curve is single-valued over.
// rms 4.3 us, max 9.9 us, R2 = 0.9987.
//
// LOWER BOUND is set by a repeatable thrust NOTCH at 1450-1530 us: thrust
// climbs to 24.1 N at 1450, falls to 21.5 N at 1500, then jumps to 25.8 N at
// 1550. It is present in all four 5100AH runs, so it is the ESC/motor, not
// measurement noise. Thrust is not invertible across it -- ~21.5-24.5 N is
// produced by three different PWM values -- so no fit can extend through it.
// 1540 us is the first clean sample above it.
//
// UPPER BOUND is saturation: past 1950 us thrust is flat at ~59.6-60 N, so
// PWM stops being recoverable from thrust.
//
// On the error: the worst residual is at 1810 us and appears in every
// candidate fit range, because the data itself is non-monotonic there
// (46.79 N at 1800, 47.88 at 1810, 47.71 at 1820). It is scatter in the
// measurement, not a modelling failure. Restricting the fit to the hover band
// only improves rms near hover from 4.0 to 2.9 us -- about 0.09 N -- which is
// not worth surrendering 12 N of descent authority.
namespace edf {
    constexpr float P0 =  1201.4072055f;
    constexpr float P1 =  14.5853222f;
    constexpr float P2 = -0.0348208f;

    // Regression validity window. Data covers 25.107 N (1540 us) to
    // 59.594 N (1950 us).
    constexpr float MIN_N = 25.2f;
    constexpr float MAX_N = 59.594f;

    // Zero thrust is NOT reachable through the regression -- clamping would
    // take 0 up to MIN_N and emit ~1547 us, i.e. 25 N when you asked for none.
    // Actuator::drive() intercepts commands at or below CUTOFF_N and writes
    // OFF_US directly instead. Pass CUTOFF_N and OFF_US to the constructor.
    constexpr float NEUTRAL_N = 0.0f;

    // Shutoff threshold. Deliberately near zero rather than at MIN_N: a
    // command of, say, 24 N during a descent must SATURATE to MIN_N, not kill
    // the motor. Only an explicit ~zero command means "stop". Nothing in
    // flight commands between CUTOFF_N and MIN_N, so the discontinuity there
    // is unreachable except via neutral()/disarm().
    constexpr float CUTOFF_N = 1.0f;

    // Raw PWM landmarks, for arming/shutdown paths that bypass the regression
    constexpr uint16_t OFF_US           = 900;
    constexpr uint16_t MIN_US           = 1250;
    constexpr uint16_t IDLE_US          = 1560;
    constexpr uint16_t MAX_SUSTAINED_US = 1730;   // thermal limit
    constexpr uint16_t MAX_US           = 2000;   // burst only

    // Hard PWM floor for any in-flight command, applied AFTER the regression
    // as a second guard on top of MIN_N. Must sit above the notch, whose last
    // bad sample is 1530 us (23.13 N, lower than 1520's 23.28 N). 1547 us is
    // where MIN_N lands on the fit, so in normal operation the two agree.
    constexpr uint16_t FLIGHT_MIN_US = 1547;

    // Spin-up before handing over to the controller: the fan is brought
    // straight to the flight floor so it never dwells in the notch.
    // ~25.6 N, about 63% of hover -- the vehicle stays on the pad.
    constexpr uint16_t PRIME_US = FLIGHT_MIN_US;
    constexpr uint32_t PRIME_MS = 2000;
}

// ── Gimbal ──────────────────────────────────────────────────────────────────
// Regression: PWM_us = P0 + P1*theta + P2*theta^2, theta in DEGREES.
//
// P0 is the true mechanical neutral for each axis (1559.2 / 1527.8) and is NOT
// the same as V0.1's hardcoded 1550 us centers -- trust the regression.
namespace gimbal {
    // Mechanical travel limit. Declared once here; the controller clamps to
    // the same number by aliasing it in namespace ctrl.
    constexpr float TRAVEL_DEG = 8.0f;
    constexpr float TRAVEL_RAD = TRAVEL_DEG * unit::D2R;

    // roll servo
    namespace x {
        constexpr float P0 =  1564.2072714f;
        constexpr float P1 =  38.0541193f;
        constexpr float P2 = -0.4495172f;

        constexpr float MIN_DEG     = -TRAVEL_DEG;
        constexpr float MAX_DEG     =  TRAVEL_DEG;
        constexpr float NEUTRAL_DEG =  0.0f;
    }
    // pitch servo
    namespace y {
        constexpr float P0 =  1568.8333081f;
        constexpr float P1 =  25.6989555f;
        constexpr float P2 = -0.0263025f;

        constexpr float MIN_DEG     = -TRAVEL_DEG;
        constexpr float MAX_DEG     =  TRAVEL_DEG;
        constexpr float NEUTRAL_DEG =  0.0f;
    }

    // Raw servo endpoints, as a mechanical sanity bound. The regression at
    // +/-TRAVEL_DEG lands inside these on both axes.
    constexpr uint16_t MIN_US = 1190;
    constexpr uint16_t MAX_US = 1950;

    // Bench servo dance ('d'): one full circle at each amplitude.
    constexpr float    DANCE_AMP_DEG[]     = { 8.0f, 7.0f, 5.0f, 4.0f, 2.0f };
    constexpr uint8_t  DANCE_N_AMPS        = 5;
    constexpr float    DANCE_PERIOD_S      = 2.0f;
}

// ── RCS ─────────────────────────────────────────────────────────────────────
// Regression: PWM_us = P0 + P1*F + P2*F^2, F = SINGLE-motor force in NEWTONS.
// drive() takes vehicle torque in N*m; the class converts.
//
// Fit against Mappings/Surpass-RCS-PWM-to-Thrust-Mappings/Surpass-RCS-PWM-Reg-002
// over PWM 1100-1800. rms 3.7 us, max 9.4 us.
//
// The curve is clean and strictly monotonic -- no notch, unlike the EDF. The
// upper end is excluded because thrust plateaus around 1830-1860 before
// stepping again; including it inflates rms to 25 us and distorts the fit
// across the whole low-force region where yaw actually operates.
namespace rcs {
    constexpr float P0 = 1101.0690f;
    constexpr float P1 = 151.2324f;
    constexpr float P2 = 48.1642f;

    constexpr float MIN_N      = 0.0f;    // 1101 us, inside the deadband --
                                          // unlike the EDF, F=0 is a real
                                          // operating point here
    constexpr float MAX_N      = 2.55f;   // per motor, top of the fitted band
                                          // (1800 us). The motor reaches
                                          // 3.52 N at 2000 us if you refit.
    constexpr float NEUTRAL_NM = 0.0f;    // note: N*m, drive()'s input unit

    // Torque authority of the pair, derived from force limit and moment arm.
    constexpr float MAX_TORQUE_NM = 2.0f * MAX_N * vehicle::COM_TO_RCS_M;

    // A command at or below CUTOFF_NM writes OFF_US instead of going through
    // the regression, so neutral()/init/disarm hold the ESCs below 1000 us
    // (same as Teststand). Zero torque in flight lands here too; 1101 us and
    // 900 us are both no thrust.
    constexpr float    CUTOFF_NM = 0.0f;
    constexpr uint16_t OFF_US    = 900;

    constexpr uint16_t MIN_US = 1100;
    constexpr uint16_t MAX_US = 1800;
}

// ── IMU (BNO080) ────────────────────────────────────────────────────────────
namespace imu {
    // Body mount offsets, ADDED to the raw attitude solution (as V0.1). These
    // were loose consts in Sensors.h in V0.1.
    constexpr float ROLL_OFFSET_DEG  = 1.3772f;
    constexpr float PITCH_OFFSET_DEG = 0.6578f;
    constexpr float YAW_OFFSET_DEG   = 0.0f;

    constexpr float ROLL_OFFSET_RAD  = ROLL_OFFSET_DEG  * unit::D2R;
    constexpr float PITCH_OFFSET_RAD = PITCH_OFFSET_DEG * unit::D2R;
    constexpr float YAW_OFFSET_RAD   = YAW_OFFSET_DEG   * unit::D2R;

    // Calibration: the BNO reports 0-3 accuracy per axis. Hold all three at 3
    // for this long while rotating the box pattern and the Imu class saves
    // automatically -- no keypress, no blocking wait in the sample path.
    constexpr uint8_t  ACCURACY_TARGET     = 3;
    constexpr uint32_t CAL_DWELL_MS        = 2000;
    constexpr uint32_t CAL_SAVE_TIMEOUT_MS = 200;

    // First-order IIR on the raw gyro and accelerometer:
    //     out = (1-alpha)*new + alpha*prev
    constexpr float ALPHA_GYRO  = 0.10f;
    constexpr float ALPHA_ACCEL = 0.050f;

    // Report cadence requested from the BNO, and SPI clock.
    constexpr uint16_t REPORT_INTERVAL_MS = (uint16_t)loop::DT_MS;
    constexpr uint32_t SPI_HZ             = 3000000;  // BNO080 max; Teststand uses this

    // Attitude is the one measurement the vehicle cannot fly without, so the
    // staleness window is tight: 10 missed cycles.
    constexpr uint32_t STALE_LIMIT_US = loop::DT_US * 10;
}

// ── Rangefinder (LIDAR-Lite v3HP) ───────────────────────────────────────────
namespace lidar {
    constexpr uint8_t I2C_ADDRESS           = 0x62;
    constexpr uint8_t SHORT_RANGE_FAST_MODE = 2;
    constexpr uint8_t NORMAL_MODE           = 0;
    constexpr uint8_t CONFIGURE_MODE        = NORMAL_MODE;

    // Height of the sensor above the landing feet, subtracted from every
    // reading so z is measured from the ground contact point.
    constexpr float MOUNT_OFFSET_M = 0.08f;

    // A raw reading of 0 cm is the v3HP's dropout code and is always rejected.
    // Otherwise the floor is the mount offset itself: sitting on the pad, z
    // is ~0 and can read slightly negative, and those readings must reach the
    // estimator so it is seeded before liftoff.
    constexpr float MIN_VALID_M = -MOUNT_OFFSET_M;
    constexpr float MAX_VALID_M = 20.0f;

    // Acquisition is slower than the control loop and readings are dropped on
    // rejection, so this is looser than the IMU's.
    constexpr uint32_t STALE_LIMIT_US = loop::DT_US * 40;
}

// ── Optical flow (PMW3901) ──────────────────────────────────────────────────
namespace flow {
    constexpr float FOV_DEG      = 42.0f;
    constexpr int   WIDTH_PIXELS = 30;

    // PixArt / PX4 conversion: raw counts → radians. Not the geometric
    // pinhole focal length (that comes out ~39 px). V0.1 used 412.27 from a
    // bench fit; PX4's PMW3901 driver uses 385. Difference is ~7%.
    constexpr float FOCAL_PIXELS = 385.0f;

    // Datasheet working range. Below this the chip still returns counts but
    // they do not scale with height. Lidar AGL can also go negative on the
    // pad (mount offset), which inverted the old v = flow * zWorld scale.
    constexpr float MIN_RANGE_M = 0.08f;
    constexpr float MAX_RANGE_M = 30.0f;

    // SQUAL floor. PX4 drops a sample when quality is 0.
    constexpr uint8_t MIN_QUALITY = 1;

    constexpr float MIN_DT_S = 0.001f;
    constexpr float MAX_DT_S = 0.100f;

    constexpr uint32_t STALE_LIMIT_US = loop::DT_US * 40;
}

// ── Estimator ───────────────────────────────────────────────────────────────
// Steady-state Kalman gain, held constant (no covariance propagation on the
// Teensy). Layout mirrors the 6x6 Kf matrix in V0.1's Sensors.h; only the
// non-zero entries are named here and Estimator builds the matrix from them.
//
// State: [x, y, z, vx, vy, vz]
namespace est {
    constexpr float K_POS      = 0.1000f;   // horizontal position <- pos meas
    constexpr float K_POS_VEL  = 0.0008f;   // horizontal cross term
    constexpr float K_Z        = 0.0768f;   // altitude <- lidar
    constexpr float K_Z_VZ     = 0.0850f;   // altitude <- vz
    constexpr float K_VEL      = 0.7000f;   // horizontal velocity <- flow
    constexpr float K_VZ_Z     = 1.0000f;   // vz <- lidar
    constexpr float K_VZ       = 0.0000f;   // vz <- vz  (unused, kept explicit)
}

// ── Waypoints ───────────────────────────────────────────────────────────────
namespace wp {
    // Fractions of the setpoint. 0.30 = ±30%. A zero axis uses altitude
    // as the 100% reference (LAND uses the last hover altitude).
    constexpr float WP_TOL_XY   = 0.30f;
    constexpr float WP_TOL_Z    = 0.15f;
    constexpr float LAND_TOL_XY = 0.33f;
    constexpr float LAND_TOL_Z  = 0.010f;
}

// ── Controller ──────────────────────────────────────────────────────────────
namespace ctrl {
    // LQR state order: [roll, pitch, yaw, gx, gy, gz, z, vz]
    // Control order:   [gimbal_x_rad, gimbal_y_rad, yaw_torque_Nm, thrust_N]
    constexpr int NX = 8;
    constexpr int NU = 4;

    constexpr float K_ROLL  = 0.3700f;
    constexpr float K_PITCH = K_ROLL;
    // constexpr float K_YAW   = 0.0316f;
    constexpr float K_YAW   = -0.3516f;
    constexpr float K_GX    = 0.1240f;
    constexpr float K_GY    = K_GX;
    constexpr float K_GZ    = -0.1561f;
    constexpr float K_Z     = 3.1000f;
    constexpr float K_VZ    = 8.6942f;

    // Output limits. Aliases, so the controller and the actuator physically
    // cannot clamp to different numbers.
    constexpr float MAX_TVC_DEG = gimbal::TRAVEL_DEG;
    constexpr float MAX_TVC_RAD = gimbal::TRAVEL_RAD;

    // Command floor is the regression floor, NOT a tuning choice: below
    // edf::MIN_N the thrust curve is unmapped and, further down, ambiguous
    // (see the notch note). V0.1 used 20 N, which sits inside the notch.
    constexpr float MIN_THRUST_N = edf::MIN_N;
    constexpr float MAX_THRUST_N = edf::MAX_N;

    constexpr float MAX_YAW_TORQUE_NM = rcs::MAX_TORQUE_NM;

    // Servo command IIR: out = (1-a)*new + a*prev
    constexpr float SERVO_ALPHA = 0.10f;

    // Abort if attitude exceeds this
    constexpr float MAX_TILT_DEG = 35.0f;
    constexpr float MAX_TILT_RAD = MAX_TILT_DEG * unit::D2R;

    // Outer position LQR (V0.1 K_pos). State [x, y, vx, vy, xint, yint].
    // Output is attitude setpoints in radians: roll from y, pitch from x.
    namespace pos {
        constexpr float K_Y    = -0.100031f;
        constexpr float K_VY   = -0.1400f;
        constexpr float K_YINT = -0.05000f;
        constexpr float K_X    =  0.100031f;
        constexpr float K_VX   =  0.1400f;
        constexpr float K_XINT =  0.05000f;
        constexpr float INT_LIM_M     = 0.35f;
        constexpr float MAX_ATT_DEG   = 10.0f;
        constexpr float MAX_ATT_RAD   = MAX_ATT_DEG * unit::D2R;
    }
}

}  // namespace cfg
