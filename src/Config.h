// AUTO-GENERATED — do not edit by hand.
// Source: config.yaml  |  Script: scripts/gen_config.py
#pragma once

#include <math.h>

namespace cfg {

    // --- Teensy Pin Assignments ---
    constexpr int RW_PIN = 37;
    constexpr int EDF_PIN = 36;
    constexpr int Y_SERVO_PIN = 33;
    constexpr int X_SERVO_PIN = 2;

    // --- Physical Constraints ---
    constexpr double MAX_VEHICLE_ANGLE_DEGREES = 35.0;
    constexpr double MAX_TVC_ANGLE_DEGREES = 8.0;
    constexpr double MAX_EDF_THRUST_NEWTONS = 60.0;

    // --- Vehicle Specs ---
    constexpr double COM_TO_TVC_METERS = 0.3302;
    constexpr double LENGTH_EDF_METERS = 0.05;
    constexpr double LENGTH_RW_METERS = 0.12;
    constexpr double MASS_EDF_KILOGRAMS = 0.7;
    constexpr double MASS_VEHICLE_KILOGRAMS = 4123.7;
    constexpr double MAX_TVC_DEFLECTION_DEGREES = 8.0;
    constexpr double MAX_YAW_TORQUE_NEWTONS = 3.61;
    constexpr double MIN_THRUST_NEWTONS = 20.0;
    constexpr double MAX_THRUST_NEWTONS = 59.77861;
    constexpr double G_METERS_PER_SECOND_SQUARED = 9.807;

    // --- Mass Moment Of Inertia ---
    constexpr double V_JXX = 0.0058595;
    constexpr double V_JYY = 0.0058595;
    constexpr double V_JZZ = 0.01202768;
    constexpr double EDF_JZZ = 0.0001744;
    constexpr double RW_JZZ = 0.00174245;

    // --- Timing ---
    constexpr int DT_MICROSECONDS = 5000;
    constexpr double DT_MILLISECONDS = 5.0;
    constexpr double DT_SECONDS = 0.005;

    // --- Optical Flow ---
    constexpr double PMW3901_FOV_DEGREES = 42.0;
    constexpr double PMW3901_FOCAL_PIXELS = 412.27;
    constexpr int PMW3901_WIDTH_PIXELS = 30;

    // --- Imu Offsets ---
    constexpr double FSM_PITCH_OFFSET_DEGREES = 0.0;
    constexpr double FSM_ROLL_OFFSET_DEGREES = 0.0;
    constexpr double FSM_YAW_OFFSET_DEGREES = 0.0;

    // --- Edf Pwm ---
    constexpr int EDF_OFF_MICROSECONDS = 900;
    constexpr int EDF_MIN_MICROSECONDS = 1250;
    constexpr int EDF_MAX_MICROSECONDS = 2000;
    constexpr int EDF_MAX_SUSTAINED_MICROSECONDS = 1730;
    constexpr int EDF_IDLE_MICROSECONDS = 1560;

    // --- Servo Pwm ---
    constexpr int SERVO_X_CENTER_MICROSECONDS = 1550;
    constexpr int SERVO_Y_CENTER_MICROSECONDS = 1550;
    constexpr int SERVO_X_MIN_MICROSECONDS = 1190;
    constexpr int SERVO_Y_MIN_MICROSECONDS = 1190;
    constexpr int SERVO_X_MAX_MICROSECONDS = 1950;
    constexpr int SERVO_Y_MAX_MICROSECONDS = 1950;

    // --- Tvc Regression X ---
    constexpr double X_P1 = -0.4495171752;
    constexpr double X_P2 = 38.0541192575;
    constexpr double X_P3 = 1559.2072714421;

    // --- Tvc Regression Y ---
    constexpr double Y_P1 = -0.0263024983;
    constexpr double Y_P2 = 25.6989554547;
    constexpr double Y_P3 = 1527.8333080999;

    // --- Edf Regression ---
    constexpr double EDF_P1 = -7.3749e-06;
    constexpr double EDF_P2 = 0.201627519;
    constexpr double EDF_P3 = 1017.2286135313;

    // --- Rw Regression ---
    constexpr double RW_P1 = 0.0008412945;
    constexpr double RW_P2 = 2.5341532938;
    constexpr double RW_P3 = 1050.1703888209;

    // --- Derived constants ---
    constexpr double D2R = M_PI / 180.0;
    constexpr double R2D = 180.0 / M_PI;

}  // namespace cfg
