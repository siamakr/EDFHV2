#ifndef _CONFIG_H
#define _CONFIG_H

#include <Arduino.h>

namespace cfg {

//=== Teensy 4.1 Pin Assignments ===//
namespace pins {
    constexpr int RW_PIN = 37;
    constexpr int EDF_PIN = 36;
    constexpr int Y_SERVO_PIN = 33;
    constexpr int X_SERVO_PIN = 2;
    constexpr int IMU_CS_PIN = 10;
    constexpr int IMU_WAKE_PIN = 9;
    constexpr int IMU_INT_PIN = 8;
    constexpr int IMU_RST_PIN = 7;
}

//=== Lidar Definitions ===//
namespace lidar {
    constexpr uint8_t DEFAULT_ADDRESS = 0x62;
    constexpr int SHORT_RANGE_FAST_SPEED = 2;
    constexpr int NORMAL_OPERATION = 0;
}

//=== Physical Constraints ===//
namespace vehicle_limits {
    constexpr float MAX__ANGLE_DEG = 35.00f;
    constexpr float MAX_TVC_ANGLE_DEG = 8.00f;
    constexpr float MAX_EDF_THRUST_N = 60.00f;
}

//=== Timing / Angle Conversion ===//
namespace timing {
    constexpr int DT_US = 5000;
    constexpr float DT_MS = 5.00f;
    constexpr float DT_S = 0.0050f;
    constexpr float D2R = PI / 180.00f;
    constexpr float R2D = 180.00f / PI;
}

//=== Vehicle Specs ===//
namespace vehicle {
    constexpr float COM_TO_TVC_M = 0.3302f;
    constexpr float LENGTH_EDF_M = 0.050f;
    constexpr float LENGTH_RW_M = 0.12f;
    constexpr float MASS_EDF_K = 0.700f;
    constexpr float MASS_VEHICLE_K = 4123.7f;
    constexpr float MAX_TVC_DEFLECTION_DEG = 8.00f;
    constexpr float MAX_TVC_DEFLECTION_RAD = timing::D2R * MAX_TVC_DEFLECTION_DEG;
    constexpr float MAX_YAW_TORQUE_N = 3.61f;
    constexpr float MIN_THRUST_N = 20.00f;
    constexpr float MAX_THRUST_N = 59.77861f;
    constexpr float G_MS2 = 9.807f;
}

//=== Mass Moments of Inertia ===//
namespace inertia {
    constexpr float V_JXX = 0.0058595f;
    constexpr float V_JYY = 0.0058595f;
    constexpr float V_JZZ = 0.01202768f;
    constexpr float EDF_JZZ = 0.0001744f;
    constexpr float RW_JZZ = 0.00174245f;
}

//=== Optical Flow ===//
namespace flow {
    constexpr float PMW3901_FOV_DEG = 42.0f;
    constexpr float PMW3901_FOCAL_PIXELS = 412.27f;
    constexpr int PMW3901_WIDTH_PIXELS = 30;
}

//=== IMU Body Mount Offsets ===//
namespace imu {
    constexpr float FSM_PITCH_OFFSET_DEG = 0.0f;
    constexpr float FSM_ROLL_OFFSET_DEG = 0.0f;
    constexpr float FSM_YAW_OFFSET_DEG = 0.0f;
    constexpr float FSM_PITCH_OFFSET_RAD = timing::D2R * FSM_PITCH_OFFSET_DEG;
    constexpr float FSM_ROLL_OFFSET_RAD = timing::D2R * FSM_ROLL_OFFSET_DEG;
    constexpr float FSM_YAW_OFFSET_RAD = timing::D2R * FSM_YAW_OFFSET_DEG;
}

//=== EDF PWM End Points ===//
namespace edf_limits {
    constexpr int OFF_US = 900;
    constexpr int MIN_US = 1250;
    constexpr int MAX_US = 2000;
    constexpr int MAX_SUSTAINED_US = 1730;
    constexpr int IDLE_US = 1560;
}

//=== Servo PWM End Points ===//
namespace servo_limits {
    constexpr int X_CENTER_US = 1550;
    constexpr int Y_CENTER_US = 1550;
    constexpr int X_MIN_US = 1190;
    constexpr int Y_MIN_US = 1190;
    constexpr int X_MAX_US = 1950;
    constexpr int Y_MAX_US = 1950;
}

//=== TVC Regression Coefficients (X) ===//
namespace tvcregr_x {
    constexpr float P0 = -0.4495171752f;
    constexpr float P1 = 38.0541192575f;
    constexpr float P2 = 1559.2072714421f;
}

//=== TVC Regression Coefficients (Y) ===//
namespace tvcregr_y {
    constexpr float P0 = -0.0263024983f;
    constexpr float P1 = 25.6989554547f;
    constexpr float P2 = 1527.8333080999f;
}

//=== EDF Thrust Regression Coefficients ===//
namespace edfregr {
    constexpr float P0 = -0.0000073749f;
    constexpr float P1 = 0.2016275190f;
    constexpr float P2 = 1017.2286135313f;
}

//=== RCS Regression Coefficients ===//
namespace rcsregr {
    constexpr float P0 = 0.0008412945f;
    constexpr float P1 = 2.5341532938f;
    constexpr float P2 = 1050.1703888209f;
}

}  // namespace cfg

#endif
