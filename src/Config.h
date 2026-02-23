///// Sensors.h /////

#ifndef _CONFIG_H
#define _CONFIG_H

#include "Math.h"
//=== Teensy 4.1 Pin Assignments ===//
#define RW_PIN 37
#define EDF_PIN 36
#define YSERVO_PIN 33
#define XSERVO_PIN 2

//=== Physical Constraints ===//
#define MAX_VEHICLE_ANGLE_DEG 35.00f
#define MAX_TVC_ANGLE_DEG 8.00f
#define MAX_EDF_THRUST_NEWTONS 60.00f

//--- Vehicle Specs + General Constants
#define COM_TO_TVC 0.3302                                       //m
#define ledf .050                                               //m
#define lrw 0.12                                               //m
#define MASS_EDF .700                                           //Kg
//#define MASS 3.273                                              //Kg
//#define MASS 2.290                                              //Kg
#define MASS 4123.7                                              //Kg
#define MAX_TVC_DEFLECTION_DEG 8.00f                           //deg
#define MAX_TVC_DEFLECTION_RAD (d2r * MAX_TVC_DEFLECTION_DEG)   //rad
#define MAX_YAW_TORQUE 3.61
#define MIN_THRUST 20.00                                        //Newtons
#define MAX_THRUST 59.77861                                       //Newtons
#define G 9.807                                                  //m/s^2

//--- MASS-MOMENT-OF-INERTIA OF VEHICLE
#define V_JXX 0.0058595f
#define V_JYY 0.0058595f
#define V_JZZ 0.01202768f
#define EDF_JZZ 0.0001744f      //MASS-MOMENT-OF-INERTIA OF EDF-PROP/MOTOR
#define RW_JZZ 0.00174245f      //MASS-MOMENT-OF-INERTIA OF REACTION WHEEL

//--- Step Change + Angles 
#define DT_USEC 5000
#define DT_MSEC 5.00f
#define DT_SEC (0.0050)
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)

//--- Optical Flow Pinhole Definition
#define PMW3901_FOV 42.0f           // Degress
#define PMW3901_FOCAL 412.27f       // Focal length in pixels (found experimentally using pin-hole model)
#define PMW3901_WIDTH 30            // Pixels

//--- IMU Body Mount Offsets
#define FSM_PITCH_OFFSET_RAD (d2r * 0.0f)
#define FSM_ROLL_OFFSET_RAD (d2r * 0.0f)
#define FSM_YAW_OFFSET_RAD (d2r * 0.0f)
// #define FSM_ROLL_OFFSET_RAD (d2r * -1.28331)
// #define FSM_PITCH_OFFSET_RAD (d2r * -2.18253)

//--- EDF Definit End Points
#define EDF_OFF_PWM 900                 //uSec
#define EDF_MIN_PWM 1250                //uSec
#define EDF_MAX_PWM 2000                //uSec
#define EDF_MAX_SUSTAINED_PWM 1730      //uSec
#define EDF_IDLE_PWM 1560               //uSec

//--- Servo Definition End Points
#define SERVO_X_CENTER_US 1550          //µs
#define SERVO_Y_CENTER_US 1550          //µs
#define SERVO_X_MIN_US 1190             //µs
#define SERVO_Y_MIN_US 1190             //µs
#define SERVO_X_MAX_US 1950             //µs
#define SERVO_Y_MAX_US 1950             //µs

//--- MKS Servo TVC Regression Coeffs
#define X_P1 -0.4495171752
#define X_P2 38.0541192575
#define X_P3 1559.2072714421

#define Y_P1 -0.0263024983
#define Y_P2 25.6989554547
#define Y_P3 1527.8333080999

//--- EDF Regression Polynomial Coeffs
#define EDF_P1 -0.0000073749
#define EDF_P2 0.2016275190
#define EDF_P3 1017.2286135313

//--- Antiroll motor regression Coeffs (Quadratic)
#define RW_P1_GRAMS 0.0008412945
#define RW_P2_GRAMS 2.5341532938
#define RW_P3_GRAMS 1050.1703888209
// #define RW_MAX_SPEED_RPS 165.00f 
// #define RW_MAX_SPEED_DPS (RW_MAX_SPEED_DPS * 180.00f/PI)

#endif

