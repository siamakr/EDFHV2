///// Sensors.h /////

#ifndef _CONSTANTS_H
#define _CONSTANTS_H

//--- Teensy 4.1 Pin Assignments
#define RW_PIN 37
#define EDF_PIN 36
#define YSERVO_PIN 33
#define XSERVO_PIN 14

//--- Physical Constraints
#define MAX_VEHICLE_ANGLE_DEG 35.00f
#define MAX_TVC_ANGLE_DEG 8.00f
#define MAX_EDF_THRUST_NEWTONS 31.00f
//--- Vehicle Specs + General Constants
#define COM_TO_TVC 0.1335                                       //m
#define ledf .050                                               //m
#define lrw 0.12                                               //m
#define MASS_EDF .700                                           //Kg
//#define MASS 3.273                                              //Kg
//#define MASS 2.290                                              //Kg
#define MASS 2.790                                              //Kg
#define MAX_TVC_DEFLECTION_DEG 4.00f                           //deg
#define MAX_TVC_DEFLECTION_RAD (d2r * MAX_TVC_DEFLECTION_DEG)   //rad
#define MAX_YAW_TORQUE 1.61
#define MIN_THRUST 17.00                                        //Newtons
#define MAX_THRUST 45.00                                        //Newtons
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

//--- LIDAR Pinhole Definition
#define PMW3901_FOV 42.0f           // Degress
#define PMW3901_FOCAL 412.27f       // Focal length in pixels (found experimentally using pin-hole model)
#define PMW3901_WIDTH 30            // Pixels

//--- IMU Body Mount Offsets
#define FSM_PITCH_OFFSET_RAD (d2r * 1.3772)
#define FSM_ROLL_OFFSET_RAD (d2r * 0.6578f)
#define FSM_YAW_OFFSET_RAD (d2r * 0.00f)
// #define FSM_ROLL_OFFSET_RAD (d2r * -1.28331)
// #define FSM_PITCH_OFFSET_RAD (d2r * -2.18253)


//--- EDF Definit End Points
#define EDF_OFF_PWM 900                 //uSec
#define EDF_MIN_PWM 1250                //uSec
#define EDF_MAX_PWM 1650                //uSec
#define EDF_MAX_SUSTAINED_PWM 1730      //uSec
#define EDF_IDLE_PWM 1260               //uSec

//--- Servo Definition End Points
#define SERVO_X_CENTER_US 1390          //µs
#define SERVO_Y_CENTER_US 1575          //µs
#define SERVO_X_MIN_US 1026             //µs
#define SERVO_Y_MIN_US 1026             //µs
#define SERVO_X_MAX_US 1900             //µs
#define SERVO_Y_MAX_US 1900             //µs

//--- MKS Servo TVC Regression Coeffs
#define X_P1 0.00
#define X_P2 -37.94
#define X_P3 1297
//#define X_P3 1340

#define Y_P1 0.00
#define Y_P2 31.48
//#define Y_P3 1600
#define Y_P3 1543

//[-0.013683 6.2274 1133.2]

//EDF Regression Polynomial Coeffs
//Linear V2
#define EDF_P1 0.00
#define EDF_P2 14.69
#define EDF_P3 1167

//Quadratic V2 Mr.Madd
// #define EDF_P1 -0.1409
// #define EDF_P2 20.42
// #define EDF_P3 1117

//Linear V2 EPF
// #define EDF_P1 0.00
// #define EDF_P2 12.74
// #define EDF_P3 1132

//--- Quadratic V1 EPF
// -0.0624 11.8330 1.0686e+03
// #define EDF_P1 -0.0694
// #define EDF_P2 12.1330
// #define EDF_P3 1069

//--- Antiroll motor regression Coeffs (Quadratic)
#define RW_P1_GRAMS -0.013683
#define RW_P2_GRAMS 6.2274
#define RW_P3_GRAMS 1033
// #define RW_MAX_SPEED_RPS 165.00f 
// #define RW_MAX_SPEED_DPS (RW_MAX_SPEED_DPS * 180.00f/PI)

//  -.00025151   0.96511  -891.54


#endif

