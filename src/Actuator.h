/* This is a custom Actuation class to control the 2 MKS servo-motors
* and the Mr. Madd (90mm) Electric Ducted Fan (EDF) motor to be used 
* by Controller and Teststand classes. 
* This class handles the conversion between desired TVC angles and the
* regression curves that describes the mapping of TVC angle to servo 
* PWM values. 
* Similarly, the EDF motor also has this conversion handled here including all 
* initializations.  
*/

#ifndef _EDFH_ACTUATOR_H
#define _EDFH_ACTUATOR_H

#include <Arduino.h>
#include "Servo.h"
#include <math.h>
#include "Constants.h"

/*
//define which board being used 
//#define UNO
#define TEENSY 


#ifdef TEENSY
// Teensy 4.1 Pin assignments
#define RW_PIN 37
#define EDF_PIN 36
#define YSERVO_PIN 33
#define XSERVO_PIN 14
#endif

#ifdef UNO 
#define RW_PIN 37
#define EDF_PIN 6
#define YSERVO_PIN 33
#define XSERVO_PIN 14
#endif

//// EDF Params 
#define EDF_OFF_PWM 900                 //uSec
#define EDF_MIN_PWM 1250                //uSec
#define EDF_MAX_PWM 1650                //uSec
#define EDF_MAX_SUSTAINED_PWM 1730      //uSec
#define EDF_IDLE_PWM 1260               //uSec

#define SERVO_X_CENTER_US 1390          //µs
#define SERVO_Y_CENTER_US 1575          //µs
#define SERVO_X_MIN_US 1026             //µs
#define SERVO_Y_MIN_US 1026             //µs
#define SERVO_X_MAX_US 1900             //µs
#define SERVO_Y_MAX_US 1900             //µs

// MKS Servo TVC Regression Coeffs
#define X_P1 0.00
#define X_P2 -37.94
//#define X_P3 1297
#define X_P3 1320

#define Y_P1 0.00
#define Y_P2 31.48
#define Y_P3 1600
//#define Y_P3 1543

//[-0.013683 6.2274 1133.2]

//EDF Regression Polynomial Coeffs
//Linear V2
// #define EDF_P1 0.00
// #define EDF_P2 14.69
// #define EDF_P3 1167

//Quadratic V2 Mr.Madd
//#define EDF_P1 -0.1409
//#define EDF_P2 20.42
//#define EDF_P3 1117

//Linear V2 EPF
#define EDF_P1 0.00
#define EDF_P2 12.74
#define EDF_P3 1132

//Quadratic V1 EPF
// -0.0624 11.8330 1.0686e+03
// #define EDF_P1 -0.0694
// #define EDF_P2 12.1330
// #define EDF_P3 1069

//Antiroll motor regression Coeffs (Quadratic)
#define RW_P1_GRAMS -0.013683
#define RW_P2_GRAMS 6.2274
#define RW_P3_GRAMS 1033
// #define RW_MAX_SPEED_RPS 165.00f 
// #define RW_MAX_SPEED_DPS (RW_MAX_SPEED_DPS * 180.00f/PI)

//  -.00025151   0.96511  -891.54
//Physical Constraints
#define MAX_VEHICLE_ANGLE_DEG 35.00f
#define MAX_TVC_ANGLE_DEG 8.00f
#define MAX_EDF_THRUST_NEWTONS 31.00f
*/

#define d2r (PI/180.00f)
#define r2d (180.00f/PI)

template<typename T>
T clamp( T Value, T Min, T Max){
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}

typedef struct {
  int pwmx, pwmy, pwmedf, pwmrw;
  float ang_x, ang_y, f_thrust, antirotor_thrust_g;
}actuator_data_t;


class Actuator{
public:
  Servo sx; 
  Servo sy; 
  Servo edf;
  Servo rw;
  actuator_data_t ad;

  Actuator();

  //...Initialiaztion Functions...// 
  void init(void);

  void init_servos(void);
  void init_edf(void);
  void init_rw(void);
  void zero_servos();
  void zero_rw(void);
  void edf_shutdown(void);
    void prime_edf(void);
  void prime_edf(int delay_time_ms);
  bool prime_edf(int delay_time_ms, float start_timer);

  //... Actuation Functions ...//
  void writeXservo(float angle);
  void writeXservo(int pwm);
  void writeYservo(float angle);
  void writeYservo(int pwm);
  void writeEDF(float Ft);
  void writeEDF(int pwm);
  void writeRW(float grams);
  bool servo_dance(float start_angle, int delay_time_ms);
  void LIMIT(int & value, int min, int max);

private:

};

#endif