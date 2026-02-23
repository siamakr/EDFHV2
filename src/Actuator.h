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
#include "Config.h"


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