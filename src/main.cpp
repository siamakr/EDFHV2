#include <Arduino.h>
#include "Sensors.h"
#include "Controller.h"

#define STATEMACHINE
//#define SENSOR_ONLY 
//#define QUICKTESTENV

//Envoking Class objects 
Sensors sensor; 
Controller control; 

//..... State Machine Vars .....// 
//... Timing Vars ...//
static float sensor_timer{0};
static float print_timer{0.00f};
static float flow_timer{0.00f};
static float pos_controller_timer{0.00f};
static float mst{0.00f};       //Mission Start Timer
static float init_timer{0.00f};
static float elapsed_time{0.00f};
//... Flags ...//
static bool is_calibrated{false};  
static bool start_flag{false};
static bool prime_start_flag{false};
int count = 0;


void print_control_imu(void);
void step_response_state_machine(float step_interval_ms, float angle);
void run_hover_program(void);
void print_control_imu_estimater(void);
void print_estimator_main();
void print_controller(void);
void flow_debugger(void);
void user_read_int(int & value, String message);
void user_read_anykey(String message);
void user_read_float(float & value, String message);

#ifdef STATEMACHINE
void setup() {

  Serial.begin(115200);
  control.init();
  #ifndef SENSOR_ONLY
  //--- Initialize control Actuators ---//
  //control.init();
   //control.act.init_servos();
   //control.act.init_rw();
   //control.act.init_edf();
   //control.act.zero_servos();
   //control.act.zero_rw();
  #endif
  //control.act.init_servos();

  //--- Initialize Sensors ---//
    sensor.flow_init();
    sensor.lidar_init();
    sensor.fsm_init();
  //--- Initialize Sensors ---//

  //--- Initialize initial coniditions of flight and set state machine start ---//
    control.set_reference(SETPOINT_Z, 0.500f);
    control.set_reference(SETPOINT_YAW, d2r*60.00);
  //--- Initialize initial coniditions of flight and set state machine start ---//

  control.status = CONTROL_STATUS_IMU_CALIBRATION;
  //Serial.println("testing begin...");  
  Serial.println("roll,pitch,yaw,qi,qj,qk,qw,x,y,z,vx,vy,vz,linacc,gyroacc,quatacc"); 

  //init_timer = millis(); 
}

void loop() {
  // Sample FSM as fast as possible since using SPI (Doesn't like "delay()" function too much)
  sensor.sample_fsm();
  //--- State Machine ---//
  switch (control.status){
    case CONTROL_STATUS_STATIONARY:
      if(Serial.available() == true){     //wait for user input to start flight
        //Init all timers
          init_timer = micros();  //resets edf priming timer
          mst = micros();         //mission start timer
          sensor_timer = micros();
          flow_timer = micros();
          print_timer = micros();

        //Set init set-points
          control.set_reference(SETPOINT_Z, 0.400f);
          control.set_reference(SETPOINT_YAW, d2r*60); 

        //Change state   
          control.status = CONTROL_STATUS_EDF_PRIMING;  //changes state to edf priming on next state
      }
    break;
    
    case CONTROL_STATUS_EDF_PRIMING:
      control.act.edf.writeMicroseconds(1250);
      if(micros() - init_timer <= 6000000){          //hold EDF priming PWM signal for 6 seconds 
        control.status = CONTROL_STATUS_EDF_PRIMING;
      }else{
        Serial.println("priming finished..."); 
        Serial.println("testing begin...");     //Tells Python app to begin storing data
        Serial.println("time,x,vx,u0,roll,roll_sp,gx,y,vy,u1,pitch,pitch_sp,gy,yaw,yaw_sp,u2,z,vz,tm,rollgain,pitchgain,gxgain,gygain,s0,s1,s2"); 
        control.status = CONTROL_STATUS_FLYING; 
      }
    break;

    case CONTROL_STATUS_LIFTOFF:
      // control._gain_z = 0.10f;
      // control._gain_z_int = 2.00f;

      // if(sensor.estimate.z > 0.0 && sensor.estimate.z < 0.38){
      //   control._gain_roll = .43f;
      //   control._gain_gx = 0.08f;
      // }
      run_hover_program();
      step_response_state_machine(2500000, 3.00f);
      if(sensor.estimate.z >= 0.380f){ 
        control.status = CONTROL_STATUS_FLYING;
      }
    break;

    case CONTROL_STATUS_FLYING:
      // control._gain_z = 7.100f;
      // control._gain_vz = 5.6942f;
      // control._gain_z_int = 7.0f;

      run_hover_program();
      step_response_state_machine(2500000, 3.00f);
    break;
    
    case CONTROL_STATUS_LANDING:
      // control._gain_z = 0.00f;
      // control._gain_z_int = 3.500f;

      run_hover_program();
      step_response_state_machine(2500000, 3.00f);
    break;

    case CONTROL_STATUS_IMU_CALIBRATION:
      if(micros() - print_timer >= (DT_USEC * 20)  ){       //must use this type of timer for delays 
        print_timer = micros();
        sensor.print_fsm();
      }
      //--continously sample IMU and check if all sensors within are fully calibrated (3,3,3)
      if(sensor.data.gyroAccuracy == 3 && sensor.data.quatAccuracy == 3 && sensor.data.linAccuracy == 3){
        count++;
        
        //Serial.println("IMU calibration done..."); 
        //Serial.println("Now in wait mode..."); 
       //control.status = CONTROL_STATUS_STATIONARY;
       }if(count >= 10){
       // Serial.println("+++"); 
        Serial.println("calibration done..."); 
        control.status = CONTROL_STATUS_STATIONARY;
        }else{
          control.status = CONTROL_STATUS_IMU_CALIBRATION;
        }
    break;
    
    default:
      control.status = CONTROL_STATUS_STATIONARY;
    break;
  }
}

void run_hover_program(void){
  //Sample IMU as fast as possible
  //..... Sensor Timer .....//
  if(micros() - sensor_timer >= DT_USEC){       //DT_USEC = 5µs
    sensor_timer = micros();    //update timer
    sensor.sample_lidar();      //read lidar 
    sensor.run_estimator();     //execute estimator

    //run position controller to get roll_desired, pitch_desired vals
    control.lqr_pos(sensor.estimate.vx, 
                    sensor.estimate.vy, 
                    sensor.estimate.x, 
                    sensor.estimate.y, 
                    sensor.data.yaw);

    // //take output of pos controller and set as reference for attitude controller
      control.set_reference(SETPOINT_ROLL, control.U_pos(0));
      control.set_reference(SETPOINT_PITCH, control.U_pos(1));

    //run attitude controller + control allocator
    control.lqr(sensor.data.roll, 
                  sensor.data.pitch, 
                  sensor.data.yaw, 
                  sensor.data.gx, 
                  sensor.data.gy, 
                  sensor.data.gz, 
                  sensor.estimate.z, 
                  sensor.estimate.vz);

  }
  //..... Optical Flow Timer .....//   
  if(micros() - flow_timer >= DT_USEC*2){      //run at half speed
    flow_timer = micros();
    sensor.sample_flow();       //read flow
  }

  //..... Print Timer .....//
  if(micros() - print_timer >= (DT_USEC*4)  ){  //print at 1/4 speed to not bog up Serial
    print_timer = micros();
    //control.print_debug();
    print_control_imu();
    //print_control_imu_estimater();
    //print_controller();
    //sensor.print_estimator();
    //print_estimator_main();
    //flow_debugger();

  }
}

// Used to run step-response testing as guidance path
void step_response_state_machine(float step_interval_ms, float angle){
  elapsed_time = micros() - mst;

  if(elapsed_time >= (step_interval_ms * 1) && elapsed_time < (step_interval_ms * 2) )
  {
    // control.set_reference(SETPOINT_PITCH, 0.00f);
    // control.set_reference(SETPOINT_ROLL , 0.00f);
    control.set_reference(SETPOINT_Z , 0.50f);
    control.set_reference(SETPOINT_X , 0.00f);
    control.set_reference(SETPOINT_Y , 0.00f);
  }

  else if(elapsed_time >= ( step_interval_ms * 2) && elapsed_time < (step_interval_ms * 3)){
    control.set_reference(SETPOINT_Z , 0.50f);
    control.set_reference(SETPOINT_X , 0.00f);
    control.set_reference(SETPOINT_Y , 0.00f);
  }

  else if(elapsed_time >= ( step_interval_ms * 3 ) && elapsed_time < (step_interval_ms * 4)){
    control.set_reference(SETPOINT_Z , 0.50f);
    control.set_reference(SETPOINT_X , 0.00f);
    control.set_reference(SETPOINT_Y , 0.00f);
  }

  else if(elapsed_time >= (step_interval_ms * 4) && elapsed_time <= (step_interval_ms * 5)){
    control.set_reference(SETPOINT_Z , 0.50f);
    control.set_reference(SETPOINT_X , 0.00f);
    control.set_reference(SETPOINT_Y , 0.00f);
  }

  else if(elapsed_time >= (step_interval_ms * 5) && elapsed_time < (step_interval_ms * 6)){
    control.set_reference(SETPOINT_Z , 0.50f);
    control.set_reference(SETPOINT_X , 0.00f);
    control.set_reference(SETPOINT_Y , 0.00f);
  }

  else if(elapsed_time >= (step_interval_ms * 6) && (elapsed_time < step_interval_ms * 7)){
    control.set_reference(SETPOINT_Z , 0.50f);
    control.set_reference(SETPOINT_X , 0.00f);
    control.set_reference(SETPOINT_Y , 0.00f);
  }

  else if(elapsed_time >= (step_interval_ms * 7) && (elapsed_time < step_interval_ms * 8)){
    control.status = CONTROL_STATUS_LANDING;
    control.set_reference(SETPOINT_Z , 0.050f);
    control.set_reference(SETPOINT_X , 0.00f);
    control.set_reference(SETPOINT_Y , 0.00f);
  }

  // if(elapsed_time >= (step_interval_ms * 8) && (elapsed_time < step_interval_ms * 9))
  // {
  //   control.status = CONTROL_STATUS_LANDING;
  //   control.set_reference(SETPOINT_Z , 0.050f);
  //   control.set_reference(SETPOINT_X , 0.00f);
  //   control.set_reference(SETPOINT_Y , 0.00f);
  // }

  // if(elapsed_time >= (step_interval_ms * 9) && (elapsed_time < step_interval_ms * 10))
  // {
  //   control.set_reference(SETPOINT_PITCH, -angle);
  //   control.set_reference(SETPOINT_ROLL , angle);
  // }

  // if(elapsed_time >= (step_interval_ms * 10) && (elapsed_time < step_interval_ms * 11))
  // {
  //   control.set_reference(SETPOINT_PITCH, angle);
  //   control.set_reference(SETPOINT_ROLL , angle);
  // }

  // if(elapsed_time >= (step_interval_ms * 11) && (elapsed_time < step_interval_ms * 12))
  // {
  //   control.set_reference(SETPOINT_PITCH, 0.00f);
  //   control.set_reference(SETPOINT_ROLL , 0.00f);
  // }

  // if(elapsed_time >= (step_interval_ms * 12) && (elapsed_time < step_interval_ms * 13))
  // {
  //   control.set_reference(SETPOINT_PITCH, 2*angle);
  //   control.set_reference(SETPOINT_ROLL , 2*angle);
  // }

  // if(elapsed_time >= (step_interval_ms * 13) && (elapsed_time < step_interval_ms * 14))
  // {
  //   control.set_reference(SETPOINT_PITCH, -2*angle);
  //   control.set_reference(SETPOINT_ROLL , -angle);
  // }

  // if(elapsed_time >= (step_interval_ms * 15) && (elapsed_time < step_interval_ms * 16))
  // {
  //   control.set_reference(SETPOINT_PITCH, 0.00f);
  //   control.set_reference(SETPOINT_ROLL , 0.00f);
  // }

  //else 
 
  #ifndef SENSOR_ONLY
    if(elapsed_time >= (step_interval_ms * 7) && sensor.estimate.z <= .150f){
      control.act.edf_shutdown();
      control.act.zero_servos();
      control.act.zero_rw();
      //while(1);
      Serial.println("testing done..."); 
      control.status = CONTROL_STATUS_STATIONARY;
    }
  #endif
}

//TODO: This function should go in Controller!!!
void print_control_imu(void){
  char text[250];
  //              Roll  RollSP pitch  pitchSP  yaw       gx      gy    gz        ax    ay      az        cdax   cdaxx   pwmx   cday   cdayy  pwmy  Tm    pwmedf
  sprintf(text, "%0.6f,   %0.5f, %0.5f,   %0.5f, %0.5f, %0.5f,  %0.5f,  \t  %0.5f, %0.5f,   %0.5f, %0.5f, %0.5f,  %0.5f,  \t  %0.5f, %0.5f, %0.5f,     \t %0.5f, %0.5f,   %0.5f,  %0.5f, %0.5f,  %0.5f, %0.5f,      %i,%i,%i    ",
    
    elapsed_time/1000000,
    sensor.estimate.x,
    sensor.estimate.vx,
    r2d*control.cd.u(0),
    r2d*sensor.data.roll,
    r2d*control.cd.ref0, 
    sensor.data.gx,
    
    sensor.estimate.y,
    sensor.estimate.vy,
    r2d*control.cd.u(1),
    r2d*sensor.data.pitch, 
    r2d*control.cd.ref1,
    sensor.data.gy,
    
    r2d*sensor.data.yaw,
    r2d*control.SP_hover_int(2),
    control.cd.u(2),

    sensor.estimate.z,
    sensor.estimate.vz,
    control.cd.Tm,
    control.act.ad.pwmedf,
    control._gain_pitch,
    control._gain_gx,
    control._gain_gy,


    sensor.data.linAccuracy,
    sensor.data.gyroAccuracy,
    sensor.data.quatAccuracy
    );

  Serial.println(text);

}

void print_control_imu_estimater(void)
{
  char text[280];
  //              Roll  RollSP pitch  pitchSP  yaw       gx      gy    gz        ax    ay      az        cdax   cdaxx   pwmx   cday   cdayy  pwmy  Tm    pwmedf
  sprintf(text, "%0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f,     \t %0.5f, %0.5f,    \t  %0.5f, %i,  \t  ",
    sensor.data.ez,
    sensor.estimate.z,
    sensor.estimate.vz,

    sensor.estimate.vx,
    sensor.estimate.x,
    
    sensor.estimate.vy,
    sensor.estimate.y,

    // sensor.data.ez,
    // sensor.data.evz_accel,
    // sensor.estimate.z,
    // sensor.estimate.vz,
    // sensor.debug.xpre_z,

    control.cd.Tm,
    control.act.ad.pwmedf);

  Serial.println(text);

}

void print_controller(void){
  char text[250];
  //              roll  rollsp    pitch  pitchsp    yaw        deltax deltay deltax deltay  u0    u1      u2      u3        estvz estz    dataz   dataez  ax      ay    az        Tedf    pwmedf   
  sprintf(text, "%0.5f, %0.5f,%i,       %0.5f,  %0.5f,  %0.5f,       %0.5f,  %0.5f,  %0.5f,  %0.5f,         %0.5f, %0.5f,     %0.5f, %0.5f,  %0.5f,  %0.5f,    %0.5f, %i,       %i, %i, %i ",
  // r2d*sensor.data.roll,                 // 1
  // r2d*control.SP_hover_int(0),          // 2
  // r2d*sensor.data.pitch,                // 3
  // r2d*control.SP_hover_int(1),          // 4
  // r2d*sensor.data.yaw,                // 3
  // r2d*control.SP_hover_int(2),          // 4

  r2d*sensor.data.yaw,                // 3
  control.cd.u(2),
  control.act.ad.pwmrw,


  r2d*control.cd.delta_xx,              //6
  r2d*control.cd.delta_x,               //7
  r2d*control.cd.delta_yy,              //8
  r2d*control.cd.delta_y,               //9

  r2d*control.cd.u(0),                  //10
  r2d*control.cd.u(1),                  //11
  (control.cd.u(2)/RW_JZZ),                      //12
  control.cd.u(3),

  sensor.estimate.vz,                   //13
  sensor.estimate.z,                    //14

  sensor.data.z,                        //15
  sensor.data.ez,                       //16
  control.SP_hover_int(6),              //17

  // sensor.data.ax,                       //18
  // sensor.data.ay,                       //19
  // sensor.data.az,                       //20

  control.cd.Tedf,                      //21
  control.cd.u(3),                      //22
    control.act.ad.pwmedf,              //23

  sensor.data.linAccuracy,
  sensor.data.gyroAccuracy,
  sensor.data.quatAccuracy);

  Serial.println(text);

}
void print_estimator_main(void){
  char text[250];

  sprintf(text, "%0.5f, %0.5f, %0.5f, %0.5f, \t   %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f, \t  %i, %i, %i ",
  r2d*sensor.data.roll,
  r2d*sensor.data.pitch,
  r2d*sensor.data.yaw,
  sensor.data.z,

  sensor.estimate.x,
  sensor.estimate.y,
  sensor.estimate.z,

  sensor.estimate.vx,
  sensor.estimate.vy,
  sensor.estimate.vz,

  sensor.data.ax,
  sensor.data.ay,
  sensor.data.az,

  sensor.data.linAccuracy,
  sensor.data.gyroAccuracy,
  sensor.data.quatAccuracy);

  Serial.println(text);
}

void flow_debugger(void){
  char text[250];

  sprintf(text, "  %0.5f, %0.5f,  %0.5f, %0.5f, %0.5f,\t       %0.5f, %0.5f, %0.5f, \t       %0.5f, %0.5f, %0.5f, \t  %i, %i, %i ",    

  sensor.data.ax,
  sensor.debug.vx_raw,
  sensor.data.vx,
  sensor.debug.x_int,
  sensor.debug.xpre_vx,

  // sensor.data.ay,
  // sensor.debug.vy_raw,
  // sensor.data.vy,
  // sensor.debug.y_int,
  // sensor.debug.xpre_vy,

  r2d*sensor.data.roll,
  r2d*sensor.data.pitch,
  r2d*sensor.data.yaw,
  
  sensor.estimate.vx,
  sensor.estimate.vy,
  sensor.estimate.vz,


  sensor.data.linAccuracy,
  sensor.data.gyroAccuracy,
  sensor.data.quatAccuracy);

  Serial.println(text);
}

void user_read_int(int & value, String message){
        //get start microseconds from user and store
    Serial.print(message);

    while (Serial.available()) Serial.read(); //Clear anything in RX buffer
    while (Serial.available() == 0) delay(10); //Wait for user to press key

    //Read user input
    value = (int)Serial.parseInt();
    Serial.println();  
}

void user_read_float(float & value, String message){
      //get start microseconds from user and store
  Serial.print(message);

  while (Serial.available()) Serial.read(); //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10); //Wait for user to press key

  //Read user input
  value = Serial.parseFloat();
  Serial.println();  
}

void user_read_anykey(String message){
    Serial.println(message);
    while (Serial.available()) Serial.read(); //Clear anything in RX buffer
    while (Serial.available() == 0) delay(10); //Wait for user to press key
}

#endif

#ifdef QUICKTESTENV

#include "Actuator.h" 

Actuator a;
bool flag{false};

void setup(){
  Serial.begin(115200);
  a.init_servos();
}

void loop(){
  //   a.servo_dance_x(12.00f, 15);
  //  // delay(2000);
  //   a.servo_dance_x(10.00f, 10);
  //  // delay(2000);
  //   a.servo_dance_x(8.00f, 7);
  //   a.servo_dance_x(7.00f, 5);
  //   a.servo_dance_x(5.00f, 3);
  //   a.servo_dance_x(3.00f, 2);
  //   a.servo_dance_x(2.00f, 1);
  //   a.zero_servos();
  //  // delay(2000);


  //   delay(10000);

  while(a.step_response_x_servo(5));
  delay(1000); 
  a.zero_servos();
  while(a.step_response_y_servo(5));
  delay(1000);
  a.zero_servos();

}
#endif

      //control.print_debug();
     // print_control_imu();
      //print_control_imu_estimater();
      //print_controller();
      //sensor.print_estimator();
      // print_estimator_main();
      //flow_debugger();