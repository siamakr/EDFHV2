///// Sensors.h /////

#ifndef _EDFH_SENSORS_H
#define _EDFH_SENSORS_H

#include <Wire.h>
#include <SPI.h>
#include "BNO080.h"
#include "LIDARLite_v3HP.h"
#include "PMW3901.h"
#include <Math.h>
#include <BasicLinearAlgebra.h>
#include <stdint.h>
#include "Constants.h"

using namespace BLA;

/*
#define PMW3901_FOV 42.0f           // Degress
#define PMW3901_FOCAL 412.27f       // Focal length in pixels (found experimentally using pin-hole model)
#define PMW3901_WIDTH 30            // Pixels

//..... Defines .....//
#define DT_USEC 5000
#define DT_MSEC 5.00f
#define DT_SEC (0.0050)
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)

#define FSM_PITCH_OFFSET_RAD (d2r * 1.3772)
#define FSM_ROLL_OFFSET_RAD (d2r * 0.6578f)
#define FSM_YAW_OFFSET_RAD (d2r * 0.00f)
// #define FSM_ROLL_OFFSET_RAD (d2r * -1.28331)
// #define FSM_PITCH_OFFSET_RAD (d2r * -2.18253)

*/

//..... SPI Pin Definitons .....//
const byte imuCSPin = 10;
const byte imuWAKPin = 7;
const byte imuINTPin = 8;
const byte imuRSTPin = 9;

const float roll_offset{d2r*1.3772};
const float pitch_offset{d2r*0.6578f};

//..... Lidar Definitions .....//
//uint8_t garminAddress{0x62};

//.......... Structure Definitions ..........//
typedef struct{
    float ax_nf, ay_nf, az_nf;              //nf = no filter
    float ax_f, ay_f, az_f;                 //f = filtered
    float vx_raw, vy_raw;                   //direct output from flow sensor
    float vx_comp, vy_comp;                 //compensated flow measurements
    float x_comp, y_comp;
    float x_int, y_int;                     //directly integrated flow measurements 
    float xpre_vx, xpre_vy, xpre_vz, xpre_z;        //the predicted values from estimator (prior to estimation step)
}debug_t;

typedef struct{
    float roll, pitch, yaw;
    float gx, gy, gz;
    float ax, ay, az;
    float vx, vy;           //these are raw flow velocity data before kalman filter
    float axw, ayw, azw;
    float qi, qj, qk, qw;
    float x, y, z; //this will be removed when imu_data_t is envoked
    float evx, evy, evz, evz_accel, evz_accel_prev;
    float ex, ey, ez;
    byte linAccuracy{0};
    byte gyroAccuracy{0};
    //byte magAccuracy{0};
    float quatRadianAccuracy{0};
    byte quatAccuracy{0}; 
    
    struct{ // Bitfield, using 1 byte, to represent if new measurements are available
        uint8_t imu     : 1;
        uint8_t flow    : 1;
        uint8_t lidar   : 1;
        uint8_t pos     : 1;
    } status;
} fsm_data_t;

typedef struct{
    float x, y, z;
    float vx, vy, vz; 
} estimator_data_t;

//...... Class Definition .....//
class Sensors{    
public:
    uint8_t cspin = 29;
    fsm_data_t data;
    estimator_data_t estimate;
    BNO080 fsm;
    PMW3901 flow;
    LIDARLite_v3HP garmin;
    uint16_t distance;
    debug_t debug;

    volatile float yaw_origin{0.00f};
    volatile float yaw_raw{0.00f};

    // Estimator matrixes
    Matrix<6,6> A = {   1,  0,  0,  DT_SEC, 0,  0,          //x
                        0,  1,  0,  0,  DT_SEC, 0,          //y
                        0,  0,  1,  0,  0,  DT_SEC,         //z
                        0,  0,  0,  1,  0,  0,              //vx
                        0,  0,  0,  0,  1,  0,              //vy
                        0,  0,  0,  0,  0,  1 };            //vz

    Matrix<6,3> B = {   0.5*pow(DT_SEC,2),  0,              0,         
                        0,              0.5*pow(DT_SEC,2),  0,         
                        0,              0,              0.5*pow(DT_SEC,2),  
                        DT_SEC,             0,              0,         
                        0,              DT_SEC,             0,         
                        0,              0,              DT_SEC };
                    //  ax              ay                  az
 
    Matrix<6,6> H = {   0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,   //x
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,   //y
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,   //z  (LIDAR)
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,   //vx (FLOW)
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,   //vy (FLOW)   
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f};  //vz



    // State vector
    Matrix<6,1> Xe= {0.00f,0.00f,0.00f,0.00f,0.00f,0.00f} ;

    // Prediction vector
    Matrix<6,1> Xpre = {0.00f,0.00f,0.00f,0.00f,0.00f,0.00f};  

    // Measurement vector
    Matrix<6,1> Z = {0.00f,0.00f,0.00f,0.00f,0.00f,0.00f};

    // Input vector
    Matrix<3,1> U_est = {0.00f,0.00f,0.00f};

    // Estimator gain
     Matrix<6,6> Kf = {        0.1000,         0,         0,    0.0008,         0,         0,
                                    0,    0.1000,         0,         0,    0.0008,         0,
                                    0,         0,    0.0768,         0,         0,    0.085,
                               0.0008,         0,         0,    0.7000,         0,         0,
                                    0,    0.0008,         0,         0,    0.7000,         0,
                                    0,         0,    1.0000,         0,         0,    0.0000};


 
        //FILTER PARAMS
    const float _alpha_gyro{0.10};                //GYROSCOPE FILTER ALPHA
    const float _alpha_accel{0.050};               //ACCELEROMETER SIGNAL FILTER ALPHA 

    Sensors(void);
    void init(void);
    void fsm_init(void);
    void lidar_init(void);
    void flow_init(void);
    void save_calibrate_fsm(void);
    void sample_fsm(void);
    void sample_lidar(void);
    void sample_flow(void);
    void run_estimator(void);
    void rotate_to_world( float * vector );
    void set_origin(void);
    void update_pos( float x, float y );
    float rotate_yaw(float yaw);
    void clamp(float &value, float min, float max);
    float IIR(float newSample, float prevOutput, float alpha);
    void print_debug(void);
    void print_imu(void);
    void print_fsm(void);
    void print_estimator(void);

private:

};

#endif
