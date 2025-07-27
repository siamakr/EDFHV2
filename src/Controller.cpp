#include "Controller.h"
#include "Sensors.h"


Controller::Controller( void ){
    
}

void Controller::init(void){
    //Initialize/zero all actuators
    act.init_servos();
    act.init_rw();
    act.init_edf();
    act.zero_servos();
    act.zero_rw();
}

void Controller::lqr(float r, float p, float y, float gx, float gy, float gz, float z, float vz){
    //Envoke Matricies on the function stack and initialize them
    Matrix<4,1> U = {0.00}; // Control Vector
    Matrix<12,1> error {0.00}; // State error vector
    //Matrix<4,12> K = K_int;
    Matrix<12,1> REF = SP_hover_int;         //store desired setpoint (either by user or from Position Controller Output)
    Matrix<12,1> Xs = {0.00};     //State Vector

    //load state vecotr
    Xs = {r, p, -y, gx, gy, -gz, z, vz};
    REF(0) += U_pos(0);
    REF(1) += U_pos(1);

    // store these values for serial write
    cd.ref0 = REF(0);
    cd.ref1 = REF(1);
    //calculate reference error
    error = Xs - REF;

    //altitude integral action 
    error(8)  = ( ( error(6) >= (-1 * _int_bound_alt) ) && ( error(6) <= _int_bound_alt ) ) ? (cd.e_int(8) +  (error(6) * DT_SEC ) ) : 0.00f;       

    LIMIT(error(8) , -1 * _int_bound_alt, _int_bound_alt);

    //Run LQR Controller + full integral action
    U = -K_int * error;

    //Update the EDF motor control signal with Vehicle weight
    U(3) += MASS * G;         //Normal Mode

    //Calculate each component of the Thrust Vector
    float Tx{ U(3) * U(0)};
    float Ty{ U(3) * U(1) };
    float Tz{ U(3) }; 

    //Calculate desired torque for roll/pitch using alternate method
    cd.trq_x = Tx * COM_TO_TVC;
    cd.trq_y = Ty * COM_TO_TVC;

    //Get the magnitude of the thrust vector
    float Tm{sqrt(pow(Tx,2) + pow(Ty,2) + pow(Tz,2))};

    //Actuation angles w.r.t thrust vector
    float delta_xx{asin(Tx/(Tm))};
    float delta_yy{asin(Ty/(Tm))};
    
    //Using alternate method
    //  U(0) = delta_xx;
    //  U(1) = delta_yy;
    //  U(3) = Tm;

    //convert desired Yaw torque into thrust force per yaw motor
    const float desired_yaw_force = ((U(2) / lrw) / 2) ;                    //dividing by 2 to split force between 2 yaw props
    const float desired_yaw_grams{(desired_yaw_force/G) * 1000};

    //Feedforward
    // U(0) += U_pos(0) * _gain_ff_roll;
    // U(1) += U_pos(1) * _gain_ff_pitch;

    //Filter
    IIR(U(0), cd.u(0), _alpha_servo);
    IIR(U(1), cd.u(1), _alpha_servo);

    //Limit/Clamp
    LIMIT(U(0), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD );
    LIMIT(U(1), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD );
    LIMIT(U(2), 0, MAX_YAW_TORQUE);            //FROM 10 RAD/S TO 80 RAD/S THIS IS CURRENTLY TORQUE THOUGH
    LIMIT(U(3), MIN_THRUST, MAX_THRUST);


    //Actuate
    act.writeEDF((float) U(3));
    act.writeXservo((float) r2d * U(0));
    act.writeYservo((float) r2d * U(1));
    act.writeRW(desired_yaw_grams);

    //Store data for next iteration 
    cd.delta_xx = delta_xx;
    cd.delta_yy = delta_yy;
    cd.e_int = error;
    // cd.u(0) = delta_xx;
    // cd.u(1) = delta_yy;
    cd.u = U;
    cd.Tm = Tm;
}

void Controller::lqr_pos( float x, float y, float vx, float vy, float yaw ){

    Matrix<2,1> output;
    Matrix<6,1> error;

    X_pos = {x, y, vx, vy, 0, 0};                   // Load state vector

    error = SP_pos - X_pos;                         // Calculate state error

    error_integral_x += error(0) * DT_USEC*1;       // Integral computation and limit
    error_integral_y += error(1) * DT_USEC*1;

    LIMIT( error_integral_x, -0.35, 0.35 );         // Clamp integral values
    LIMIT( error_integral_y, -0.35, 0.35 );

    error(4) = error_integral_x;                    // Load integral error 
    error(5) = error_integral_y;

    output = K_pos * error;                         // Run controller

    LIMIT( output(0), -10 * d2r, 10 * d2r );        // Clamp output
    LIMIT( output(1), -10 * d2r, 10 * d2r );

    //U_pos = output;                               // Update 
    //This is now done inside Main.cpp
}

void Controller::actuate( float angle_x_rad, float angle_y_rad, float thrust_force_newton ){
    //angles are switched due to calibration imu axis change, will change this
    //in the polynomial regression definition of the servo angles to tvc angle
    
    act.writeXservo((float)  angle_x_rad);
    act.writeYservo((float)  angle_y_rad);
    act.writeEDF((float) thrust_force_newton);
}

void Controller::actuate_servos(float angle_x_rad, float angle_y_rad){
    act.writeXservo((float)  angle_x_rad);
    act.writeYservo((float)  angle_y_rad);
}

void Controller::actuate_edf(float thrust_force_newton){
    act.writeEDF((float) thrust_force_newton);
}

void Controller::set_reference( control_setpoint_t cs, float value ){

    switch( cs ){
        case SETPOINT_X: SP_pos(0) = value; break;
        case SETPOINT_Y: SP_pos(1) = value; break;
        case SETPOINT_Z: SP_hover_int(6) = value; break;
        case SETPOINT_ROLL: SP_hover_int(0) = value; break;
        case SETPOINT_PITCH: SP_hover_int(1) = value; break;
        case SETPOINT_YAW: SP_hover_int(2) = value; break;
    }
}

void Controller::gain_schedule(float error_roll, float error_gx, float error_pitch, float error_gy, float error_altitude){
    //-- roll 
    const float slope_roll{(0.250 - 0.13)/(d2r*2.00f)};
    _gain_roll = -slope_roll * abs(error_roll) + 0.13;
    LIMIT(_gain_roll, 0.15f, 0.25f);
    //-- pitch 
    const float slope_pitch{(0.250 - 0.13)/(d2r*2.00f)};
    _gain_pitch = -slope_pitch * abs(error_pitch) + 0.13;
    LIMIT(_gain_pitch, 0.15f, 0.25f);

    //-- gx
    const float slope_gx{(0.19f - 0.075) / (0.200f)};
    _gain_gx = -slope_gx * abs(error_gx) + 0.0750;
    LIMIT(_gain_gx, 0.080, 0.12);
    //-- gy
    const float slope_gy{(0.19f - 0.075) / (0.200f)};
    _gain_gy = -slope_gy * abs(error_gy) + 0.0750;
    LIMIT(_gain_gy, 0.080, 0.12);
    //-- altitude
}

float Controller::limit(float value, float min, float max){
    return value <= min ? min : (value >= max ? max : value); 
}

void Controller::LIMIT(float & value, float min, float max ){
    value = ( (value <= min) ? min : (value >= max ? max : value) );
}

float Controller::IIRF(float newSample, float prevOutput, float alpha){
    return ( ( (1.00f - alpha ) * newSample ) + ( alpha * prevOutput ) );
}

void Controller::IIR(float & new_sample, float prev_output, float alpha){
    new_sample = ( ( (1.00f - alpha ) * new_sample ) + ( alpha * prev_output ) );
}

void Controller::print_debug(void){  
    Serial << debug << "\n" ;
}