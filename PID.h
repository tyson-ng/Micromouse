// PID.h - PID controllers for wall-following, steering, and velocity
#ifndef PID_H
#define PID_H
#include "GlobalConfig.h"

// Wall-following PID
float Kp = 4.0; float Ki = 0.0; float Kd = 3.5;      
float error = 0;
float previous_error = 0;
float integral = 0;

// Steering PID (gyro-based)
float Kp_steer = 15.0; float Ki_steer = 0.05; float Kd_steer = 2.5;
float prev_err_steer = 0; float integral_steer = 0;

// Velocity PID
float Kp_vel = 2.2;  
float Ki_vel = 0.35; 
float Kd_vel = 0.08;

const float BASE_PWM = 100.0; // feedforward base

float target_speed_L = 60.0; float target_speed_R = 60.0;

float prev_err_vel_L = 0, integral_vel_L = 0;
float prev_err_vel_R = 0, integral_vel_R = 0;
long last_countL = 0, last_countR = 0;
float current_vel_L = 0, current_vel_R = 0;

void calculateVelocity(long currL, long currR) {
    current_vel_L = (float)(currL - last_countL);
    current_vel_R = (float)(currR - last_countR);
    last_countL = currL; last_countR = currR;
}

void calculateVelocityPID(float tL, float tR, float &outL, float &outR) {
    float errL = tL - current_vel_L;
    integral_vel_L = constrain(integral_vel_L + errL, -200, 200);
    float pidL = Kp_vel * errL + Ki_vel * integral_vel_L + Kd_vel * (errL - prev_err_vel_L);
    prev_err_vel_L = errL;
    outL = BASE_PWM + pidL;

    float errR = tR - current_vel_R;
    integral_vel_R = constrain(integral_vel_R + errR, -200, 200);
    float pidR = Kp_vel * errR + Ki_vel * integral_vel_R + Kd_vel * (errR - prev_err_vel_R);
    prev_err_vel_R = errR;
    outR = BASE_PWM + pidR;
}

// Wall error: difference between left and right side sensors
float getError() {
    error = IRd[0] - IRd[1] ;
    error = constrain(error, -20, 20);
    return error;
}

// Single-wall error (left only)
float getErrorL(){
    error = 30 - IRd[0];
    error = constrain(error, -10, 10);
    return error;  
}

// Single-wall error (right only)
float getErrorR(){
    error = IRd[1] - 30;
    error = constrain(error, -10, 10);
    return error;
}

float calL(){
  float e = getErrorL();
  if ((e > 0 && integral < 20) || (e < 0 && integral > -20)) {
    integral += e;
  }
  float pid = Kp * e + Kd * (e - previous_error) + Ki * integral;
  previous_error = e;
  return pid;
}

float calR(){
  float e = getErrorR();
  if ((e > 0 && integral < 20) || (e < 0 && integral > -20)) {
    integral += e;
  }
  float pid = Kp * e + Kd * (e - previous_error) + Ki * integral;
  previous_error = e;
  return pid;
}

float calculate_pid() {
  float e = getError();
  if ((e > 0 && integral < 20) || (e < 0 && integral > -20)) {
    integral += e;
  }
  float pid = Kp * e + Kd * (e - previous_error) + Ki * integral;
  previous_error = e;
  return pid;
}


#endif