// RobotMotion.h - High-level movement: forward, turn 90, turn 180
#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include "GlobalConfig.h"
#include "Sensors.h"
#include "Motors.h"
#include "PID.h"
#include "Gyro.h"

uint16_t DELAY_AFTER_CELL = 50;
uint16_t DELAY_AFTER_TURN = 70;

void forward_one_cell() {
    resetEncoder(); 
    previous_error = 0;
    integral = 0;
    
    bool isLeftWall = false; 
    bool isRightWall = false;
    static uint32_t lastDebugTime = 0;

    int WALL_EXIST_VAL = 28;
    int WALL_LOST_VAL  = 20;
    int WALL_TOO_CLOSE = 34; // emergency steer threshold

    while ((abs(countLeft) + abs(countRight)) / 2 <= CELL_TICKS) {
        
        ir_read_once();

        // Update wall state with hysteresis
        if (IRd[0] >= WALL_EXIST_VAL) isLeftWall = true;
        else if (IRd[0] <= WALL_LOST_VAL) isLeftWall = false;

        if (IRd[1] >= WALL_EXIST_VAL) isRightWall = true;
        else if (IRd[1] <= WALL_LOST_VAL) isRightWall = false;

        float current_pid = 0;
        
        // Emergency steer if too close to a wall
        if (IRd[0] > WALL_TOO_CLOSE) {
            current_pid = 20;
        } 
        else if (IRd[1] > WALL_TOO_CLOSE) {
            current_pid = -20;
        }
        // Normal wall-following PID
        else if (isLeftWall && isRightWall) {
            current_pid = calculate_pid();
        } 
        else if (isLeftWall) {
            current_pid = calL();
        } 
        else if (isRightWall) {
            current_pid = calR();
        } 
        else {
            current_pid = 0;
            integral = 0;
        }

        int leftSpeed  = constrain(PWM_FWD + current_pid, 0, MAX_PWM);
        int rightSpeed = constrain(PWM_FWD - current_pid, 0, MAX_PWM);
        setMotor(leftSpeed, rightSpeed);
        
        // Debug output every 100ms
        if (millis() - lastDebugTime > 100) {
            lastDebugTime = millis();
            BT.print("L:"); BT.print(IRd[0]); 
            BT.print(" R:"); BT.print(IRd[1]);
            BT.print(" | PID:"); BT.print(current_pid, 1);
            BT.print(" | Spd:"); BT.print(leftSpeed); 
            BT.print("/"); BT.println(rightSpeed);
        }
    }
}

void turn_90_left() {
    resetEncoder();
    while((abs(countLeft)+abs(countRight))/2 <= TURN90_TICKS){
        setMotor(-PWM_TURN, +PWM_TURN);
    }
    drive_brake();
    motorStop();
    wait_ms(100);
}

void turn_90_right(){
    resetEncoder();
    while((abs(countLeft)+abs(countRight))/2 <= TURN90_TICKS){
        setMotor(+PWM_TURN, -PWM_TURN);
    }
    drive_brake();
    motorStop();
    wait_ms(100);
}

void turn_180_exact(){
    resetEncoder();
    while((abs(countLeft)+abs(countRight))/2 <= TURN180_TICKS){
        setMotor(-PWM_TURN, +PWM_TURN);      
    }
    drive_brake();
    motorStop();
    wait_ms(100);
}

#endif // ROBOT_MOTION_H
