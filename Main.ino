// Main.ino - Micromouse entry point

#include <Arduino.h>
#include <stdlib.h>
#include <HardwareSerial.h>

#include "GlobalConfig.h"
#include "Sensors.h"
#include "Motors.h"
#include "PID.h"
#include "RobotMotion.h"
#include "FloodFill.h"
#include "Gyro.h"
#include "Speaker.h"

void setup(){
  BT.begin(BT_BAUD);
  motor_init();
  ir_init();
  setupGyro();
  speaker_init();

  generateInitialWalls();
  currentXY.x=STARTING_X;
  currentXY.y=STARTING_Y;
  currentHeading=STARTING_HEADING;
  target=STARTING_TARGET;

  //playToTiTe();
  delay(1000);
}

void loop(){
    // Uncomment the test you want to run:

    // --- Turn left 90 ---
    //  turn_90_right();
    //  delay(5000);
    //  for (int i = 0; i < 10; i++) {
       forward_one_cell();
    // }  
    //testIRSensors();
    // BT.println("Done 15 cells");
    // motorStop(); 
    // turn_180_exact();
    // delay(5000);
    // while(1);

    // --- Turn right 90 ---
    // turn_90_right();
    // BT.print("Angle: "); BT.println(angle_x, 1);
    // while(1);

    // --- Turn 180 ---
    // turn_180_exact();
    // BT.print("Angle: "); BT.println(angle_x, 1);
    // while(1);

    // --- Forward 1 cell ---
    // forward_one_cell();
    // motorStop();
    // while(1);

    // --- Forward + 180 when wall detected (loop) ---
    // forward_one_cell();
    // if (wallFront()) { turn_180_exact(); }

    // --- Continuous gyro read ---
    // updateGyro();
    // static uint32_t lastGyroDebug = 0;
    // if (millis() - lastGyroDebug > 100) {
    //     lastGyroDebug = millis();
    //     BT.print("Gyro: "); BT.println(angle_x, 2);
    // }

    // --- FloodFill solver ---
    // solver();
    
    //testIRSensors();
    //testEncoder();
}
