// GlobalConfig.h - Pin definitions, constants & global variables
#ifndef GLOBAL_CONFIG_H
#define GLOBAL_CONFIG_H

#include <Arduino.h>
#include <stdlib.h>
#include <HardwareSerial.h>

// --- FloodFill scoring ---
const int TURN_SCORE   = 0;
const int TILE_SCORE   = 1;
const int STREAK_SCORE = 0;

#define STARTING_TARGET  1
#define STARTING_HEADING WEST
#define STARTING_X       0
#define STARTING_Y       0

#define LOWER_X_GOAL 7
#define LOWER_Y_GOAL 7
#define UPPER_X_GOAL 8
#define UPPER_Y_GOAL 8

#define MAZE_WIDTH   16
#define MAZE_HEIGHT  16
#define OUT_OF_BOUNDS -2
#define NOT_YET_SET  -1

// --- Enums ---
typedef enum Heading { NORTH, WEST, SOUTH, EAST } Heading;
typedef enum Action  { LEFT, FORWARD, RIGHT, IDLE } Action;

// --- Robot config ---
int stepMove = 1;

#define BT_BAUD 9600

// Gyro (SPI2)
#define GYRO_CS   PB12
#define GYRO_SCLK PB13
#define GYRO_MISO PB14
#define GYRO_MOSI PB15

// Speaker
const int speakerPin = PA10;

// IR sensors
const uint32_t EMIT[4] = { PA9, PB8, PA8, PB9 };
const uint32_t RECV[4] = { PA4, PA3, PA5, PA2 };
uint16_t vOn[4], vOff[4], IRd[4];

// Bluetooth
HardwareSerial BT(PB11, PB10);

// Motor pins
#define MLEFT_IN1    PA6
#define MLEFT_IN2    PA7
#define MRIGHT_IN3   PB0
#define MRIGHT_IN4   PB1
#define MAX_PWM      255

// Encoder pins
#define ENCLA PA15
#define ENCLB PB3
#define ENCRA PB6
#define ENCRB PB7

// Non-blocking delay
void wait_ms(uint32_t time_ms) {
    uint32_t start_time = millis();
    while (millis() - start_time < time_ms) { }
}

#endif // GLOBAL_CONFIG_H
