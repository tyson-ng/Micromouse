// Motors.h - Motor control and encoder reading (hardware timer)
#ifndef MOTORS_H
#define MOTORS_H

#include "GlobalConfig.h"
#include "HardwareTimer.h"

// Hardware timer objects for encoders
HardwareTimer *Encoder_Left_TIM2;
HardwareTimer *Encoder_Right_TIM4;

// Movement constants (encoder ticks)
const int32_t CELL_TICKS    = 2250;
const int32_t TURN90_TICKS  = 900;
const int32_t TURN180_TICKS = 1845;
const int PWM_FWD   = 70;
const int PWM_TURN  = 50;

// Curve turn config
float curve_ratio = 0.56; 
int TICKS_CURVE_90 = 3500;

// --- Encoder reading ---
// Read raw timer count, cast to int16_t for signed value
uint32_t getRawLeft() {
    return Encoder_Left_TIM2->getCount();
}

uint32_t getRawRight() {
    return Encoder_Right_TIM4->getCount();
}

// Macros that auto-read the timer on each access
#define countLeft  ((int16_t)getRawLeft())
#define countRight ((int16_t)getRawRight())

// --- Function declarations ---
void motor_init();
void motorLeft(int power);
void motorRight(int power);
void setMotor(int L, int R);
void motorStop();
void drive_brake();
void resetEncoder();
void testMotors(); 
void testEncoder();

// --- Init ---
void motor_init() {
  // Motor pins
  pinMode(MLEFT_IN1, OUTPUT);
  pinMode(MLEFT_IN2, OUTPUT);
  pinMode(MRIGHT_IN3, OUTPUT);
  pinMode(MRIGHT_IN4, OUTPUT);

  digitalWrite(MLEFT_IN1, HIGH);
  digitalWrite(MLEFT_IN2, HIGH);
  digitalWrite(MRIGHT_IN3, HIGH);
  digitalWrite(MRIGHT_IN4, HIGH);

  // Left encoder (TIM2 - PA15, PB3)
  // Need to disable JTAG and remap for PA15
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1;

  pinMode(PA15, INPUT_PULLUP);
  pinMode(PB3,  INPUT_PULLUP);

  Encoder_Left_TIM2 = new HardwareTimer(TIM2);
  Encoder_Left_TIM2->setOverflow(65535);

  TIM_HandleTypeDef *htim2 = Encoder_Left_TIM2->getHandle();
  TIM_Encoder_InitTypeDef sConfig2 = {0};
  sConfig2.EncoderMode = TIM_ENCODERMODE_TI12; 
  sConfig2.IC1Polarity = TIM_ICPOLARITY_FALLING; // Invert so forward = positive
  sConfig2.IC2Polarity = TIM_ICPOLARITY_RISING;
  
  HAL_TIM_Encoder_Init(htim2, &sConfig2);
  HAL_TIM_Encoder_Start(htim2, TIM_CHANNEL_ALL);
  Encoder_Left_TIM2->resume();

  // Right encoder (TIM4 - PB6, PB7)
  pinMode(PB6, INPUT_PULLUP);
  pinMode(PB7, INPUT_PULLUP);

  Encoder_Right_TIM4 = new HardwareTimer(TIM4);
  Encoder_Right_TIM4->setOverflow(65535);

  TIM_HandleTypeDef *htim4 = Encoder_Right_TIM4->getHandle();
  TIM_Encoder_InitTypeDef sConfig4 = {0};
  sConfig4.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig4.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig4.IC2Polarity = TIM_ICPOLARITY_RISING;

  HAL_TIM_Encoder_Init(htim4, &sConfig4);
  HAL_TIM_Encoder_Start(htim4, TIM_CHANNEL_ALL);
  Encoder_Right_TIM4->resume();
}

void resetEncoder() {
  Encoder_Left_TIM2->setCount(0);
  Encoder_Right_TIM4->setCount(0);
}

void testEncoder() {
    BT.print("L: ");
    BT.print(countLeft);
    BT.print(" | R: ");
    BT.println(countRight);
    delay(50);
}

// --- Motor drive functions ---

void motorLeft(int power) {
  bool forward = true;
  if (power < 0) { power = -power; forward = false; }
  if (power > MAX_PWM) power = MAX_PWM;

  if (forward) {
      analogWrite(MLEFT_IN1, MAX_PWM - power);
      analogWrite(MLEFT_IN2, MAX_PWM);
  }
  else {
      analogWrite(MLEFT_IN1, MAX_PWM);
      analogWrite(MLEFT_IN2, MAX_PWM - power);
  }
}

void motorRight(int power) {
  bool forward = true;
  if (power < 0) { power = -power; forward = false; }
  if (power > MAX_PWM) power = MAX_PWM;

  if (forward) {
      analogWrite(MRIGHT_IN3, MAX_PWM);
      analogWrite(MRIGHT_IN4, MAX_PWM - power);
  }
  else {
      analogWrite(MRIGHT_IN3, MAX_PWM - power);
      analogWrite(MRIGHT_IN4, MAX_PWM);
  }
}

void setMotor(int L, int R){
  motorLeft(L);
  motorRight(R);
}

void motorStop() { setMotor(0,0); }

void drive_brake(){
  digitalWrite(MLEFT_IN1,HIGH);
  digitalWrite(MLEFT_IN2,HIGH);
  digitalWrite(MRIGHT_IN3,HIGH);
  digitalWrite(MRIGHT_IN4,HIGH);

  analogWrite(MLEFT_IN1,255);
  analogWrite(MLEFT_IN2,255);
  analogWrite(MRIGHT_IN3,255);
  analogWrite(MRIGHT_IN4,255);
}

void testMotors() {
    BT.println("[TEST] Motors...");
    resetEncoder();
    motorLeft(150); motorRight(0);
    wait_ms(1000);
    motorStop();
    BT.print("L Value: "); BT.println(countLeft);
    wait_ms(500);
    
    resetEncoder();
    motorLeft(-150); motorRight(0);
    wait_ms(1000);
    motorStop();
    BT.print("L Value: "); BT.println(countLeft);
    wait_ms(500);
    
    BT.println("Done!");
}

#endif // MOTORS_H