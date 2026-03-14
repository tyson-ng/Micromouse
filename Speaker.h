// Speaker.h - Buzzer functions
#ifndef SPEAKER_H
#define SPEAKER_H

#include <Arduino.h>
#include "GlobalConfig.h"

void speaker_init() {
  pinMode(speakerPin, OUTPUT);
}

void playToTiTe() {
  tone(speakerPin, 440, 300);  // A4
  delay(400);
  tone(speakerPin, 880, 300);  // A5
  delay(400);
  tone(speakerPin, 659, 600);  // E5
  delay(700);
}

#endif // SPEAKER_H
