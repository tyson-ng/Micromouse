// Sensors.h - IR sensor reading and wall detection
#ifndef SENSORS_H
#define SENSORS_H

#include "GlobalConfig.h"

void ir_init();
void ir_read_once();
uint16_t readMedian(uint8_t pin, int samples = 3);
void debugPrintIR();

bool wallFront();
bool wallLeft();
bool wallRight();

// Fast median of 3 samples
uint16_t readMedian(uint8_t pin, int samples) {
    uint16_t a = analogRead(pin);
    uint16_t b = analogRead(pin);
    uint16_t c = analogRead(pin);

    if ((a <= b) && (a <= c)) return (b <= c) ? b : c;
    if ((b <= a) && (b <= c)) return (a <= c) ? a : c;
    return (a <= b) ? a : b;
}

void ir_init(){
  analogReadResolution(12);
  for(int i=0;i<4;i++){
    pinMode(EMIT[i],OUTPUT); digitalWrite(EMIT[i],LOW);
    pinMode(RECV[i],INPUT_ANALOG);
  }
}

void debugPrintIR() {
    char buf[64];
    int n = snprintf(buf, sizeof(buf),
        "[IR] SL:%u  SR:%u  FL:%u  FR:%u",
        IRd[0], IRd[1], IRd[2], IRd[3]
    );
    BT.write(buf, n);
    BT.write('\n');
}

// Read all 4 IR sensors sequentially
void ir_read_once() {
    const int LED_WAIT = 30; // us, wait for LED to stabilize

    // Side Left
    digitalWrite(EMIT[0], HIGH);
    delayMicroseconds(LED_WAIT);
    uint16_t v0 = readMedian(RECV[0], 10); 
    digitalWrite(EMIT[0], LOW);

    // Side Right
    digitalWrite(EMIT[1], HIGH);
    delayMicroseconds(LED_WAIT);
    uint16_t v1 = readMedian(RECV[1], 10);
    digitalWrite(EMIT[1], LOW);

    // Front Left
    digitalWrite(EMIT[2], HIGH);
    delayMicroseconds(LED_WAIT);
    uint16_t v2 = readMedian(RECV[2], 3);
    digitalWrite(EMIT[2], LOW);

    // Front Right
    digitalWrite(EMIT[3], HIGH);
    delayMicroseconds(LED_WAIT);
    uint16_t v3 = readMedian(RECV[3], 3);
    digitalWrite(EMIT[3], LOW);

    // Scale down
    IRd[0] = v0 / 100; // SL
    IRd[1] = v1 / 100; // SR
    IRd[2] = v2 / 100; // FL
    IRd[3] = v3 / 100; // FR
}

// Wall detection thresholds
bool wallFront(){ir_read_once(); return IRd[2] >=  30; }
bool wallRight(){ir_read_once(); return IRd[1] >=  28; }
bool wallLeft(){ir_read_once();  return IRd[0] >=  28; }

// Continuous sensor test (blocks forever, reset to exit)
void testIRSensors() {
    BT.println("--- SENSOR TEST ---");
    
    while (true) {
        ir_read_once();

        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 100) {
            lastPrint = millis();
            BT.print("L: ");  BT.print(IRd[0]); 
            BT.print(" | R: "); BT.print(IRd[1]);
            BT.print(" | FL: "); BT.print(IRd[2]);
            BT.print(" | FR: "); BT.println(IRd[3]);
        }
    }
}

#endif // SENSORS_H
