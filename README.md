# Micromouse

Autonomous maze-solving robot using FloodFill algorithm. Built on STM32F103C8T6 (Blue Pill), programmed with Arduino framework.

## Hardware

- **MCU:** STM32F103C8T6 (Blue Pill)
- **Motors:** 2x DC motors with quadrature encoders
- **Sensors:** 4x IR pairs (2 side, 2 front) for wall detection
- **Gyro:** MPU9250 via SPI
- **Comms:** HC-05 Bluetooth for debugging
- **Others:** Passive buzzer

## How it works

1. Robot enters the maze and reads walls using IR sensors
2. Wall data is stored and FloodFill recalculates distances every step
3. Robot picks the lowest-cost neighbor cell and moves there
4. PID controllers keep the robot centered between walls
5. After reaching the goal, it returns to start

## File overview

```
Main.ino          - setup() and loop()
GlobalConfig.h    - Pin definitions, constants
Sensors.h         - IR reading, wall detection
Motors.h          - Motor control, encoder config (hardware timers)
PID.h             - PID controllers (wall-following, velocity)
Gyro.h            - MPU9250 gyro setup and angle integration
RobotMotion.h     - Movement functions (forward, turn 90, turn 180)
FloodFill.h       - Maze solver, queue, wall mapping
Speaker.h         - Buzzer sounds
```

## Upload

1. Open `Main.ino` in Arduino IDE
2. Board: Generic STM32F103C8
3. Upload via ST-Link or Serial

## Debug

Connect Bluetooth at 9600 baud to see live sensor data and decisions:
```
L:28 R:31 | PID:2.5 | Spd:72/67
[FF] DECISION = FORWARD
```

## License

MIT
