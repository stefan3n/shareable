#ifndef HardwareConfig
#define HardwareConfig

/*
    Any defines related to pinouts and other hardware related things should go here.
*/

#include "Arduino.h"

#define MOTOR_BREAK 6 // Pin for braking - both sides

#define MOTOR_ROTATION_LEFT 7
#define MOTOR_ROTATION_RIGHT 8

#define MOTOR_SPEED_PULSE_LEFT 2
#define MOTOR_SPEED_PULSE_RIGHT 4

#define MOTOR_PWM_LEFT 3
#define MOTOR_PWM_RIGHT 5

#define RIGHT_DIR_DEFAULT 1
#define LEFT_DIR_DEFAULT 0

#define TICKS_PER_REVOLUTION 98
#define WHEEL_RADIUS 14 //cm

#endif
