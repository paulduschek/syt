#include <stdio.h>
#include <wiringPi.h>
#include <ads1115.h>
#include <softPwm.h>

int speed;
int forward;
int backward;
int joystick;

void setupDC(int ADS, int I2C_ADDR, int SPEED, int FWD, int BWD, int JOYSTICK);
int map(int x, int in_min, int in_max, int out_min, int out_max);
void doJoystick();