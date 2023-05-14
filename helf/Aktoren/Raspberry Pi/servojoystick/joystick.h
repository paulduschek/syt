//
// Created by pauld on 11.03.2023.
//
#ifndef SMUEP5AHIT_JOYSTICK_H
#define SMUEP5AHIT_JOYSTICK_H

int joyPin;

int doJoystick();
void setupJoystick(int ads, int i2caddr, int pin);
int map(int x, int in_min, int in_max, int out_min, int out_max);

#endif //SMUEP5AHIT_JOYSTICK_H
