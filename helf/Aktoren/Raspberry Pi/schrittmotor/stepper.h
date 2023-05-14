#include <wiringPi.h>
#include <stdbool.h>
#include <stdio.h>

int pin1;
int pin2;
int pin3;
int pin4;

void doSteps(int steps, bool forward, int velocity);
void execSteps(int out[4]);
void setupStepper(int PIN1, int PIN2, int PIN3, int PIN4);