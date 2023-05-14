//
// Created by pauld on 11.03.2023.
//

#include "joystick.h"

void setupJoystick(int ads, int i2caddr, int pin){
    wiringPiSetupPhys(); /* Setup von WiringPI */

    ads1115Setup(ads, i2caddr); //ADS

    joyPin = pin;
}

int doJoystick(){
    int val = analogRead(joyPin);

    /* constrain für Raspberry Pi */
    if(val <= 1000){
        val = 1000;
    }
    else if(val >= 30000){
        val = 30000;
    }

    return map(val, 1000, 30000, 20, 120);
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}