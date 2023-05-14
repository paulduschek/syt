//
// Created by pauld on 11.03.2023.
//
#include "servo.h"

void setupServo(int servoPin){
    pin = servoPin;
}

void writeServo(int input)
{
    file = fopen("/dev/servoblaster", "w");

    if(file == NULL)
    {
        printf("* [ ERROR ] Could not open file\n");
    }
    else
    {
        fprintf(file, "%d=%d\n", pin, input);
        fflush(file);
    }
}

void closeServo()
{
    fclose(file);
}