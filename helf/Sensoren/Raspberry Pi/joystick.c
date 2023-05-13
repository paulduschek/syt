/**
-- simple joystick output to serial console x and y axis
*/

#include <wiringPi.h>
#include <stdio.h>
#include <ads1115.h>

#define ADS_ADDRESS 0x49
#define ADS_PIN 120

int x;
int y;

int main() {
    wiringPiSetupPhys();
    ads1115Setup(ADS_PIN, ADS_ADDRESS);

    while (true) {
        x = analogRead(ADS_PIN + 1);
        y = analogRead(ADS_PIN + 2);

        printf("x: %d, y; %d\n", x, y);
        delay(10000);
    }

    return 0;
}