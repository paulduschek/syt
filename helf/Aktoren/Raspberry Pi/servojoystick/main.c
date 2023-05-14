#include <stdio.h>
#include "servo.h"
#include "joystick.h"

#define PIN_SERVO 7
#define ADS 120 /* PIN für ADS */
#define PIN_JOYSTICK ADS+1 /* ADS+1: Pin A0 am ADS-Board usw. */

int main() {
    setupServo(PIN_SERVO);
    setupJoystick(ADS, 0x49, PIN_JOYSTICK);
    while (1){
        writeServo(doJoystick())
    }
}
