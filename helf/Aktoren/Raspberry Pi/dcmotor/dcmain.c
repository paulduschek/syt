#include "dc.h"

#define ADS 120
#define I2C_ADDR 0x49
#define JOYSTICK ADS+2
#define SPEED 40
#define FWD 38
#define BWD 36

void main(){
	setupDC(ADS, I2C_ADDR, SPEED, FWD, BWD, JOYSTICK);
	while(1){
		doJoystick();
	}
}