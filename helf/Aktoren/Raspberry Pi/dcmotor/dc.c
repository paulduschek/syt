#include "dc.h"

void setupDC(int ADS, int I2C_ADDR, int SPEED, int FWD, int BWD, int JOYSTICK){
	wiringPiSetupPhys();
	ads1115Setup(ADS, I2C_ADDR);
	
	speed = SPEED;
	forward = FWD;
	backward = BWD;
	
	pinMode(speed, OUTPUT);
	pinMode(forward, OUTPUT);
	pinMode(backward, OUTPUT);
	
	softPwmCreate(speed, 1, 100);
	joystick = JOYSTICK;
}

int map(int x, int in_min, int in_max, int out_min, int out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void doJoystick(){
	int value = analogRead(joystick);
	if(value >= 20230){
		int fwd_val = map(value, 20230, 32767, 1, 100;
		digitalWrite(forward, HIGH);
		digitalWrite(backward, LOW);
		softPwmWrite(speed, fwd_val);
	}
	else{
		int bwd_val = map(value, 0, 20229, 100, 1);
		digitalWrite(forward, LOW);
		digitalWrite(backward, HIGH);
		softPwmWrite(speed, bwd_val);
	}
}