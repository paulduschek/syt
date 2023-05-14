#include "stepper.h"

int signals[8][4]={
	{0,0,0,1},
	{0,0,1,1},
	{0,0,1,0},
	{0,1,1,0},
	{0,1,0,0},
	{1,1,0,0},
	{1,0,0,0},
	{1,0,0,1}
}

int seq = 0;

void doSteps(int steps, bool forward, int velocity){
	for(int i = 0; i < steps; i++){
		if(forward){
			seq++;
			if(seq >= 8){
				seq = 0;
			}
		}
		else{
			seq--;
			if(seq <= 0){
				seq = 7;
			}
		}
		execSteps(signals[seq]);
		
		if(velocity > 1000){
			velocity = 1000;
		}
		else if(velocity <= 0){
			velocity = 1;
		}
		delay(1000/velocity);
	}
}

void execSteps(int out[4]){
	digitalWrite(pin1, out[0]);
	digitalWrite(pin2, out[1]);
	digitalWrite(pin3, out[2]);
	digitalWrite(pin4, out[3]);
}

void setupStepper(int PIN1, int PIN2, int PIN3, int PIN4){
	wiringPiSetupPhys();
	
	pin1 = PIN1;
	pin2 = PIN2;
	pin3 = PIN3;
	pin4 = PIN4;
	
	pinMode(pin1, OUTPUT);
	pinMode(pin2, OUTPUT);
	pinMode(pin3, OUTPUT);
	pinMode(pin4, OUTPUT);
}

// für Stepper mit Joystick
void doJoystick(){
	int val = analogRead(joystick);
	if(val > 30000){
		val = 30000;
	}
	else if(value < 1000){
		value = 1000;
	}
	if(value > 10000){
		seq++;
		if(seq >= 8){
			seq = 0;
		}
	}
	else{
		seq--;
		if(seq <= 0){
			seq = 7;
		}
	}
	execSteps(signals[seq]);
	
	int velocity = map(val, 1000, 30000, 1, 1000);
	delay(velocity/1000);
}