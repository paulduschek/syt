#include "servo.h"

#define SERVONR 7

void main(){
	setupServo(SERVONR);
	int input;
	while(1){
		printf("give 1-180\n");
		sscanf("%d", &input);
		if(input > 0 && input <= 180){
			writeServo(input);
		}
		else{
			printf("wrong in\n");
		}
	}
}