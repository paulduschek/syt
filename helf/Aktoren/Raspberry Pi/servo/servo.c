#include "servo.h"

#define PATH "/dev/servoblaster"

void setupServo(int servoNr){
	nr = servoNr;
	file = fopen(PATH, "w");
}

void writeServo(int input){
	if(file == null){
		printf("ERROR");
	}
	else{
		fprintf(file, "%d=%d", nr, input);
		fflush(file);
	}
}

void closeServo(){
	fclose(file);
}