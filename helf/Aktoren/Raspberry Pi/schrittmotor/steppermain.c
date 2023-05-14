#include "stepper.h"

#define PIN1 38
#define PIN2 40
#define PIN3 36
#define PIN4 35

void main(){
	setupStepper(PIN1, PIN2, PIN3, PIN4);
	doSteps(700, true, 230);
	doSteps(700, false, 230);
	return 0;
}