#include <wiringPi.h>

#define BUTTON 0
#define LED 3

int buttonState;

int main(){
	wiringPiSetupPhys();
	
	pinMode(BUTTON, INPUT);
	pinMode(LED, OUTPUT);
	
	while(true){
		buttonState = digitalRead(BUTTON);
		if(buttonState == 1){
			digitalWrite(LED, HIGH);
		}
		else if(buttonState == 0){
			digitalWrite(LED, LOW);
		}
	}
	return 0;
}