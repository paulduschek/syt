#include <stdio.h>
#include <stdbool.h>
#include <ads1115.h>
#include <wiringPi.h>

#define PINBASE 120
#define ADS_ADDRESS 0x49

int main(){
	wiringPiSetupPhys();
	ads1115Setup(PINBASE, ADS_ADDRESS);
	
	while(true){
		int value = analogRead(PINBASE + 2);
		printf("value: &d\n", value);
		delay(1000);
	}
	
	return 0;
}